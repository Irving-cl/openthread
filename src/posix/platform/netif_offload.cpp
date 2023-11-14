
#include "openthread-posix-config.h"
#include "platform-posix.h"
#include "platform-posix-offload.h"

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#ifdef __linux__
#include <linux/if_link.h>
#include <linux/if_tun.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#endif // __linux__
#include <math.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#if defined(__APPLE__) || defined(__NetBSD__) || defined(__FreeBSD__)
#include <netinet/in.h>
#if defined(__APPLE__) || defined(__FreeBSD__)
#include <net/if_var.h>
#endif // defined(__APPLE__) || defined(__FreeBSD__)
#include <net/route.h>
#include <netinet6/in6_var.h>
#if defined(__APPLE__) || defined(__FreeBSD__)
// the prf_ra structure is defined inside another structure (in6_prflags), and C++
//   treats that as out of scope if another structure tries to use it -- this (slightly gross)
//   workaround makes us dependent on our definition remaining in sync (at least the size of it),
//   so we add a compile-time check that will fail if the SDK ever changes
//
// our definition of the struct:
struct prf_ra
{
    u_char onlink : 1;
    u_char autonomous : 1;
    u_char reserved : 6;
} prf_ra;
// object that contains the SDK's version of the structure:
struct in6_prflags compile_time_check_prflags;
// compile time check to make sure they're the same size:
extern int
    compile_time_check_struct_prf_ra[(sizeof(struct prf_ra) == sizeof(compile_time_check_prflags.prf_ra)) ? 1 : -1];
#endif
#include <net/if_dl.h>    // struct sockaddr_dl
#include <netinet6/nd6.h> // ND6_INFINITE_LIFETIME

#ifdef __APPLE__
#if OPENTHREAD_POSIX_CONFIG_MACOS_TUN_OPTION == OT_POSIX_CONFIG_MACOS_UTUN
#include <net/if_utun.h>
#endif

#if OPENTHREAD_POSIX_CONFIG_MACOS_TUN_OPTION == OT_POSIX_CONFIG_MACOS_TUN
#include <sys/ioccom.h>
// FIX ME: include the tun_ioctl.h file (challenging, as it's location depends on where the developer puts it)
#define TUNSIFHEAD _IOW('t', 96, int)
#define TUNGIFHEAD _IOR('t', 97, int)
#endif

#include <sys/kern_control.h>
#endif // defined(__APPLE__)

#if defined(__NetBSD__) || defined(__FreeBSD__)
#include <net/if_tun.h>
#endif // defined(__NetBSD__) || defined(__FreeBSD__)

#endif // defined(__APPLE__) || defined(__NetBSD__) || defined(__FreeBSD__)

#include <openthread/border_router.h>
#include <openthread/icmp6.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/logging.h>
#include <openthread/message.h>
#include <openthread/nat64.h>
#include <openthread/netdata.h>
#include <openthread/platform/border_routing.h>
#include <openthread/platform/misc.h>
#include <openthread/platform/offload.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "net/ip6_address.hpp"

#include "resolver.hpp"

unsigned int gNetifIndex = 0;
char         gNetifName[IFNAMSIZ];
#if OPENTHREAD_CONFIG_NAT64_TRANSLATOR_ENABLE
static otIp4Cidr sActiveNat64Cidr;
#endif

const char *otSysGetThreadNetifName(void) { return gNetifName; }

unsigned int otSysGetThreadNetifIndex(void) { return gNetifIndex; }

#if OPENTHREAD_CONFIG_PLATFORM_NETIF_ENABLE
#if OPENTHREAD_POSIX_CONFIG_FIREWALL_ENABLE
#include "firewall.hpp"
#endif
#include "posix/platform/ip6_utils.hpp"

using namespace ot::Posix::Ip6Utils;

#ifndef OPENTHREAD_POSIX_TUN_DEVICE

#ifdef __linux__
#define OPENTHREAD_POSIX_TUN_DEVICE "/dev/net/tun"
#elif defined(__NetBSD__) || defined(__FreeBSD__)
#define OPENTHREAD_POSIX_TUN_DEVICE "/dev/tun0"
#elif defined(__APPLE__)
#if OPENTHREAD_POSIX_CONFIG_MACOS_TUN_OPTION == OT_POSIX_CONFIG_MACOS_UTUN
#define OPENTHREAD_POSIX_TUN_DEVICE // not used - calculated dynamically
#elif OPENTHREAD_POSIX_CONFIG_MACOS_TUN_OPTION == OT_POSIX_CONFIG_MACOS_TUN
#define OPENTHREAD_POSIX_TUN_DEVICE "/dev/tun0"
#endif
#else
// good luck -- untested platform...
#define OPENTHREAD_POSIX_TUN_DEVICE "/dev/net/tun"
#endif

#endif // OPENTHREAD_POSIX_TUN_DEVICE

#if defined(__linux__)
static uint32_t sNetlinkSequence = 0; ///< Netlink message sequence.
#endif

static int sTunFd     = -1; ///< Used to exchange IPv6 packets.
static int sIpFd      = -1; ///< Used to manage IPv6 stack on Thread interface.
static int sNetlinkFd = -1; ///< Used to receive netlink events.

static constexpr size_t kMaxIp6Size = OPENTHREAD_CONFIG_IP6_MAX_DATAGRAM_LENGTH;
#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
static bool sIsSyncingState = false;
#endif

#if defined(__linux__)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

static struct rtattr *AddRtAttr(struct nlmsghdr *aHeader,
                                uint32_t         aMaxLen,
                                uint8_t          aType,
                                const void      *aData,
                                uint8_t          aLen)
{
    uint8_t        len = RTA_LENGTH(aLen);
    struct rtattr *rta;

    assert(NLMSG_ALIGN(aHeader->nlmsg_len) + RTA_ALIGN(len) <= aMaxLen);
    OT_UNUSED_VARIABLE(aMaxLen);

    rta           = (struct rtattr *)((char *)(aHeader) + NLMSG_ALIGN((aHeader)->nlmsg_len));
    rta->rta_type = aType;
    rta->rta_len  = len;
    if (aLen)
    {
        memcpy(RTA_DATA(rta), aData, aLen);
    }
    aHeader->nlmsg_len = NLMSG_ALIGN(aHeader->nlmsg_len) + RTA_ALIGN(len);

    return rta;
}

void AddRtAttrUint32(struct nlmsghdr *aHeader, uint32_t aMaxLen, uint8_t aType, uint32_t aData)
{
    AddRtAttr(aHeader, aMaxLen, aType, &aData, sizeof(aData));
}

static void SetAddrGenModeToNone(void)
{
    struct
    {
        struct nlmsghdr  nh;
        struct ifinfomsg ifi;
        char             buf[512];
    } req;

    const uint8_t mode = IN6_ADDR_GEN_MODE_NONE;

    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len   = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    req.nh.nlmsg_type  = RTM_NEWLINK;
    req.nh.nlmsg_pid   = 0;
    req.nh.nlmsg_seq   = ++sNetlinkSequence;

    req.ifi.ifi_index  = static_cast<int>(gNetifIndex);
    req.ifi.ifi_change = 0xffffffff;
    req.ifi.ifi_flags  = IFF_MULTICAST | IFF_NOARP;

    {
        struct rtattr *afSpec  = AddRtAttr(&req.nh, sizeof(req), IFLA_AF_SPEC, 0, 0);
        struct rtattr *afInet6 = AddRtAttr(&req.nh, sizeof(req), AF_INET6, 0, 0);
        struct rtattr *inet6AddrGenMode =
            AddRtAttr(&req.nh, sizeof(req), IFLA_INET6_ADDR_GEN_MODE, &mode, sizeof(mode));

        afInet6->rta_len += inet6AddrGenMode->rta_len;
        afSpec->rta_len += afInet6->rta_len;
    }

    if (send(sNetlinkFd, &req, req.nh.nlmsg_len, 0) != -1)
    {
        otLogInfoPlat("[netif] Sent request#%u to set addr_gen_mode to %d", sNetlinkSequence, mode);
    }
    else
    {
        otLogWarnPlat("[netif] Failed to send request#%u to set addr_gen_mode to %d", sNetlinkSequence, mode);
    }
}

static void SetLinkState(otInstance *aInstance, bool aState)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError      error = OT_ERROR_NONE;
    struct ifreq ifr;
    bool         ifState = false;

    VerifyOrExit(sIpFd >= 0);
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, gNetifName, sizeof(ifr.ifr_name));
    VerifyOrExit(ioctl(sIpFd, SIOCGIFFLAGS, &ifr) == 0, perror("ioctl"); error = OT_ERROR_FAILED);

    ifState = ((ifr.ifr_flags & IFF_UP) == IFF_UP) ? true : false;

    otLogNotePlat("[netif] Changing interface state to %s%s.", aState ? "up" : "down",
                  (ifState == aState) ? " (already done, ignoring)" : "");

    if (ifState != aState)
    {
        otLogNotePlat("!!! ifr_name:%s, state:%d", ifr.ifr_name, aState);
        ifr.ifr_flags = aState ? (ifr.ifr_flags | IFF_UP) : (ifr.ifr_flags & ~IFF_UP);
        VerifyOrExit(ioctl(sIpFd, SIOCSIFFLAGS, &ifr) == 0, perror("ioctl"); error = OT_ERROR_FAILED);
#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
        // wait for RTM_NEWLINK event before processing notification from kernel to avoid infinite loop
        sIsSyncingState = true;
#endif
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnPlat("[netif] Failed to update state %s", otThreadErrorToString(error));
    }
}

static void platformConfigureTunDevice(otPlatformConfig *aPlatformConfig)
{
    struct ifreq ifr;
    const char  *interfaceName;

    sTunFd = open(OPENTHREAD_POSIX_TUN_DEVICE, O_RDWR | O_CLOEXEC | O_NONBLOCK);
    VerifyOrDie(sTunFd >= 0, OT_EXIT_ERROR_ERRNO);

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
    if (!aPlatformConfig->mPersistentInterface)
    {
        ifr.ifr_flags |= static_cast<short>(IFF_TUN_EXCL);
    }

    interfaceName = aPlatformConfig->mInterfaceName;
    if (interfaceName)
    {
        VerifyOrDie(strlen(interfaceName) < IFNAMSIZ, OT_EXIT_INVALID_ARGUMENTS);

        strncpy(ifr.ifr_name, interfaceName, IFNAMSIZ);
    }
    else
    {
        strncpy(ifr.ifr_name, "wpan%d", IFNAMSIZ);
    }

    VerifyOrDie(ioctl(sTunFd, TUNSETIFF, static_cast<void *>(&ifr)) == 0, OT_EXIT_ERROR_ERRNO);

    strncpy(gNetifName, ifr.ifr_name, sizeof(gNetifName));
    otLogInfoPlat("Netif name:%s", gNetifName);
    if (aPlatformConfig->mPersistentInterface)
    {
        VerifyOrDie(ioctl(sTunFd, TUNSETPERSIST, 1) == 0, OT_EXIT_ERROR_ERRNO);
        // Set link down to reset the tun configuration.
        // This will drop all existing IP addresses on the interface.
        SetLinkState(gInstance, false);
    }

    VerifyOrDie(ioctl(sTunFd, TUNSETLINK, ARPHRD_NONE) == 0, OT_EXIT_ERROR_ERRNO);

    ifr.ifr_mtu = static_cast<int>(kMaxIp6Size);
    VerifyOrDie(ioctl(sIpFd, SIOCSIFMTU, static_cast<void *>(&ifr)) == 0, OT_EXIT_ERROR_ERRNO);

    // SetLinkState(gInstance, true);
}
#endif // defined(__linux__)

static void platformConfigureNetLink(void)
{
#if defined(__linux__)
    sNetlinkFd = SocketWithCloseExec(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE, kSocketNonBlock);
#elif defined(__APPLE__) || defined(__NetBSD__) || defined(__FreeBSD__)
    sNetlinkFd = SocketWithCloseExec(PF_ROUTE, SOCK_RAW, 0, kSocketNonBlock);
#else
#error "!! Unknown platform !!"
#endif
    VerifyOrDie(sNetlinkFd >= 0, OT_EXIT_ERROR_ERRNO);

#if defined(SOL_NETLINK)
    {
        int enable = 1;

#if defined(NETLINK_EXT_ACK)
        if (setsockopt(sNetlinkFd, SOL_NETLINK, NETLINK_EXT_ACK, &enable, sizeof(enable)) != 0)
        {
            otLogWarnPlat("[netif] Failed to enable NETLINK_EXT_ACK: %s", strerror(errno));
        }
#endif
#if defined(NETLINK_CAP_ACK)
        if (setsockopt(sNetlinkFd, SOL_NETLINK, NETLINK_CAP_ACK, &enable, sizeof(enable)) != 0)
        {
            otLogWarnPlat("[netif] Failed to enable NETLINK_CAP_ACK: %s", strerror(errno));
        }
#endif
    }
#endif

#if defined(__linux__)
    {
        struct sockaddr_nl sa;

        memset(&sa, 0, sizeof(sa));
        sa.nl_family = AF_NETLINK;
        sa.nl_groups = RTMGRP_LINK | RTMGRP_IPV6_IFADDR;
        VerifyOrDie(bind(sNetlinkFd, reinterpret_cast<struct sockaddr *>(&sa), sizeof(sa)) == 0, OT_EXIT_ERROR_ERRNO);
    }
#endif

#if defined(__APPLE__) || defined(__NetBSD__) || defined(__FreeBSD__)
    {
        int status;
#ifdef ROUTE_FILTER
        unsigned int msgfilter = ROUTE_FILTER(RTM_IFINFO) | ROUTE_FILTER(RTM_NEWADDR) | ROUTE_FILTER(RTM_DELADDR) |
                                 ROUTE_FILTER(RTM_NEWMADDR) | ROUTE_FILTER(RTM_DELMADDR);
#define FILTER_CMD ROUTE_MSGFILTER
#define FILTER_ARG msgfilter
#define FILTER_ARG_SZ sizeof(msgfilter)
#endif
#ifdef RO_MSGFILTER
        uint8_t msgfilter[] = {RTM_IFINFO, RTM_NEWADDR, RTM_DELADDR};
#define FILTER_CMD RO_MSGFILTER
#define FILTER_ARG msgfilter
#define FILTER_ARG_SZ sizeof(msgfilter)
#endif
#if defined(ROUTE_FILTER) || defined(RO_MSGFILTER)
        status = setsockopt(sNetlinkFd, AF_ROUTE, FILTER_CMD, FILTER_ARG, FILTER_ARG_SZ);
        VerifyOrDie(status == 0, OT_EXIT_ERROR_ERRNO);
#endif
        status = fcntl(sNetlinkFd, F_SETFL, O_NONBLOCK);
        VerifyOrDie(status == 0, OT_EXIT_ERROR_ERRNO);
    }
#endif // defined(__APPLE__) || defined(__NetBSD__) || defined(__FreeBSD__)
}

void platformNetifInit(otPlatformConfig *aPlatformConfig)
{
    sIpFd = SocketWithCloseExec(AF_INET6, SOCK_DGRAM, IPPROTO_IP, kSocketNonBlock);
    VerifyOrDie(sIpFd >= 0, OT_EXIT_ERROR_ERRNO);

    platformConfigureNetLink();
    platformConfigureTunDevice(aPlatformConfig);

    gNetifIndex = if_nametoindex(gNetifName);
    VerifyOrDie(gNetifIndex > 0, OT_EXIT_FAILURE);

#if OPENTHREAD_POSIX_USE_MLD_MONITOR
    mldListenerInit();
#endif

#if __linux__
    SetAddrGenModeToNone();
#endif
}

static void clearAllAddresses(void)
{

}

static void UpdateUnicastLinux(otInstance *aInstance, const otNetifAddress &aAddressInfo, bool aIsAdded)
{
    OT_UNUSED_VARIABLE(aInstance);

    struct
    {
        struct nlmsghdr  nh;
        struct ifaddrmsg ifa;
        char             buf[512];
    } req;

    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len   = NLMSG_LENGTH(sizeof(struct ifaddrmsg));
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK | NLM_F_CREATE | NLM_F_EXCL;
    req.nh.nlmsg_type  = aIsAdded ? RTM_NEWADDR : RTM_DELADDR;
    req.nh.nlmsg_pid   = 0;
    req.nh.nlmsg_seq   = ++sNetlinkSequence;

    req.ifa.ifa_family    = AF_INET6;
    req.ifa.ifa_prefixlen = aAddressInfo.mPrefixLength;
    req.ifa.ifa_flags     = IFA_F_NODAD;
    req.ifa.ifa_scope     = aAddressInfo.mScopeOverride;
    req.ifa.ifa_index     = gNetifIndex;

    AddRtAttr(&req.nh, sizeof(req), IFA_LOCAL, &aAddressInfo.mAddress, sizeof(aAddressInfo.mAddress));

    if (!aAddressInfo.mPreferred)
    {
        struct ifa_cacheinfo cacheinfo;

        memset(&cacheinfo, 0, sizeof(cacheinfo));
        cacheinfo.ifa_valid = UINT32_MAX;

        AddRtAttr(&req.nh, sizeof(req), IFA_CACHEINFO, &cacheinfo, sizeof(cacheinfo));
    }

#if OPENTHREAD_POSIX_CONFIG_INSTALL_OMR_ROUTES_ENABLE
    if (IsOmrAddress(aInstance, aAddressInfo))
    {
        // Remove prefix route for OMR address if `OPENTHREAD_POSIX_CONFIG_INSTALL_OMR_ROUTES_ENABLE` is enabled to
        // avoid having two routes.
        if (aIsAdded)
        {
            AddRtAttrUint32(&req.nh, sizeof(req), IFA_FLAGS, IFA_F_NOPREFIXROUTE);
        }
    }
    else
#endif
    {
#if OPENTHREAD_POSIX_CONFIG_NETIF_PREFIX_ROUTE_METRIC > 0
        if (aAddressInfo.mScopeOverride > ot::Ip6::Address::kLinkLocalScope)
        {
            AddRtAttrUint32(&req.nh, sizeof(req), IFA_RT_PRIORITY, OPENTHREAD_POSIX_CONFIG_NETIF_PREFIX_ROUTE_METRIC);
        }
#endif
    }

    if (send(sNetlinkFd, &req, req.nh.nlmsg_len, 0) != -1)
    {
        otLogInfoPlat("[netif] Sent request#%u to %s %s/%u", sNetlinkSequence, (aIsAdded ? "add" : "remove"),
                      Ip6AddressString(&aAddressInfo.mAddress).AsCString(), aAddressInfo.mPrefixLength);
    }
    else
    {
        otLogWarnPlat("[netif] Failed to send request#%u to %s %s/%u", sNetlinkSequence, (aIsAdded ? "add" : "remove"),
                      Ip6AddressString(&aAddressInfo.mAddress).AsCString(), aAddressInfo.mPrefixLength);
    }
}

static void processAddressChange(const otNetifAddress *aAddressArray, uint8_t aNumAddr, void *aContext)
{
    // if (aAddressInfo->mAddress->mFields.m8[0] == 0xff)
    // {
    //     UpdateMulticast(static_cast<otInstance *>(aContext), *aAddressInfo->mAddress, aIsAdded);
    // }
    // else
    // {
    //     UpdateUnicast(static_cast<otInstance *>(aContext), *aAddressInfo, aIsAdded);
    // }
    (void)aAddressArray;
    (void)aNumAddr;
    (void)aContext;

    clearAllAddresses();

    for (uint8_t i = 0; i < aNumAddr; i++)
    {
        UpdateUnicastLinux(nullptr, aAddressArray[i], true);
    }
}

void platformNetifSetUp(void)
{
    OT_ASSERT(gInstance != nullptr);

    // otIp6SetReceiveFilterEnabled(gInstance, true);
}

static void processTransmit(otInstance *aInstance)
{
    ssize_t    rval;
    uint8_t    packet[kMaxIp6Size];
    otError    error  = OT_ERROR_NONE;
    size_t     offset = 0;
#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE && OPENTHREAD_CONFIG_NAT64_TRANSLATOR_ENABLE
    bool isIp4 = false;
#endif

    assert(gInstance == aInstance);

    rval = read(sTunFd, packet, sizeof(packet));
    VerifyOrExit(rval > 0, error = OT_ERROR_FAILED);

#if defined(__APPLE__) || defined(__NetBSD__) || defined(__FreeBSD__)
    // BSD tunnel drivers have (for legacy reasons), may have a 4-byte header on them
    if ((rval >= 4) && (packet[0] == 0) && (packet[1] == 0))
    {
        rval -= 4;
        offset = 4;
    }
#endif

// #if OPENTHREAD_POSIX_LOG_TUN_PACKETS
    otLogInfoPlat("[netif] Packet to NCP (%hu bytes)", static_cast<uint16_t>(rval));
    otDumpInfoPlat("", &packet[offset], static_cast<size_t>(rval));
// #endif
    platformNetifOffloadSendIp6(packet, rval);

exit:

    if (error != OT_ERROR_NONE)
    {
        if (error == OT_ERROR_DROP)
        {
            otLogInfoPlat("[netif] Message dropped by Thread");
        }
        else
        {
            otLogWarnPlat("[netif] Failed to transmit, error:%s", otThreadErrorToString(error));
        }
    }
}

static void processNetifLinkEvent(otInstance *aInstance, struct nlmsghdr *aNetlinkMessage)
{
    struct ifinfomsg *ifinfo = reinterpret_cast<struct ifinfomsg *>(NLMSG_DATA(aNetlinkMessage));
    otError           error  = OT_ERROR_NONE;
    bool              isUp;
    (void)aInstance;

    VerifyOrExit(ifinfo->ifi_index == static_cast<int>(gNetifIndex) && (ifinfo->ifi_change & IFF_UP));

    isUp = ((ifinfo->ifi_flags & IFF_UP) != 0);

    otLogInfoPlat("[netif] Host netif is %s", isUp ? "up" : "down");

#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
    if (sIsSyncingState)
    {
        // VerifyOrExit(isUp == otIp6IsEnabled(aInstance),
        //              otLogWarnPlat("[netif] Host netif state notification is unexpected (ignore)"));
        sIsSyncingState = false;
    }
    // else
#endif
    //     if (isUp != otIp6IsEnabled(aInstance))
    // {
    //     SuccessOrExit(error = otIp6SetEnabled(aInstance, isUp));
    //     otLogInfoPlat("[netif] Succeeded to sync netif state with host");
    // }

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE && OPENTHREAD_CONFIG_NAT64_TRANSLATOR_ENABLE
    if (isUp && sActiveNat64Cidr.mLength > 0)
    {
        // Recover NAT64 route.
        AddIp4Route(sActiveNat64Cidr, kNat64RoutePriority);
    }
#endif

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnPlat("[netif] Failed to sync netif state with host: %s", otThreadErrorToString(error));
    }
}

#define ERR_RTA(errmsg, requestPayloadLength) \
    ((struct rtattr *)((char *)(errmsg)) + NLMSG_ALIGN(sizeof(struct nlmsgerr)) + NLMSG_ALIGN(requestPayloadLength))

// The format of NLMSG_ERROR is described below:
//
// ----------------------------------------------
// | struct nlmsghdr - response header          |
// ----------------------------------------------------------------
// |    int error                               |                 |
// ---------------------------------------------| struct nlmsgerr |
// | struct nlmsghdr - original request header  |                 |
// ----------------------------------------------------------------
// | ** optionally (1) payload of the request   |
// ----------------------------------------------
// | ** optionally (2) extended ACK attrs       |
// ----------------------------------------------
//
static void HandleNetlinkResponse(struct nlmsghdr *msg)
{
    const struct nlmsgerr *err;
    const char            *errorMsg;
    size_t                 rtaLength;
    size_t                 requestPayloadLength = 0;
    uint32_t               requestSeq           = 0;

    if (msg->nlmsg_len < NLMSG_LENGTH(sizeof(struct nlmsgerr)))
    {
        otLogWarnPlat("[netif] Truncated netlink reply of request#%u", requestSeq);
        ExitNow();
    }

    err        = reinterpret_cast<const nlmsgerr *>(NLMSG_DATA(msg));
    requestSeq = err->msg.nlmsg_seq;

    if (err->error == 0)
    {
        otLogInfoPlat("[netif] Succeeded to process request#%u", requestSeq);
        ExitNow();
    }

    // For rtnetlink, `abs(err->error)` maps to values of `errno`.
    // But this is not a requirement in RFC 3549.
    errorMsg = strerror(abs(err->error));

    // The payload of the request is omitted if NLM_F_CAPPED is set
    if (!(msg->nlmsg_flags & NLM_F_CAPPED))
    {
        requestPayloadLength = NLMSG_PAYLOAD(&err->msg, 0);
    }

    rtaLength = NLMSG_PAYLOAD(msg, sizeof(struct nlmsgerr)) - requestPayloadLength;

    for (struct rtattr *rta = ERR_RTA(err, requestPayloadLength); RTA_OK(rta, rtaLength);
         rta                = RTA_NEXT(rta, rtaLength))
    {
        if (rta->rta_type == NLMSGERR_ATTR_MSG)
        {
            errorMsg = reinterpret_cast<const char *>(RTA_DATA(rta));
            break;
        }
        else
        {
            otLogDebgPlat("[netif] Ignoring netlink response attribute %d (request#%u)", rta->rta_type, requestSeq);
        }
    }

    otLogWarnPlat("[netif] Failed to process request#%u: %s", requestSeq, errorMsg);

exit:
    return;
}


static void logAddrEvent(bool isAdd, const ot::Ip6::Address &aAddress, otError error)
{
    OT_UNUSED_VARIABLE(aAddress);

    if ((error == OT_ERROR_NONE) || ((isAdd) && (error == OT_ERROR_ALREADY || error == OT_ERROR_REJECTED)) ||
        ((!isAdd) && (error == OT_ERROR_NOT_FOUND || error == OT_ERROR_REJECTED)))
    {
        otLogInfoPlat("[netif] %s [%s] %s%s", isAdd ? "ADD" : "DEL", aAddress.IsMulticast() ? "M" : "U",
                      aAddress.ToString().AsCString(),
                      error == OT_ERROR_ALREADY     ? " (already subscribed, ignored)"
                      : error == OT_ERROR_REJECTED  ? " (rejected)"
                      : error == OT_ERROR_NOT_FOUND ? " (not found, ignored)"
                                                    : "");
    }
    else
    {
        otLogWarnPlat("[netif] %s [%s] %s failed (%s)", isAdd ? "ADD" : "DEL", aAddress.IsMulticast() ? "M" : "U",
                      aAddress.ToString().AsCString(), otThreadErrorToString(error));
    }
}

static void processNetifAddrEvent(otInstance *aInstance, struct nlmsghdr *aNetlinkMessage)
{
    (void)aInstance;
    struct ifaddrmsg   *ifaddr = reinterpret_cast<struct ifaddrmsg *>(NLMSG_DATA(aNetlinkMessage));
    size_t              rtaLength;
    otError             error = OT_ERROR_NONE;
    struct sockaddr_in6 addr6;

    VerifyOrExit(ifaddr->ifa_index == static_cast<unsigned int>(gNetifIndex) && ifaddr->ifa_family == AF_INET6);

    rtaLength = IFA_PAYLOAD(aNetlinkMessage);

    for (struct rtattr *rta = reinterpret_cast<struct rtattr *>(IFA_RTA(ifaddr)); RTA_OK(rta, rtaLength);
         rta                = RTA_NEXT(rta, rtaLength))
    {
        switch (rta->rta_type)
        {
        case IFA_ADDRESS:
        case IFA_LOCAL:
        case IFA_BROADCAST:
        case IFA_ANYCAST:
        case IFA_MULTICAST:
        {
            ot::Ip6::Address addr;
            memcpy(&addr, RTA_DATA(rta), sizeof(addr));

            memset(&addr6, 0, sizeof(addr6));
            addr6.sin6_family = AF_INET6;
            memcpy(&addr6.sin6_addr, RTA_DATA(rta), sizeof(addr6.sin6_addr));

            if (aNetlinkMessage->nlmsg_type == RTM_NEWADDR)
            {
                otLogInfoPlat("new addr");
                if (!addr.IsMulticast())
                {
                    otNetifAddress netAddr;

                    netAddr.mAddress      = addr;
                    netAddr.mPrefixLength = ifaddr->ifa_prefixlen;

                    // error = otIp6AddUnicastAddress(aInstance, &netAddr);
                }
                else
                {
                    otNetifMulticastAddress netAddr;

                    netAddr.mAddress = addr;

                    // error = otIp6SubscribeMulticastAddress(aInstance, &addr);
                }

                logAddrEvent(/* isAdd */ true, addr, error);
                if (error == OT_ERROR_ALREADY || error == OT_ERROR_REJECTED)
                {
                    error = OT_ERROR_NONE;
                }

                SuccessOrExit(error);
            }
            else if (aNetlinkMessage->nlmsg_type == RTM_DELADDR)
            {
                otLogInfoPlat("del addr");
                if (!addr.IsMulticast())
                {
                    // error = otIp6RemoveUnicastAddress(aInstance, &addr);
                }
                else
                {
                    // error = otIp6UnsubscribeMulticastAddress(aInstance, &addr);
                }

                logAddrEvent(/* isAdd */ false, addr, error);
                if (error == OT_ERROR_NOT_FOUND || error == OT_ERROR_REJECTED)
                {
                    error = OT_ERROR_NONE;
                }

                SuccessOrExit(error);
            }
            else
            {
                continue;
            }
            break;
        }

        default:
            otLogDebgPlat("[netif] Unexpected address type (%d).", (int)rta->rta_type);
            break;
        }
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnPlat("[netif] Failed to process event, error:%s", otThreadErrorToString(error));
    }
}

static void processNetlinkEvent(otInstance *aInstance)
{
    const size_t kMaxNetifEvent = 8192;
    ssize_t      length;

    union
    {
#if defined(__linux__)
        nlmsghdr nlMsg;
#else
        rt_msghdr rtMsg;
#endif
        char buffer[kMaxNetifEvent];
    } msgBuffer;

    length = recv(sNetlinkFd, msgBuffer.buffer, sizeof(msgBuffer.buffer), 0);

    VerifyOrExit(length > 0);

#if defined(__linux__)
    for (struct nlmsghdr *msg = &msgBuffer.nlMsg; NLMSG_OK(msg, static_cast<size_t>(length));
         msg                  = NLMSG_NEXT(msg, length))
    {
#else
    {
        // BSD sends one message per read to routing socket (see route.c, monitor command)
        struct rt_msghdr *msg;

        msg = &msgBuffer.rtMsg;

#define nlmsg_type rtm_type

#endif
        switch (msg->nlmsg_type)
        {
        case RTM_NEWADDR:
        case RTM_DELADDR:
            processNetifAddrEvent(aInstance, msg);
            break;

#if defined(RTM_NEWLINK) && defined(RTM_DELLINK)
        case RTM_NEWLINK:
        case RTM_DELLINK:
            processNetifLinkEvent(aInstance, msg);
            break;
#endif

#if defined(RTM_NEWMADDR) && defined(RTM_DELMADDR)
        case RTM_NEWMADDR:
        case RTM_DELMADDR:
            processNetifAddrEvent(aInstance, msg);
            break;
#endif
#if !defined(__linux__)
        case RTM_IFINFO:
            processNetifInfoEvent(aInstance, msg);
            break;

#else
        case NLMSG_ERROR:
            HandleNetlinkResponse(msg);
            break;
#endif

#if defined(ROUTE_FILTER) || defined(RO_MSGFILTER) || defined(__linux__)
        default:
            otLogWarnPlat("[netif] Unhandled/Unexpected netlink/route message (%d).", (int)msg->nlmsg_type);
            break;
#else
            // this platform doesn't support filtering, so we expect messages of other types...we just ignore them
#endif
        }
    }

exit:
    return;
}

void platformNetifProcess(const otSysMainloopContext *aContext)
{
    assert(aContext != nullptr);
    VerifyOrExit(gNetifIndex > 0);

    if (FD_ISSET(sTunFd, &aContext->mErrorFdSet))
    {
        close(sTunFd);
        DieNow(OT_EXIT_FAILURE);
    }

    if (FD_ISSET(sNetlinkFd, &aContext->mErrorFdSet))
    {
        close(sNetlinkFd);
        DieNow(OT_EXIT_FAILURE);
    }

    if (FD_ISSET(sTunFd, &aContext->mReadFdSet))
    {
        processTransmit(gInstance);
    }

    if (FD_ISSET(sNetlinkFd, &aContext->mReadFdSet))
    {
        processNetlinkEvent(gInstance);
    }

exit:
    return;
}

void platformNetifUpdateFdSet(otSysMainloopContext *aContext)
{
    VerifyOrExit(gNetifIndex > 0);

    assert(aContext != nullptr);
    assert(sTunFd >= 0);
    assert(sNetlinkFd >= 0);
    assert(sIpFd >= 0);

    FD_SET(sTunFd, &aContext->mReadFdSet);
    FD_SET(sTunFd, &aContext->mErrorFdSet);
    FD_SET(sNetlinkFd, &aContext->mReadFdSet);
    FD_SET(sNetlinkFd, &aContext->mErrorFdSet);
#if OPENTHREAD_POSIX_USE_MLD_MONITOR
    FD_SET(sMLDMonitorFd, &aContext->mReadFdSet);
    FD_SET(sMLDMonitorFd, &aContext->mErrorFdSet);
#endif

#if OPENTHREAD_CONFIG_DNS_UPSTREAM_QUERY_ENABLE
    gResolver.UpdateFdSet(*aContext);
#endif

    if (sTunFd > aContext->mMaxFd)
    {
        aContext->mMaxFd = sTunFd;
    }

    if (sNetlinkFd > aContext->mMaxFd)
    {
        aContext->mMaxFd = sNetlinkFd;
    }

#if OPENTHREAD_POSIX_USE_MLD_MONITOR
    if (sMLDMonitorFd > aContext->mMaxFd)
    {
        aContext->mMaxFd = sMLDMonitorFd;
    }
#endif
exit:
    return;
}

void platformNetifOffloadUpdateIp6Addresses(otNetifAddress *aAddressArray, uint8_t aNumAddr)
{
    for (uint8_t i = 0; i < aNumAddr; i++)
    {
        char buffer[64] = {0};
        otIp6AddressToString(&aAddressArray[i].mAddress, buffer, sizeof(buffer));
        otLogInfoPlat("Addr: %s", buffer);
    }

    processAddressChange(aAddressArray, aNumAddr, nullptr);

    SetLinkState(nullptr, true);
}

void platformNetifOffloadReceiveIp6(uint8_t *aBuf, uint16_t aLen)
{
    otError  error     = OT_ERROR_NONE;

    VerifyOrExit(aLen <= kMaxIp6Size);
    VerifyOrExit(sTunFd > 0);

#if OPENTHREAD_POSIX_LOG_TUN_PACKETS
    otLogInfoPlat("[netif] Packet from NCP (%u bytes)", static_cast<uint16_t>(aLen));
    otDumpInfoPlat("", aBuf, aLen);
#endif

    VerifyOrExit(write(sTunFd, aBuf, aLen) == aLen, perror("write"); error = OT_ERROR_FAILED);

exit:

    if (error != OT_ERROR_NONE)
    {
        otLogWarnPlat("[netif] Failed to receive, error:%s", otThreadErrorToString(error));
    }
}

#endif // OPENTHREAD_CONFIG_PLATFORM_NETIF_ENABLE
