
#include "infra_if.hpp"
#include "common/num_utils.hpp"

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#include "border_router/routing_manager_offload.hpp"
#include "common/as_core_type.hpp"
#include "common/locator_getters.hpp"
#include "common/logging.hpp"
#include "instance/instance.hpp"
#include "net/icmp6.hpp"

namespace ot {
namespace BorderRouter {

RegisterLogModule("InfraIf");

InfraIf::InfraIf(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mInitialized(false)
    , mIsRunning(false)
    , mIfIndex(0)
{
}

Error InfraIf::Init(uint32_t aIfIndex)
{
    Error error = kErrorNone;

    VerifyOrExit(!mInitialized, error = kErrorInvalidState);

    mIfIndex     = aIfIndex;
    mInitialized = true;

    LogInfo("Init %s", ToString().AsCString());

exit:
    return error;
}

void InfraIf::Deinit(void)
{
    mInitialized = false;
    mIsRunning   = false;
    mIfIndex     = 0;

    LogInfo("Deinit");
}

bool InfraIf::HasAddress(const Ip6::Address &aAddress) const
{
    OT_ASSERT(mInitialized);

    return otPlatInfraIfHasAddress(mIfIndex, &aAddress);
}

Error InfraIf::Send(const Icmp6Packet &aPacket, const Ip6::Address &aDestination) const
{
    OT_ASSERT(mInitialized);

    return otPlatInfraIfSendIcmp6Nd(mIfIndex, &aDestination, aPacket.GetBytes(), aPacket.GetLength());
}

Error InfraIf::HandleStateChanged(uint32_t aIfIndex, bool aIsRunning)
{
    Error error = kErrorNone;

    VerifyOrExit(mInitialized, error = kErrorInvalidState);
    VerifyOrExit(aIfIndex == mIfIndex, error = kErrorInvalidArgs);

    VerifyOrExit(aIsRunning != mIsRunning);
    LogInfo("State changed: %sRUNNING -> %sRUNNING", mIsRunning ? "" : "NOT ", aIsRunning ? "" : "NOT ");

    mIsRunning = aIsRunning;

    Get<RoutingManagerOffload>().HandleInfraIfStateChanged();

exit:
    return error;
}

void InfraIf::HandledReceived(uint32_t aIfIndex, const Ip6::Address &aSource, const Icmp6Packet &aPacket)
{
    Error error = kErrorNone;

    VerifyOrExit(mInitialized && mIsRunning, error = kErrorInvalidState);
    VerifyOrExit(aIfIndex == mIfIndex, error = kErrorDrop);
    VerifyOrExit(aPacket.GetBytes() != nullptr, error = kErrorInvalidArgs);
    VerifyOrExit(aPacket.GetLength() >= sizeof(Ip6::Icmp::Header), error = kErrorParse);

    Get<RoutingManagerOffload>().HandleReceived(aPacket, aSource);

exit:
    if (error != kErrorNone)
    {
        LogDebg("Dropped ICMPv6 message: %s", ErrorToString(error));
    }
}

InfraIf::InfoString InfraIf::ToString(void) const
{
    InfoString string;

    string.Append("infra netif %lu", ToUlong(mIfIndex));
    return string;
}

//---------------------------------------------------------------------------------------------------------------------

extern "C" void otPlatInfraIfRecvIcmp6Nd(otInstance         *aInstance,
                                         uint32_t            aInfraIfIndex,
                                         const otIp6Address *aSrcAddress,
                                         const uint8_t      *aBuffer,
                                         uint16_t            aBufferLength)
{
    InfraIf::Icmp6Packet packet;

    packet.Init(aBuffer, aBufferLength);
    AsCoreType(aInstance).Get<InfraIf>().HandledReceived(aInfraIfIndex, AsCoreType(aSrcAddress), packet);
}

extern "C" otError otPlatInfraIfStateChanged(otInstance *aInstance, uint32_t aInfraIfIndex, bool aIsRunning)
{
    return AsCoreType(aInstance).Get<InfraIf>().HandleStateChanged(aInfraIfIndex, aIsRunning);
}

} // namespace BorderRouter
} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
