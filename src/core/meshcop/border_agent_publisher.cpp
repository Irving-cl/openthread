/*
 *  Copyright (c) 2024, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the Border Agent Service Publisher.
 */

#include "border_agent_publisher.hpp"

#if OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE && OPENTHREAD_CONFIG_PLATFORM_DNSSD_ENABLE

#include <openthread/mdns.h>

#include "backbone_router/bbr_local.hpp"
#include "border_router/routing_manager.hpp"
#include "common/as_core_type.hpp"
#include "common/encoding.hpp"
#include "common/error.hpp"
#include "common/log.hpp"
#include "common/string.hpp"
#include "instance/instance.hpp"
#include "mac/mac.hpp"
#include "mac/mac_types.hpp"
#include "meshcop/border_agent.hpp"
#include "meshcop/network_name.hpp"
#include "net/dns_types.hpp"
#include "net/dnssd.hpp"
#include "net/ip6_address.hpp"
#include "thread/mle.hpp"
#include "thread/version.hpp"

namespace ot {
namespace MeshCoP {

static const char kBorderAgentServiceType[]      = "_meshcop._udp";   ///< Border agent service type of mDNS
static const char kBorderAgentEpskcServiceType[] = "_meshcop-e._udp"; ///< Border agent ePSKc service

#if OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1
static const char kThreadVersionString[] = "1.1.1";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_2
static const char kThreadVersionString[] = "1.2.0";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_3
static const char kThreadVersionString[] = "1.3.0";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_3_1
static const char kThreadVersionString[] = "1.3.1";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_4
static const char kThreadVersionString[] = "1.4.0";
#endif

RegisterLogModule("BA_Publisher");

BorderAgentPublisher::BorderAgentPublisher(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mEnabled(false)
    , mRequestId(0)
{
    Clear();
}

void BorderAgentPublisher::SetEnabled(bool aEnabled)
{
    VerifyOrExit(mEnabled != aEnabled);
    mEnabled = aEnabled;

    if (mEnabled)
    {
        Start();
    }
    else
    {
        Stop();
    }

exit:
    return;
}

void BorderAgentPublisher::Start(void)
{
    StringWriter stringWriter(mServiceInstanceName, sizeof(mServiceInstanceName));

    WriteServiceInstanceNameWithExtAddr(stringWriter);
    UpdateMeshCopService();
}

void BorderAgentPublisher::Stop(void) { UnpublishMeshCopService(); }

void BorderAgentPublisher::WriteServiceInstanceNameWithExtAddr(StringWriter &aStringWriter)
{
    const Mac::ExtAddress &extAddress = Get<Mac::Mac>().GetExtAddress();

    memset(mServiceInstanceName, 0, sizeof(mServiceInstanceName));

    aStringWriter.Append("%s", mBaseServiceInstanceName)
        .Append(" #")
        .Append("%02x", extAddress.m8[6])
        .Append("%02x", extAddress.m8[7]);
}

void BorderAgentPublisher::GenerateAlternativeServiceInstanceName(void)
{
    StringWriter stringWriter(mServiceInstanceName, sizeof(mServiceInstanceName));
    uint16_t     random = Random::NonCrypto::GetUint16();

    WriteServiceInstanceNameWithExtAddr(stringWriter);
    stringWriter.Append("(").Append("%d", random).Append(")");
}

Error BorderAgentPublisher::SetMeshCopServiceValues(const char    *aBaseServiceInstanceName,
                                                    const char    *aProductName,
                                                    const char    *aVendorName,
                                                    const uint8_t *aVendorOui)
{
    Error   error = kErrorNone;
    uint8_t serviceInstanceNameLen;
    uint8_t productNameLen;
    uint8_t vendorNameLen;

    serviceInstanceNameLen = StringLength(aBaseServiceInstanceName, kMaxBaseServiceInstanceNameLength + 1);
    VerifyOrExit(serviceInstanceNameLen <= kMaxBaseServiceInstanceNameLength, error = kErrorInvalidArgs);
    VerifyOrExit(IsValidUtf8String(aBaseServiceInstanceName), error = kErrorInvalidArgs);

    productNameLen = StringLength(aProductName, kMaxProductNameLength + 1);
    VerifyOrExit(productNameLen <= kMaxProductNameLength, error = kErrorInvalidArgs);
    VerifyOrExit(IsValidUtf8String(aProductName), error = kErrorInvalidArgs);

    vendorNameLen = StringLength(aVendorName, kMaxVendorNameLength + 1);
    VerifyOrExit(vendorNameLen <= kMaxVendorNameLength, error = kErrorInvalidArgs);
    VerifyOrExit(IsValidUtf8String(aVendorName), error = kErrorInvalidArgs);

    memcpy(mBaseServiceInstanceName, aBaseServiceInstanceName, serviceInstanceNameLen);
    memcpy(mProductName, aProductName, productNameLen);
    memcpy(mVendorName, aVendorName, vendorNameLen);
    memcpy(mVendorOui, aVendorOui, kVendorOuiLength);

    Start();

exit:
    return error;
}

Error BorderAgentPublisher::SetMeshCopSerivceVendorTxtData(const uint8_t *aVendorTxtData, uint16_t aLen)
{
    Error error = kErrorNone;

    VerifyOrExit(aLen <= kVendorTxtData, error = kErrorInvalidArgs);

    SuccessOrExit(error = mVendorTxtData.SetFrom(aVendorTxtData, aLen));
    UpdateMeshCopService();

exit:
    return error;
}

bool BorderAgentPublisher::IsThreadStarted(void)
{
    return Get<Mle::Mle>().IsChild() || Get<Mle::Mle>().IsRouter() || Get<Mle::Mle>().IsLeader();
}

void BorderAgentPublisher::HandleNotifierEvents(Events aEvents)
{
    VerifyOrExit(mEnabled);

    if (aEvents.Contains(kEventThreadRoleChanged))
    {
        LogInfo("Thread is %s", IsThreadStarted() ? "up" : "down");
    }

    if (aEvents.ContainsAny(kEventThreadRoleChanged | kEventThreadExtPanIdChanged | kEventThreadNetworkNameChanged |
                            kEventThreadBackboneRouterStateChanged | kEventThreadNetdataChanged))
    {
        UpdateMeshCopService();
    }

exit:
    return;
}

void BorderAgentPublisher::UpdateMeshCopService(void)
{
    LogInfo("!!! UpdateMeshCopService, Enabled:%d, IsMeshCopSet:%d, IsDnssdReady:%d", mEnabled, IsMeshCopValuesSet(),
            Get<Dnssd>().IsReady());

    if (mEnabled && IsMeshCopValuesSet() && Get<Dnssd>().IsReady())
    {
        LogInfo("Start Thread Border Agent (Meshcop service publisher)");
        PublishMeshCopService();
    }
}

void BorderAgentPublisher::PublishMeshCopService(void)
{
    static constexpr int kBorderAgentServiceDummyPort = 49152;

    Error                  error = OT_ERROR_NONE;
    Dnssd::Service         serviceInfo;
    Message               *message     = Get<MessagePool>().Allocate(Message::kTypeOther);
    const NetworkName     &networkName = Get<NetworkNameManager>().GetNetworkName();
    const ExtendedPanId   &extPanId    = Get<ExtendedPanIdManager>().GetExtPanId();
    const Mac::ExtAddress &extAddr     = Get<Mac::Mac>().GetExtAddress();
    uint32_t               partitionId = Get<Mle::MleRouter>().GetLeaderData().GetPartitionId();
#if OPENTHREAD_CONFIG_BORDER_AGENT_ID_ENABLE
    BorderAgent::Id id;
#endif
    StateBitmap state;
    uint32_t    stateUint32;

    Dns::TxtEntry txtEntry;

    VerifyOrExit(message != nullptr, error = kErrorNoBufs);

#if OPENTHREAD_CONFIG_BORDER_AGENT_ID_ENABLE
    if (Get<BorderAgent>().GetId(id) == kErrorNone)
    {
        txtEntry.Init("id", id.mId, sizeof(id));
        txtEntry.AppendTo(*message);
    }
#endif
    txtEntry.Init("vo", mVendorOui, sizeof(mVendorOui));
    txtEntry.AppendTo(*message);
    txtEntry.Init("vn", reinterpret_cast<const uint8_t *>(mVendorName),
                  StringLength(mVendorName, kMaxVendorNameLength));
    txtEntry.AppendTo(*message);
    txtEntry.Init("mn", reinterpret_cast<const uint8_t *>(mProductName),
                  StringLength(mProductName, kMaxProductNameLength));
    txtEntry.AppendTo(*message);
    txtEntry.Init("nn", reinterpret_cast<const uint8_t *>(networkName.GetAsCString()),
                  StringLength(networkName.GetAsCString(), sizeof(networkName)));
    txtEntry.AppendTo(*message);
    txtEntry.Init("xp", extPanId.m8, sizeof(extPanId));
    txtEntry.AppendTo(*message);
    txtEntry.Init("tv", reinterpret_cast<const uint8_t *>(kThreadVersionString), sizeof(kThreadVersionString));
    txtEntry.AppendTo(*message);
    txtEntry.Init("xa", extAddr.m8, sizeof(extAddr));
    txtEntry.AppendTo(*message);
    state = GetStateBitmap();
#if OPENTHREAD_CONFIG_BORDER_AGENT_EPHEMERAL_KEY_ENABLE
    state.mEpskcSupported = Get<BorderAgent>().IsEphemeralKeyActive();
#endif
    stateUint32 = BigEndian::HostSwap32(state.ToUint32());
    txtEntry.Init("sb", reinterpret_cast<uint8_t *>(&stateUint32), sizeof(stateUint32));
    txtEntry.AppendTo(*message);

    if (state.mThreadIfStatus == kThreadIfStatusActive)
    {
        txtEntry.Init("pt", reinterpret_cast<uint8_t *>(&partitionId), sizeof(partitionId));
        txtEntry.AppendTo(*message);

        AppendActiveTimestampTxtEntry(txtEntry, *message);
    }

#if OPENTHREAD_CONFIG_BACKBONE_ROUTER_ENABLE
    AppendBbrTxtEntry(state, txtEntry, *message);
#endif

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
    AppendOmrTxtEntry(txtEntry, *message);
#endif

    message->AppendBytes(mVendorTxtData.GetBytes(), mVendorTxtData.GetLength());

    SuccessOrExit(error = mTxtData.SetFrom(*message));

    serviceInfo.Clear();
    serviceInfo.mHostName        = "";
    serviceInfo.mServiceInstance = mServiceInstanceName;
    serviceInfo.mServiceType     = kBorderAgentServiceType;
    serviceInfo.mPort    = Get<BorderAgent>().GetState() != BorderAgent::kStateStopped ? Get<BorderAgent>().GetUdpPort()
                                                                                       : kBorderAgentServiceDummyPort;
    serviceInfo.mTxtData = mTxtData.GetBytes();
    serviceInfo.mTxtDataLength = mTxtData.GetLength();

    LogInfo("Publish service %s.%s.local (requestId:%u)", mServiceInstanceName, kBorderAgentServiceType, mRequestId);
    Get<Dnssd>().RegisterService(serviceInfo, mRequestId++, PublishMeshCopServiceCallback);

exit:
    if (error != kErrorNone)
    {
        LogWarn("Failed to publish MeshCop service, %s", ErrorToString(error));
    }
    FreeMessage(message);
}

void BorderAgentPublisher::UnpublishMeshCopService(void)
{
    Dnssd::Service serviceInfo;

    serviceInfo.Clear();
    serviceInfo.mHostName        = "";
    serviceInfo.mServiceInstance = mServiceInstanceName;
    serviceInfo.mServiceType     = kBorderAgentServiceType;

    LogInfo("Unpublish meshcop service %s.%s.local (requestId:%u)", mServiceInstanceName, kBorderAgentServiceType,
            mRequestId);
    Get<Dnssd>().UnregisterService(serviceInfo, mRequestId++, UnpublishMeshCopServiceCallback);
}

void BorderAgentPublisher::HandleEpskcStateChanged(void)
{
    if (Get<BorderAgent>().IsEphemeralKeyActive())
    {
        PublishEpskcService();
    }
    else
    {
        UnpublishEpskcService();
    }
}

void BorderAgentPublisher::PublishEpskcService(void)
{
    Dnssd::Service serviceInfo;

    serviceInfo.Clear();
    serviceInfo.mHostName        = "";
    serviceInfo.mServiceInstance = mServiceInstanceName;
    serviceInfo.mServiceType     = kBorderAgentEpskcServiceType;
    serviceInfo.mPort            = Get<BorderAgent>().GetUdpPort();

    LogInfo("Publish service %s.%s.local, port:%u (requestId:%u)", mServiceInstanceName, kBorderAgentServiceType,
            serviceInfo.mPort, mRequestId);
    Get<Dnssd>().RegisterService(serviceInfo, mRequestId++, PublishEpskcServiceCallback);
}

void BorderAgentPublisher::UnpublishEpskcService(void)
{
    Dnssd::Service serviceInfo;

    serviceInfo.Clear();
    serviceInfo.mHostName        = "";
    serviceInfo.mServiceInstance = mServiceInstanceName;
    serviceInfo.mServiceType     = kBorderAgentEpskcServiceType;

    LogInfo("Unpublish service %s.%s.local (requestId:%u)", mServiceInstanceName, kBorderAgentEpskcServiceType,
            mRequestId);
    Get<Dnssd>().UnregisterService(serviceInfo, mRequestId++, UnpublishEpskcServiceCallback);
}

void BorderAgentPublisher::HandleDnssdPlatformStateChange(void)
{
    VerifyOrExit(mEnabled);

    switch (Get<Dnssd>().GetState())
    {
    case Dnssd::State::kReady:
        UpdateMeshCopService();
        break;
    default:
        LogWarn("Platform DNSSD not available!");
        break;
    }
exit:
    return;
}

void BorderAgentPublisher::Clear(void)
{
    memset(mBaseServiceInstanceName, 0, sizeof(mBaseServiceInstanceName));
    memset(mProductName, 0, sizeof(mProductName));
    memset(mVendorName, 0, sizeof(mVendorName));
    memset(mVendorOui, 0, sizeof(mVendorOui));
}

bool BorderAgentPublisher::IsMeshCopValuesSet(void) { return mBaseServiceInstanceName[0] != '\0'; }

BorderAgentPublisher::StateBitmap BorderAgentPublisher::GetStateBitmap(void)
{
    StateBitmap state;

    state.mConnectionMode = kConnectionModePskc;
    state.mAvailability   = kAvailabilityHigh;

    switch (Get<Mle::MleRouter>().GetRole())
    {
    case Mle::DeviceRole::kRoleDisabled:
        state.mThreadIfStatus = kThreadIfStatusNotInitialized;
        state.mThreadRole     = kThreadRoleDisabledOrDetached;
        break;
    case Mle::DeviceRole::kRoleDetached:
        state.mThreadIfStatus = kThreadIfStatusInitialized;
        state.mThreadRole     = kThreadRoleDisabledOrDetached;
        break;
    case Mle::DeviceRole::kRoleChild:
        state.mThreadIfStatus = kThreadIfStatusActive;
        state.mThreadRole     = kThreadRoleChild;
        break;
    case Mle::DeviceRole::kRoleRouter:
        state.mThreadIfStatus = kThreadIfStatusActive;
        state.mThreadRole     = kThreadRoleRouter;
        break;
    case Mle::DeviceRole::kRoleLeader:
        state.mThreadIfStatus = kThreadIfStatusActive;
        state.mThreadRole     = kThreadRoleLeader;
        break;
    }

#if OPENTHREAD_CONFIG_BACKBONE_ROUTER_ENABLE
    state.mBbrIsActive = state.mThreadIfStatus == kThreadIfStatusActive &&
                         Get<BackboneRouter::Local>().GetState() != BackboneRouter::Local::State::kStateDisabled;
    state.mBbrIsPrimary = state.mThreadIfStatus == kThreadIfStatusActive &&
                          Get<BackboneRouter::Local>().GetState() == BackboneRouter::Local::State::kStatePrimary;
#endif

    return state;
}

static uint64_t ConvertTimestampToUint64(const otTimestamp &aTimestamp)
{
    // 64 bits Timestamp fields layout
    //-----48 bits------//-----15 bits-----//-------1 bit-------//
    //     Seconds      //      Ticks      //  Authoritative    //
    return (aTimestamp.mSeconds << 16) | static_cast<uint64_t>(aTimestamp.mTicks << 1) |
           static_cast<uint64_t>(aTimestamp.mAuthoritative);
}

void BorderAgentPublisher::AppendActiveTimestampTxtEntry(Dns::TxtEntry &txtEntry, Message &aMessage)
{
    Error                  error;
    MeshCoP::Dataset::Info dataset;

    if ((error = Get<MeshCoP::ActiveDatasetManager>().Read(dataset)) != kErrorNone)
    {
        LogWarn("Failed to get active dataset: %s", ErrorToString(error));
    }
    else
    {
        uint64_t activeTimestampValue = ConvertTimestampToUint64(dataset.mActiveTimestamp);

        activeTimestampValue = BigEndian::HostSwap64(activeTimestampValue);
        txtEntry.Init("at", reinterpret_cast<uint8_t *>(&activeTimestampValue), sizeof(activeTimestampValue));
        txtEntry.AppendTo(aMessage);
    }
}

#if OPENTHREAD_CONFIG_BACKBONE_ROUTER_ENABLE
void BorderAgentPublisher::AppendBbrTxtEntry(StateBitmap aState, Dns::TxtEntry &txtEntry, Message &aMessage)
{
    static constexpr uint16_t kBackboneUdpPort = 61631; ///< The BBR port.

    const DomainName &domainName = Get<MeshCoP::NetworkNameManager>().GetDomainName();

    if (aState.mBbrIsActive)
    {
        BackboneRouter::Config bbrConfig;
        uint16_t               bbrPort = BigEndian::HostSwap16(kBackboneUdpPort);

        Get<BackboneRouter::Local>().GetConfig(bbrConfig);
        txtEntry.Init("sq", &bbrConfig.mSequenceNumber, sizeof(bbrConfig.mSequenceNumber));
        txtEntry.AppendTo(aMessage);

        txtEntry.Init("bb", reinterpret_cast<const uint8_t *>(&bbrPort), sizeof(bbrPort));
        txtEntry.AppendTo(aMessage);
    }

    txtEntry.Init("dn", reinterpret_cast<const uint8_t *>(domainName.GetAsCString()),
                  StringLength(domainName.GetAsCString(), sizeof(domainName)));
}
#endif // OPENTHREAD_CONFIG_BACKBONE_ROUTER_ENABLE

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
void BorderAgentPublisher::AppendOmrTxtEntry(Dns::TxtEntry &txtEntry, Message &aMessage)
{
    Ip6::Prefix                                   prefix;
    BorderRouter::RoutingManager::RoutePreference preference;

    if (Get<BorderRouter::RoutingManager>().GetFavoredOmrPrefix(prefix, preference) == kErrorNone)
    {
        uint8_t omrData[Ip6::NetworkPrefix::kSize + 1];
        omrData[0] = prefix.mLength;
        memcpy(omrData + 1, prefix.mPrefix.mFields.m8, (prefix.mLength + 7) / 8);

        txtEntry.Init("omr", omrData, 1 + (prefix.mLength + 7) / 8);
        txtEntry.AppendTo(aMessage);
    }
}
#endif

void BorderAgentPublisher::PublishMeshCopServiceCallback(otInstance          *aInstance,
                                                         otPlatDnssdRequestId aRequestId,
                                                         otError              aError)
{
    AsCoreType(aInstance).Get<BorderAgentPublisher>().PublishMeshCopServiceCallback(aRequestId, aError);
}

void BorderAgentPublisher::PublishMeshCopServiceCallback(otPlatDnssdRequestId aRequestId, otError aError)
{
    OT_UNUSED_VARIABLE(aRequestId); // For unused variable warning when Log is not enabled.

    if (aError == kErrorAbort)
    {
        // OTBR_ERROR_ABORTED is thrown when an ongoing service registration is
        // cancelled. This can happen when the meshcop service is being updated
        // frequently. To avoid false alarms, it should not be logged like a real error.
        LogInfo("Cancelled previous publishing meshcop service (requestId:%u)", aRequestId);
    }
    else if (aError == kErrorDuplicated)
    {
        // Try to unpublish current service in case we are trying to register
        // multiple new services simultaneously when the original service name
        // is conflicted.
        UnpublishMeshCopService();
        GenerateAlternativeServiceInstanceName();
        PublishMeshCopService();
    }
    else
    {
        LogInfo("Result of publish meshcop service (requestId:%u): %s", aRequestId, ErrorToString(aError));
    }
}

void BorderAgentPublisher::UnpublishMeshCopServiceCallback(otInstance          *aInstance,
                                                           otPlatDnssdRequestId aRequestId,
                                                           otError              aError)
{
    AsCoreType(aInstance).Get<BorderAgentPublisher>().UnpublishMeshCopServiceCallback(aRequestId, aError);
}

void BorderAgentPublisher::UnpublishMeshCopServiceCallback(otPlatDnssdRequestId aRequestId, otError aError)
{
    OT_UNUSED_VARIABLE(aRequestId); // For unused variable warning when Log is not enabled.
    OT_UNUSED_VARIABLE(aError);

    LogInfo("Result of unpublish meshcop service (requestId:%u): %s", aRequestId, ErrorToString(aError));
}

void BorderAgentPublisher::PublishEpskcServiceCallback(otInstance          *aInstance,
                                                       otPlatDnssdRequestId aRequestId,
                                                       otError              aError)
{
    AsCoreType(aInstance).Get<BorderAgentPublisher>().PublishEpskcServiceCallback(aRequestId, aError);
}

void BorderAgentPublisher::PublishEpskcServiceCallback(otPlatDnssdRequestId aRequestId, otError aError)
{
    OT_UNUSED_VARIABLE(aRequestId); // For unused variable warning when Log is not enabled.

    if (aError == kErrorAbort)
    {
        // OTBR_ERROR_ABORTED is thrown when an ongoing service registration is
        // cancelled. This can happen when the meshcop service is being updated
        // frequently. To avoid false alarms, it should not be logged like a real error.
        LogInfo("Cancelled previous publishing meshcop-e service (requestId:%u)", aRequestId);
    }
    else if (aError == kErrorDuplicated)
    {
        // Try to unpublish current service in case we are trying to register
        // multiple new services simultaneously when the original service name
        // is conflicted.
        UnpublishEpskcService();
        GenerateAlternativeServiceInstanceName();
        PublishEpskcService();
    }
    else
    {
        LogInfo("Result of publish meshcop-e service (requestId:%u): %s", aRequestId, ErrorToString(aError));
    }
}

void BorderAgentPublisher::UnpublishEpskcServiceCallback(otInstance          *aInstance,
                                                         otPlatDnssdRequestId aRequestId,
                                                         otError              aError)
{
    AsCoreType(aInstance).Get<BorderAgentPublisher>().UnpublishEpskcServiceCallback(aRequestId, aError);
}

void BorderAgentPublisher::UnpublishEpskcServiceCallback(otPlatDnssdRequestId aRequestId, otError aError)
{
    OT_UNUSED_VARIABLE(aRequestId); // For unused variable warning when Log is not enabled.
    OT_UNUSED_VARIABLE(aError);

    LogInfo("Result of unpublish meshcop-e service (requestId:%u): %s", aRequestId, ErrorToString(aError));
}

} // namespace MeshCoP
} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE && OPENTHREAD_CONFIG_PLATFORM_DNSSD_ENABLE
