
#include "border_router/routing_manager_offload.hpp"

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/locator_getters.hpp"
#include "common/log.hpp"
#include "routing_manager_offload.hpp"

#include <openthread/platform/offload.h>

namespace ot {

namespace BorderRouter {

RegisterLogModule("RtMgrOffload");

RoutingManagerOffload::RoutingManagerOffload(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mIsRunning(false)
    , mIsEnabled(false)
    , mInfraIf(aInstance)
    , mOmrPrefixManager(aInstance)
    , mRioPreference(NetworkData::kRoutePreferenceLow)
    , mUserSetRioPreference(false)
    , mOnLinkPrefixManager(aInstance)
    , mDiscoveredPrefixTable(aInstance)
    , mRoutePublisher(aInstance)
    , mRsSender(aInstance)
    , mDiscoveredPrefixStaleTimer(aInstance)
    , mRoutingPolicyTimer(aInstance)
{
    mBrUlaPrefix.Clear();
}

Error RoutingManagerOffload::Init(uint32_t aInfraIfIndex, bool aInfraIfIsRunning)
{
    Error error = kErrorNone;

    VerifyOrExit(GetState() == kStateUninitialized || GetState() == kStateDisabled, error = kErrorInvalidState);

    if (!mInfraIf.IsInitialized())
    {
        LogInfo("Initializing - InfraIfIndex:%lu", ToUlong(aInfraIfIndex));
        SuccessOrExit(error = mInfraIf.Init(aInfraIfIndex));
        SuccessOrExit(error = LoadOrGenerateRandomBrUlaPrefix());
        mOmrPrefixManager.Init(mBrUlaPrefix);
#if OPENTHREAD_CONFIG_NAT64_BORDER_ROUTING_ENABLE
        mNat64PrefixManager.GenerateLocalPrefix(mBrUlaPrefix);
#endif
        mOnLinkPrefixManager.Init();
    }
    else if (aInfraIfIndex != mInfraIf.GetIfIndex())
    {
        LogInfo("Reinitializing - InfraIfIndex:%lu -> %lu", ToUlong(mInfraIf.GetIfIndex()), ToUlong(aInfraIfIndex));
        mInfraIf.SetIfIndex(aInfraIfIndex);
    }

    error = mInfraIf.HandleStateChanged(mInfraIf.GetIfIndex(), aInfraIfIsRunning);

exit:
    if (error != kErrorNone)
    {
        mInfraIf.Deinit();
    }

    return error;
}

Error RoutingManagerOffload::SetEnabled(bool aEnabled)
{
    Error error = kErrorNone;

    VerifyOrExit(IsInitialized(), error = kErrorInvalidState);

    VerifyOrExit(aEnabled != mIsEnabled);

    mIsEnabled = aEnabled;
    LogInfo("%s", mIsEnabled ? "Enabling" : "Disabling");
    EvaluateState();

exit:
    return error;
}

RoutingManagerOffload::State RoutingManagerOffload::GetState(void) const
{
    State state = kStateUninitialized;

    VerifyOrExit(IsInitialized());
    VerifyOrExit(IsEnabled(), state = kStateDisabled);

    state = IsRunning() ? kStateRunning : kStateStopped;

exit:
    return state;
}

void RoutingManagerOffload::SetRouteInfoOptionPreference(RoutePreference aPreference)
{
    LogInfo("User explicitly set RIO Preference to %s", RoutePreferenceToString(aPreference));
    mUserSetRioPreference = true;
    UpdateRioPreference(aPreference);
}

void RoutingManagerOffload::ClearRouteInfoOptionPreference(void)
{
    VerifyOrExit(mUserSetRioPreference);

    LogInfo("User cleared explicitly set RIO Preference");
    mUserSetRioPreference = false;
    SetRioPreferenceBasedOnRole();

exit:
    return;
}

void RoutingManagerOffload::SetRioPreferenceBasedOnRole(void)
{
    otDeviceRole role = otPlatCpGetDeviceRoleCached(nullptr);
    bool isRouterOrLeader = (role == OT_DEVICE_ROLE_ROUTER) || (role == OT_DEVICE_ROLE_LEADER);
    UpdateRioPreference(isRouterOrLeader ? NetworkData::kRoutePreferenceMedium
                                                           : NetworkData::kRoutePreferenceLow);
}

void RoutingManagerOffload::UpdateRioPreference(RoutePreference aPreference)
{
    VerifyOrExit(mRioPreference != aPreference);

    LogInfo("RIO Preference changed: %s -> %s", RoutePreferenceToString(mRioPreference),
            RoutePreferenceToString(aPreference));
    mRioPreference = aPreference;

    VerifyOrExit(mIsRunning);
    ScheduleRoutingPolicyEvaluation(kAfterRandomDelay);

exit:
    return;
}

Error RoutingManagerOffload::GetOmrPrefix(Ip6::Prefix &aPrefix) const
{
    Error error = kErrorNone;

    VerifyOrExit(IsInitialized(), error = kErrorInvalidState);
    aPrefix = mOmrPrefixManager.GetGeneratedPrefix();

exit:
    return error;
}

Error RoutingManagerOffload::GetFavoredOmrPrefix(Ip6::Prefix &aPrefix, RoutePreference &aPreference) const
{
    Error error = kErrorNone;

    VerifyOrExit(IsRunning(), error = kErrorInvalidState);
    aPrefix     = mOmrPrefixManager.GetFavoredPrefix().GetPrefix();
    aPreference = mOmrPrefixManager.GetFavoredPrefix().GetPreference();

exit:
    return error;
}

Error RoutingManagerOffload::GetOnLinkPrefix(Ip6::Prefix &aPrefix) const
{
    Error error = kErrorNone;

    VerifyOrExit(IsInitialized(), error = kErrorInvalidState);
    aPrefix = mOnLinkPrefixManager.GetLocalPrefix();

exit:
    return error;
}

Error RoutingManagerOffload::GetFavoredOnLinkPrefix(Ip6::Prefix &aPrefix) const
{
    Error error = kErrorNone;

    VerifyOrExit(IsInitialized(), error = kErrorInvalidState);
    aPrefix = mOnLinkPrefixManager.GetFavoredDiscoveredPrefix();

    if (aPrefix.GetLength() == 0)
    {
        aPrefix = mOnLinkPrefixManager.GetLocalPrefix();
    }

exit:
    return error;
}

Error RoutingManagerOffload::LoadOrGenerateRandomBrUlaPrefix(void)
{
    // Just generate now
    Error error     = kErrorNone;
    bool  generated = false;

    if (Get<Settings>().Read<Settings::BrUlaPrefix>(mBrUlaPrefix) != kErrorNone || !IsValidBrUlaPrefix(mBrUlaPrefix))
    {
        Ip6::NetworkPrefix randomUlaPrefix;

        LogNote("No valid /48 BR ULA prefix found in settings, generating new one");

        SuccessOrExit(error = randomUlaPrefix.GenerateRandomUla());

        mBrUlaPrefix.Set(randomUlaPrefix);
        mBrUlaPrefix.SetSubnetId(0);
        mBrUlaPrefix.SetLength(kBrUlaPrefixLength);

        IgnoreError(Get<Settings>().Save<Settings::BrUlaPrefix>(mBrUlaPrefix));
        generated = true;
    }

    LogNote("BR ULA prefix: %s (%s)", mBrUlaPrefix.ToString().AsCString(), generated ? "generated" : "loaded");

exit:
    if (error != kErrorNone)
    {
        LogCrit("Failed to generate random /48 BR ULA prefix");
    }
    return error;
}

void RoutingManagerOffload::EvaluateState(void)
{
    otDeviceRole role = otPlatCpGetDeviceRoleCached(nullptr);
    bool isAttached = (role == OT_DEVICE_ROLE_CHILD) ||
                      (role == OT_DEVICE_ROLE_ROUTER) ||
                      (role == OT_DEVICE_ROLE_LEADER);
    LogInfo("EvaluateState");

    if (mIsEnabled && mInfraIf.IsRunning() && isAttached)
    {
        LogInfo("Start");
        Start();
    }
    else
    {
        LogInfo("Stop");
        Stop();
    }
}

void RoutingManagerOffload::Start(void)
{
    if (!mIsRunning)
    {
        LogInfo("Starting");

        mIsRunning = true;
        UpdateDiscoveredPrefixTableOnNetDataChange();
        mOnLinkPrefixManager.Start();
        mOmrPrefixManager.Start();
        mRoutePublisher.Start();
        mRsSender.Start();
    }
}

void RoutingManagerOffload::Stop(void)
{
    VerifyOrExit(mIsRunning);

    mOmrPrefixManager.Stop();
    mOnLinkPrefixManager.Stop();

    SendRouterAdvertisement(kInvalidateAllPrevPrefixes);

    mAdvertisedPrefixes.Clear();

    mDiscoveredPrefixTable.RemoveAllEntries();
    mDiscoveredPrefixStaleTimer.Stop();

    mRaInfo.mTxCount = 0;

    mRsSender.Stop();

    mRoutingPolicyTimer.Stop();

    mRoutePublisher.Stop();

    LogInfo("Stopped");

    mIsRunning = false;
exit:
    return;
}

void RoutingManagerOffload::HandleReceived(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress)
{
    const Ip6::Icmp::Header *icmp6Header;

    VerifyOrExit(mIsRunning);

    icmp6Header = reinterpret_cast<const Ip6::Icmp::Header *>(aPacket.GetBytes());

    switch (icmp6Header->GetType())
    {
    case Ip6::Icmp::Header::kTypeRouterAdvert:
         HandleRouterAdvertisement(aPacket, aSrcAddress);
        break;
    case Ip6::Icmp::Header::kTypeRouterSolicit:
         HandleRouterSolicit(aPacket, aSrcAddress);
        break;
    case Ip6::Icmp::Header::kTypeNeighborAdvert:
         HandleNeighborAdvertisement(aPacket);
        break;
    default:
        break;
    }

exit:
    return;
}

void RoutingManagerOffload::HandleNotifierEvents(Events aEvents)
{
    if (aEvents.Contains(kEventThreadRoleChanged) && !mUserSetRioPreference)
    {
        SetRioPreferenceBasedOnRole();
    }

    mRoutePublisher.HandleNotifierEvents(aEvents);

    VerifyOrExit(IsInitialized() && IsEnabled());

    if (aEvents.Contains(kEventThreadRoleChanged))
    {
        EvaluateState();
    }

    if (mIsRunning && aEvents.Contains(kEventThreadNetdataChanged))
    {
        UpdateDiscoveredPrefixTableOnNetDataChange();
        mOnLinkPrefixManager.HandleNetDataChange();
        ScheduleRoutingPolicyEvaluation(kAfterRandomDelay);
    }

    if (aEvents.Contains(kEventThreadExtPanIdChanged))
    {
        mOnLinkPrefixManager.HandleExtPanIdChange();
    }

exit:
    return;
}

void RoutingManagerOffload::UpdateDiscoveredPrefixTableOnNetDataChange(void)
{
    otBorderRouterConfig onMeshPrefixList[10];
    uint8_t count = sizeof(onMeshPrefixList) / sizeof(otBorderRouterConfig);

    SuccessOrExit(otPlatCpNetDataGetOnMeshPrefix(onMeshPrefixList, count));

    for (uint8_t i = 0; i < count; i++)
    {
        NetworkData::OnMeshPrefixConfig prefixConfig = AsCoreType(&onMeshPrefixList[i]);
        if (!IsValidOmrPrefix(prefixConfig))
        {
            continue;
        }

        mDiscoveredPrefixTable.RemoveRoutePrefix(prefixConfig.GetPrefix());
    }

exit:
    return;
}

// This method evaluate the routing policy depends on prefix and route
// information on Thread Network and infra link. As a result, this
// method May send RA messages on infra link and publish/unpublish
// OMR and NAT64 prefix in the Thread network.
void RoutingManagerOffload::EvaluateRoutingPolicy(void)
{
    LogInfo("Evaluating routing policy");

    OT_ASSERT(mIsRunning);

    mOnLinkPrefixManager.Evaluate();
    mOmrPrefixManager.Evaluate();
    mRoutePublisher.Evaluate();

    SendRouterAdvertisement(kAdvPrefixesFromNetData);

    // TODO: Handle Srp Server

    ScheduleRoutingPolicyEvaluation(kForNextRa);
}

bool RoutingManagerOffload::IsInitalPolicyEvaluationDone(void) const
{
    // This method indicates whether or not we are done with the
    // initial policy evaluation and prefix and route setup, i.e.,
    // the OMR and on-link prefixes are determined, advertised in
    // the emitted Router Advert message on infrastructure side
    // and published in the Thread Network Data.

    return mIsRunning && !mOmrPrefixManager.GetFavoredPrefix().IsEmpty() &&
           mOnLinkPrefixManager.IsInitalEvaluationDone();
}

void RoutingManagerOffload::ScheduleRoutingPolicyEvaluation(ScheduleMode aMode)
{
    TimeMilli now   = TimerMilli::GetNow();
    uint32_t  delay = 0;
    TimeMilli evaluateTime;

    switch (aMode)
    {
    case kImmediately:
        break;

    case kForNextRa:
        delay = Random::NonCrypto::GetUint32InRange(Time::SecToMsec(kMinRtrAdvInterval),
                                                    Time::SecToMsec(kMaxRtrAdvInterval));

        if (mRaInfo.mTxCount <= kMaxInitRtrAdvertisements && delay > Time::SecToMsec(kMaxInitRtrAdvInterval))
        {
            delay = Time::SecToMsec(kMaxInitRtrAdvInterval);
        }
        break;

    case kAfterRandomDelay:
        delay = Random::NonCrypto::GetUint32InRange(kPolicyEvaluationMinDelay, kPolicyEvaluationMaxDelay);
        break;

    case kToReplyToRs:
        delay = Random::NonCrypto::GetUint32InRange(0, kRaReplyJitter);
        break;
    }

    // Ensure we wait a min delay after last RA tx
    evaluateTime = Max(now + delay, mRaInfo.mLastTxTime + kMinDelayBetweenRtrAdvs);

    LogInfo("RoutingPolicyTimer FireAtIfEarlier");
    mRoutingPolicyTimer.FireAtIfEarlier(evaluateTime);

#if OT_SHOULD_LOG_AT(OT_LOG_LEVEL_INFO)
    {
        uint32_t duration = evaluateTime - now;

        if (duration == 0)
        {
            LogInfo("Will evaluate routing policy immediately");
        }
        else
        {
            String<Uptime::kStringSize> string;

            Uptime::UptimeToString(duration, string, /* aIncludeMsec */ true);
            LogInfo("Will evaluate routing policy in %s (%lu msec)", string.AsCString() + 3, ToUlong(duration));
        }
    }
#endif
}

void RoutingManagerOffload::SendRouterAdvertisement(RouterAdvTxMode aRaTxMode)
{
    // RA message max length is derived to accommodate:
    //
    // - The RA header.
    // - One RA Flags Extensions Option (with stub router flag).
    // - One PIO for current local on-link prefix.
    // - At most `kMaxOldPrefixes` for old deprecating on-link prefixes.
    // - At most twice `kMaxOnMeshPrefixes` RIO for on-mesh prefixes.
    //   Factor two is used for RIO to account for entries invalidating
    //   previous prefixes while adding new ones.

    static constexpr uint16_t kMaxRaLength =
        sizeof(Ip6::Nd::RouterAdvertMessage::Header) + sizeof(Ip6::Nd::RaFlagsExtOption) +
        sizeof(Ip6::Nd::PrefixInfoOption) + sizeof(Ip6::Nd::PrefixInfoOption) * OnLinkPrefixManager::kMaxOldPrefixes +
        2 * kMaxOnMeshPrefixes * (sizeof(Ip6::Nd::RouteInfoOption) + sizeof(Ip6::Prefix));

    uint8_t                         buffer[kMaxRaLength];
    Ip6::Nd::RouterAdvertMessage    raMsg(mRaInfo.mHeader, buffer);
    otBorderRouterConfig onMeshPrefixList[10];
    uint8_t count = sizeof(onMeshPrefixList) / sizeof(otBorderRouterConfig);

    LogInfo("Preparing RA");

#if OPENTHREAD_CONFIG_BORDER_ROUTING_STUB_ROUTER_FLAG_IN_EMITTED_RA_ENABLE
    SuccessOrAssert(raMsg.AppendFlagsExtensionOption(/* aStubRouterFlag */ true));
    LogInfo("- FlagsExt - StubRouter:1");
#endif

    LogInfo("SendRouterAdv, NetDataGetOnMeshPrefix");
    SuccessOrAssert(otPlatCpNetDataGetOnMeshPrefix(onMeshPrefixList, count));
    LogInfo("SendRouterAdv, NetDataGetOnMeshPrefix Done");

    // Append PIO for local on-link prefix if is either being
    // advertised or deprecated and for old prefix if is being
    // deprecated.

    mOnLinkPrefixManager.AppendAsPiosTo(raMsg);

    // Determine which previously advertised prefixes need to be
    // invalidated. Under `kInvalidateAllPrevPrefixes` mode we need
    // to invalidate all. Under `kAdvPrefixesFromNetData` mode, we
    // check Network Data entries and invalidate any previously
    // advertised prefix that is no longer present in the Network
    // Data. We go through all Network Data prefixes and mark the
    // ones we find in `mAdvertisedPrefixes` as deleted by setting
    // the prefix length to zero). By the end, the remaining entries
    // in the array with a non-zero prefix length are invalidated.

    if (aRaTxMode != kInvalidateAllPrevPrefixes)
    {
        for (uint8_t i = 0; i < count; i++)
        {
            NetworkData::OnMeshPrefixConfig &prefixConfig = AsCoreType(&onMeshPrefixList[i]);

            if (!prefixConfig.mOnMesh || prefixConfig.mDp ||
                (prefixConfig.GetPrefix() == mOmrPrefixManager.GetLocalPrefix().GetPrefix()))
            {
                continue;
            }

            mAdvertisedPrefixes.MarkAsDeleted(prefixConfig.GetPrefix());
        }

        if (mOmrPrefixManager.IsLocalAddedInNetData())
        {
            mAdvertisedPrefixes.MarkAsDeleted(mOmrPrefixManager.GetLocalPrefix().GetPrefix());
        }
    }

    for (const OnMeshPrefix &prefix : mAdvertisedPrefixes)
    {
        if (prefix.GetLength() != 0)
        {
            SuccessOrAssert(raMsg.AppendRouteInfoOption(prefix, /* aRouteLifetime */ 0, mRioPreference));
            LogRouteInfoOption(prefix, 0, mRioPreference);
        }
    }

    // Discover and add prefixes from Network Data to advertise as
    // RIO in the Router Advertisement message.

    mAdvertisedPrefixes.Clear();

    if (aRaTxMode == kAdvPrefixesFromNetData)
    {
        // `mAdvertisedPrefixes` array has a limited size. We add more
        // important prefixes first in the array to ensure they are
        // advertised in the RA message. Note that `Add()` method
        // will ensure to add a prefix only once (will check if
        // prefix is already present in the array).

        // (1) Local OMR prefix.

        if (mOmrPrefixManager.IsLocalAddedInNetData())
        {
            mAdvertisedPrefixes.Add(mOmrPrefixManager.GetLocalPrefix().GetPrefix());
        }

        // (2) Favored OMR prefix.

        if (!mOmrPrefixManager.GetFavoredPrefix().IsEmpty() && !mOmrPrefixManager.GetFavoredPrefix().IsDomainPrefix())
        {
            mAdvertisedPrefixes.Add(mOmrPrefixManager.GetFavoredPrefix().GetPrefix());
        }

        // (3) All other OMR prefixes.

        for (uint8_t i = 0; i < count; i++)
        {
            //// Local OMR prefix is added to the array depending on
            //// `mOmrPrefixManager.IsLocalAddedInNetData()` at step (1).
            //// As we iterate through the Network Data prefixes, we skip
            //// over entries matching the local OMR prefix. This
            //// ensures that we stop including it in emitted RA
            //// message as soon as we decide to remove it from Network
            //// Data. Note that upon requesting it to be removed from
            //// Network Data the change needs to be registered with
            //// leader and can take some time to be updated in Network
            //// Data.
            NetworkData::OnMeshPrefixConfig &prefixConfig = AsCoreType(&onMeshPrefixList[i]);

            if (prefixConfig.mDp)
            {
                continue;
            }

            if (IsValidOmrPrefix(prefixConfig) &&
                (prefixConfig.GetPrefix() != mOmrPrefixManager.GetLocalPrefix().GetPrefix()))
            {
                mAdvertisedPrefixes.Add(prefixConfig.GetPrefix());
            }
        }

        // (4) All other on-mesh prefixes (excluding Domain Prefix).
        for (uint8_t i = 0; i < count; i++)
        {
            NetworkData::OnMeshPrefixConfig &prefixConfig = AsCoreType(&onMeshPrefixList[i]);

            if (prefixConfig.mOnMesh && !prefixConfig.mDp && !IsValidOmrPrefix(prefixConfig))
            {
                mAdvertisedPrefixes.Add(prefixConfig.GetPrefix());
            }
        }

        for (const OnMeshPrefix &prefix : mAdvertisedPrefixes)
        {
            SuccessOrAssert(raMsg.AppendRouteInfoOption(prefix, kDefaultOmrPrefixLifetime, mRioPreference));
            LogRouteInfoOption(prefix, kDefaultOmrPrefixLifetime, mRioPreference);
        }
    }

    if (raMsg.ContainsAnyOptions())
    {
        Error        error;
        Ip6::Address destAddress;

        ++mRaInfo.mTxCount;

        destAddress.SetToLinkLocalAllNodesMulticast();

        error = mInfraIf.Send(raMsg.GetAsPacket(), destAddress);

        if (error == kErrorNone)
        {
            mRaInfo.mLastTxTime = TimerMilli::GetNow();
            // TODO: Counter
            LogInfo("Sent RA on %s", mInfraIf.ToString().AsCString());
            DumpDebg("[BR-CERT] direction=send | type=RA |", raMsg.GetAsPacket().GetBytes(),
                     raMsg.GetAsPacket().GetLength());
        }
        else
        {
            // TODO: Counter
            LogWarn("Failed to send RA on %s: %s", mInfraIf.ToString().AsCString(), ErrorToString(error));
        }
    }
}

bool RoutingManagerOffload::IsReceivedRouterAdvertFromManager(const Ip6::Nd::RouterAdvertMessage &aRaMessage) const
{
    // Determines whether or not a received RA message was prepared by
    // by `RoutingManager` itself.

    bool        isFromManager = false;
    uint16_t    rioCount      = 0;
    Ip6::Prefix prefix;

    VerifyOrExit(aRaMessage.ContainsAnyOptions());

    for (const Ip6::Nd::Option &option : aRaMessage)
    {
        switch (option.GetType())
        {
        case Ip6::Nd::Option::kTypePrefixInfo:
        {
            const Ip6::Nd::PrefixInfoOption &pio = static_cast<const Ip6::Nd::PrefixInfoOption &>(option);

            VerifyOrExit(pio.IsValid());
            pio.GetPrefix(prefix);

            // If it is a non-deprecated PIO, it should match the
            // local on-link prefix.

            if (pio.GetPreferredLifetime() > 0)
            {
                VerifyOrExit(prefix == mOnLinkPrefixManager.GetLocalPrefix());
            }

            break;
        }

        case Ip6::Nd::Option::kTypeRouteInfo:
        {
            // RIO (with non-zero lifetime) should match entries from
            // `mAdvertisedPrefixes`. We keep track of the number
            // of matched RIOs and check after the loop ends that all
            // entries were seen.

            const Ip6::Nd::RouteInfoOption &rio = static_cast<const Ip6::Nd::RouteInfoOption &>(option);

            VerifyOrExit(rio.IsValid());
            rio.GetPrefix(prefix);

            if (rio.GetRouteLifetime() != 0)
            {
                VerifyOrExit(mAdvertisedPrefixes.Contains(prefix));
                rioCount++;
            }

            break;
        }

        default:
            ExitNow();
        }
    }

    VerifyOrExit(rioCount == mAdvertisedPrefixes.GetLength());

    isFromManager = true;

exit:
    return isFromManager;
}

bool RoutingManagerOffload::IsValidBrUlaPrefix(const Ip6::Prefix &aBrUlaPrefix)
{
    return aBrUlaPrefix.mLength == kBrUlaPrefixLength && aBrUlaPrefix.mPrefix.mFields.m8[0] == 0xfd;
}

bool RoutingManagerOffload::IsValidOmrPrefix(const NetworkData::OnMeshPrefixConfig &aOnMeshPrefixConfig)
{
    return IsValidOmrPrefix(aOnMeshPrefixConfig.GetPrefix()) && aOnMeshPrefixConfig.mOnMesh &&
           aOnMeshPrefixConfig.mSlaac && aOnMeshPrefixConfig.mStable;
}

bool RoutingManagerOffload::IsValidOmrPrefix(const Ip6::Prefix &aPrefix)
{
    // Accept ULA/GUA prefixes with 64-bit length.
    return (aPrefix.GetLength() == kOmrPrefixLength) && !aPrefix.IsLinkLocal() && !aPrefix.IsMulticast();
}

bool RoutingManagerOffload::IsValidOnLinkPrefix(const Ip6::Nd::PrefixInfoOption &aPio)
{
    Ip6::Prefix prefix;

    aPio.GetPrefix(prefix);

    return IsValidOnLinkPrefix(prefix) && aPio.IsOnLinkFlagSet() && aPio.IsAutoAddrConfigFlagSet();
}

bool RoutingManagerOffload::IsValidOnLinkPrefix(const Ip6::Prefix &aOnLinkPrefix)
{
    return (aOnLinkPrefix.GetLength() == kOnLinkPrefixLength) && !aOnLinkPrefix.IsLinkLocal() &&
           !aOnLinkPrefix.IsMulticast();
}

void RoutingManagerOffload::HandleRsSenderFinished(TimeMilli aStartTime)
{
    // This is a callback from `RsSender` and is invoked when it
    // finishes a cycle of sending Router Solicitations. `aStartTime`
    // specifies the start time of the RS transmission cycle.
    //
    // We remove or deprecate old entries in discovered table that are
    // not refreshed during Router Solicitation. We also invalidate
    // the learned RA header if it is not refreshed during Router
    // Solicitation.

    mDiscoveredPrefixTable.RemoveOrDeprecateOldEntries(aStartTime);

    if (mRaInfo.mHeaderUpdateTime <= aStartTime)
    {
        UpdateRouterAdvertHeader(/* aRouterAdvertMessage */ nullptr);
    }

    ScheduleRoutingPolicyEvaluation(kImmediately);
}

void RoutingManagerOffload::HandleDiscoveredPrefixStaleTimer(void)
{
    LogInfo("Stale On-Link or OMR Prefixes or RA messages are detected");
    mRsSender.Start();
}

void RoutingManagerOffload::HandleRouterSolicit(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress)
{
    OT_UNUSED_VARIABLE(aPacket);
    OT_UNUSED_VARIABLE(aSrcAddress);

    // TODO: Ip6 Counters
    LogInfo("Received RS from %s on %s", aSrcAddress.ToString().AsCString(), mInfraIf.ToString().AsCString());

    ScheduleRoutingPolicyEvaluation(kToReplyToRs);
}

void RoutingManagerOffload::HandleNeighborAdvertisement(const InfraIf::Icmp6Packet &aPacket)
{
    const Ip6::Nd::NeighborAdvertMessage *naMsg;

    VerifyOrExit(aPacket.GetLength() >= sizeof(naMsg));
    naMsg = reinterpret_cast<const Ip6::Nd::NeighborAdvertMessage *>(aPacket.GetBytes());

    mDiscoveredPrefixTable.ProcessNeighborAdvertMessage(*naMsg);

exit:
    return;
}

void RoutingManagerOffload::HandleRouterAdvertisement(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress)
{
    Ip6::Nd::RouterAdvertMessage routerAdvMessage(aPacket);

    OT_ASSERT(mIsRunning);

    VerifyOrExit(routerAdvMessage.IsValid());

    // TODO: Ip6 Counters

    LogInfo("Received RA from %s on %s", aSrcAddress.ToString().AsCString(), mInfraIf.ToString().AsCString());
    DumpDebg("[BR-CERT] direction=recv | type=RA |", aPacket.GetBytes(), aPacket.GetLength());

    mDiscoveredPrefixTable.ProcessRouterAdvertMessage(routerAdvMessage, aSrcAddress);

    // Remember the header and parameters of RA messages which are
    // initiated from the infra interface.
    if (mInfraIf.HasAddress(aSrcAddress))
    {
        UpdateRouterAdvertHeader(&routerAdvMessage);
    }

exit:
    return;
}

bool RoutingManagerOffload::ShouldProcessPrefixInfoOption(const Ip6::Nd::PrefixInfoOption &aPio, const Ip6::Prefix &aPrefix)
{
    // Indicate whether to process or skip a given prefix
    // from a PIO (from received RA message).

    bool shouldProcess = false;

    VerifyOrExit(mIsRunning);

    if (!IsValidOnLinkPrefix(aPio))
    {
        LogInfo("- PIO %s - ignore since not a valid on-link prefix", aPrefix.ToString().AsCString());
        ExitNow();
    }

    if (mOnLinkPrefixManager.IsPublishingOrAdvertising())
    {
        VerifyOrExit(aPrefix != mOnLinkPrefixManager.GetLocalPrefix());
    }

    shouldProcess = true;

exit:
    return shouldProcess;
}

bool RoutingManagerOffload::ShouldProcessRouteInfoOption(const Ip6::Nd::RouteInfoOption &aRio, const Ip6::Prefix &aPrefix)
{
    // Indicate whether to process or skip a given prefix
    // from a RIO (from received RA message).

    OT_UNUSED_VARIABLE(aRio);

    bool shouldProcess = false;

    VerifyOrExit(mIsRunning);

    if (aPrefix.GetLength() == 0)
    {
        // Always process default route ::/0 prefix.
        ExitNow(shouldProcess = true);
    }

    if (!IsValidOmrPrefix(aPrefix))
    {
        LogInfo("- RIO %s - ignore since not a valid OMR prefix", aPrefix.ToString().AsCString());
        ExitNow();
    }

    VerifyOrExit(mOmrPrefixManager.GetLocalPrefix().GetPrefix() != aPrefix);

    // Ignore OMR prefixes advertised by ourselves or in current Thread Network Data.
    // The `mAdvertisedPrefixes` and the OMR prefix set in Network Data should eventually
    // be equal, but there is time that they are not synchronized immediately:
    // 1. Network Data could contain more OMR prefixes than `mAdvertisedPrefixes` because
    //    we added random delay before Evaluating routing policy when Network Data is changed.
    // 2. `mAdvertisedPrefixes` could contain more OMR prefixes than Network Data because
    //    it takes time to sync a new OMR prefix into Network Data (multicast loopback RA
    //    messages are usually faster than Thread Network Data propagation).
    // They are the reasons why we need both the checks.

    VerifyOrExit(!mAdvertisedPrefixes.Contains(aPrefix));
    VerifyOrExit(!Get<RoutingManagerOffload>().NetworkDataContainsOmrPrefix(aPrefix));

    shouldProcess = true;

exit:
    return shouldProcess;
}

void RoutingManagerOffload::HandleDiscoveredPrefixTableChanged(void)
{
    // This is a callback from `mDiscoveredPrefixTable` indicating that
    // there has been a change in the table.

    VerifyOrExit(mIsRunning);

    ResetDiscoveredPrefixStaleTimer();
    mOnLinkPrefixManager.HandleDiscoveredPrefixTableChanged();
    mRoutePublisher.Evaluate();

exit:
    return;
}

bool RoutingManagerOffload::NetworkDataContainsOmrPrefix(const Ip6::Prefix &aPrefix) const
{
    otBorderRouterConfig onMeshPrefixList[10];
    uint8_t count = sizeof(onMeshPrefixList) / sizeof(otBorderRouterConfig);
    bool                            contains = false;

    for (uint8_t i = 0; i < count; i++)
    {
        NetworkData::OnMeshPrefixConfig &prefixConfig = AsCoreType(&onMeshPrefixList[i]);

        if (IsValidOmrPrefix(prefixConfig) && prefixConfig.GetPrefix() == aPrefix)
        {
            contains = true;
            break;
        }
    }

    return contains;
}

bool RoutingManagerOffload::NetworkDataContainsUlaRoute(void) const
{
    // Determine whether leader Network Data contains a route
    // prefix which is either the ULA prefix `fc00::/7` or
    // a sub-prefix of it (e.g., default route).

    otExternalRouteConfig routeConfigList[10];
    uint8_t count = sizeof(routeConfigList) / sizeof(otExternalRouteConfig);
    bool                             contains = false;

    for (uint8_t i = 0; i < count; i++)
    {
        NetworkData::ExternalRouteConfig &routeConfig = AsCoreType(&routeConfigList[i]);
        if (routeConfig.mStable && RoutePublisher::GetUlaPrefix().ContainsPrefix(routeConfig.GetPrefix()))
        {
            contains = true;
            break;
        }
    }

    return contains;
}

void RoutingManagerOffload::UpdateRouterAdvertHeader(const Ip6::Nd::RouterAdvertMessage *aRouterAdvertMessage)
{
    // Updates the `mRaInfo` from the given RA message.

    Ip6::Nd::RouterAdvertMessage::Header oldHeader;

    if (aRouterAdvertMessage != nullptr)
    {
        // We skip and do not update RA header if the received RA message
        // was not prepared and sent by `RoutingManager` itself.

        VerifyOrExit(!IsReceivedRouterAdvertFromManager(*aRouterAdvertMessage));
    }

    oldHeader                 = mRaInfo.mHeader;
    mRaInfo.mHeaderUpdateTime = TimerMilli::GetNow();

    if (aRouterAdvertMessage == nullptr || aRouterAdvertMessage->GetHeader().GetRouterLifetime() == 0)
    {
        mRaInfo.mHeader.SetToDefault();
        mRaInfo.mIsHeaderFromHost = false;
    }
    else
    {
        // The checksum is set to zero in `mRaInfo.mHeader`
        // which indicates to platform that it needs to do the
        // calculation and update it.

        mRaInfo.mHeader = aRouterAdvertMessage->GetHeader();
        mRaInfo.mHeader.SetChecksum(0);
        mRaInfo.mIsHeaderFromHost = true;
    }

    ResetDiscoveredPrefixStaleTimer();

    if (mRaInfo.mHeader != oldHeader)
    {
        // If there was a change to the header, start timer to
        // reevaluate routing policy and send RA message with new
        // header.

        ScheduleRoutingPolicyEvaluation(kAfterRandomDelay);
    }

exit:
    return;
}


void RoutingManagerOffload::ResetDiscoveredPrefixStaleTimer(void)
{
    TimeMilli now = TimerMilli::GetNow();
    TimeMilli nextStaleTime;

    OT_ASSERT(mIsRunning);

    // The stale timer triggers sending RS to check the state of
    // discovered prefixes and host RA messages.

    nextStaleTime = mDiscoveredPrefixTable.CalculateNextStaleTime(now);

    // Check for stale Router Advertisement Message if learnt from Host.
    if (mRaInfo.mIsHeaderFromHost)
    {
        TimeMilli raStaleTime = Max(now, mRaInfo.mHeaderUpdateTime + Time::SecToMsec(kRtrAdvStaleTime));

        nextStaleTime = Min(nextStaleTime, raStaleTime);
    }

    if (nextStaleTime == now.GetDistantFuture())
    {
        if (mDiscoveredPrefixStaleTimer.IsRunning())
        {
            LogDebg("Prefix stale timer stopped");
        }

        mDiscoveredPrefixStaleTimer.Stop();
    }
    else
    {
        mDiscoveredPrefixStaleTimer.FireAt(nextStaleTime);
        LogDebg("Prefix stale timer scheduled in %lu ms", ToUlong(nextStaleTime - now));
    }
}

#if OT_SHOULD_LOG_AT(OT_LOG_LEVEL_INFO)
void RoutingManagerOffload::LogPrefixInfoOption(const Ip6::Prefix &aPrefix,
                                         uint32_t           aValidLifetime,
                                         uint32_t           aPreferredLifetime)
{
    LogInfo("- PIO %s (valid:%lu, preferred:%lu)", aPrefix.ToString().AsCString(), ToUlong(aValidLifetime),
            ToUlong(aPreferredLifetime));
}

void RoutingManagerOffload::LogRouteInfoOption(const Ip6::Prefix &aPrefix, uint32_t aLifetime, RoutePreference aPreference)
{
    LogInfo("- RIO %s (lifetime:%lu, prf:%s)", aPrefix.ToString().AsCString(), ToUlong(aLifetime),
            RoutePreferenceToString(aPreference));
}
#else
void RoutingManagerOffload::LogPrefixInfoOption(const Ip6::Prefix &, uint32_t, uint32_t) {}
void RoutingManagerOffload::LogRouteInfoOption(const Ip6::Prefix &, uint32_t, RoutePreference) {}
#endif

//---------------------------------------------------------------------------------------------------------------------
// DiscoveredPrefixTable

RoutingManagerOffload::DiscoveredPrefixTable::DiscoveredPrefixTable(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mEntryTimer(aInstance)
    , mRouterTimer(aInstance)
    , mSignalTask(aInstance)
{
}

void RoutingManagerOffload::DiscoveredPrefixTable::ProcessRouterAdvertMessage(const Ip6::Nd::RouterAdvertMessage &aRaMessage,
                                                                       const Ip6::Address                 &aSrcAddress)
{
    // Process a received RA message and update the prefix table.

    Router *router = mRouters.FindMatching(aSrcAddress);

    if (router == nullptr)
    {
        router = AllocateRouter();

        if (router == nullptr)
        {
            LogWarn("Received RA from too many routers, ignore RA from %s", aSrcAddress.ToString().AsCString());
            ExitNow();
        }

        router->Clear();
        router->mAddress = aSrcAddress;

        mRouters.Push(*router);
    }

    // RA message can indicate router provides default route in the RA
    // message header and can also include an RIO for `::/0`. When
    // processing an RA message, the preference and lifetime values
    // in a `::/0` RIO override the preference and lifetime values in
    // the RA header (per RFC 4191 section 3.1).

    ProcessRaHeader(aRaMessage.GetHeader(), *router);

    for (const Ip6::Nd::Option &option : aRaMessage)
    {
        switch (option.GetType())
        {
        case Ip6::Nd::Option::kTypePrefixInfo:
            ProcessPrefixInfoOption(static_cast<const Ip6::Nd::PrefixInfoOption &>(option), *router);
            break;

        case Ip6::Nd::Option::kTypeRouteInfo:
            ProcessRouteInfoOption(static_cast<const Ip6::Nd::RouteInfoOption &>(option), *router);
            break;

        case Ip6::Nd::Option::kTypeRaFlagsExtension:
            ProcessRaFlagsExtOption(static_cast<const Ip6::Nd::RaFlagsExtOption &>(option), *router);
            break;

        default:
            break;
        }
    }

    UpdateRouterOnRx(*router);

    RemoveRoutersWithNoEntries();

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::ProcessRaHeader(const Ip6::Nd::RouterAdvertMessage::Header &aRaHeader,
                                                            Router                                     &aRouter)
{
    Entry      *entry;
    Ip6::Prefix prefix;

    aRouter.mManagedAddressConfigFlag = aRaHeader.IsManagedAddressConfigFlagSet();
    aRouter.mOtherConfigFlag          = aRaHeader.IsOtherConfigFlagSet();
    LogInfo("- RA Header - flags - M:%u O:%u", aRouter.mManagedAddressConfigFlag, aRouter.mOtherConfigFlag);

    prefix.Clear();
    entry = aRouter.mEntries.FindMatching(Entry::Matcher(prefix, Entry::kTypeRoute));

    LogInfo("- RA Header - default route - lifetime:%u", aRaHeader.GetRouterLifetime());

    if (entry == nullptr)
    {
        VerifyOrExit(aRaHeader.GetRouterLifetime() != 0);

        entry = AllocateEntry();

        if (entry == nullptr)
        {
            LogWarn("Discovered too many prefixes, ignore default route from RA header");
            ExitNow();
        }

        entry->SetFrom(aRaHeader);
        aRouter.mEntries.Push(*entry);
    }
    else
    {
        entry->SetFrom(aRaHeader);
    }

    mEntryTimer.FireAtIfEarlier(entry->GetExpireTime());

    SignalTableChanged();

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::ProcessPrefixInfoOption(const Ip6::Nd::PrefixInfoOption &aPio,
                                                                    Router                          &aRouter)
{
    Ip6::Prefix prefix;
    Entry      *entry;

    VerifyOrExit(aPio.IsValid());
    aPio.GetPrefix(prefix);

    VerifyOrExit(Get<RoutingManagerOffload>().ShouldProcessPrefixInfoOption(aPio, prefix));

    LogPrefixInfoOption(prefix, aPio.GetValidLifetime(), aPio.GetPreferredLifetime());

    entry = aRouter.mEntries.FindMatching(Entry::Matcher(prefix, Entry::kTypeOnLink));

    if (entry == nullptr)
    {
        VerifyOrExit(aPio.GetValidLifetime() != 0);

        entry = AllocateEntry();

        if (entry == nullptr)
        {
            LogWarn("Discovered too many prefixes, ignore on-link prefix %s", prefix.ToString().AsCString());
            ExitNow();
        }

        entry->SetFrom(aPio);
        aRouter.mEntries.Push(*entry);
    }
    else
    {
        Entry newEntry;

        newEntry.SetFrom(aPio);
        entry->AdoptValidAndPreferredLifetimesFrom(newEntry);
    }

    mEntryTimer.FireAtIfEarlier(entry->GetExpireTime());

    SignalTableChanged();

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::ProcessRouteInfoOption(const Ip6::Nd::RouteInfoOption &aRio,
                                                                   Router                         &aRouter)
{
    Ip6::Prefix prefix;
    Entry      *entry;

    VerifyOrExit(aRio.IsValid());
    aRio.GetPrefix(prefix);

    VerifyOrExit(Get<RoutingManagerOffload>().ShouldProcessRouteInfoOption(aRio, prefix));

    LogRouteInfoOption(prefix, aRio.GetRouteLifetime(), aRio.GetPreference());

    entry = aRouter.mEntries.FindMatching(Entry::Matcher(prefix, Entry::kTypeRoute));

    if (entry == nullptr)
    {
        VerifyOrExit(aRio.GetRouteLifetime() != 0);

        entry = AllocateEntry();

        if (entry == nullptr)
        {
            LogWarn("Discovered too many prefixes, ignore route prefix %s", prefix.ToString().AsCString());
            ExitNow();
        }

        entry->SetFrom(aRio);
        aRouter.mEntries.Push(*entry);
    }
    else
    {
        entry->SetFrom(aRio);
    }

    mEntryTimer.FireAtIfEarlier(entry->GetExpireTime());

    SignalTableChanged();

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::ProcessRaFlagsExtOption(const Ip6::Nd::RaFlagsExtOption &aRaFlagsOption,
                                                                    Router                          &aRouter)
{
    VerifyOrExit(aRaFlagsOption.IsValid());
    aRouter.mStubRouterFlag = aRaFlagsOption.IsStubRouterFlagSet();

    LogInfo("- FlagsExt - StubRouter:%u", aRouter.mStubRouterFlag);

exit:
    return;
}

bool RoutingManagerOffload::DiscoveredPrefixTable::Contains(const Entry::Checker &aChecker) const
{
    bool contains = false;

    for (const Router &router : mRouters)
    {
        if (router.mEntries.ContainsMatching(aChecker))
        {
            contains = true;
            break;
        }
    }

    return contains;
}

bool RoutingManagerOffload::DiscoveredPrefixTable::ContainsDefaultOrNonUlaRoutePrefix(void) const
{
    return Contains(Entry::Checker(Entry::Checker::kIsNotUla, Entry::kTypeRoute));
}

bool RoutingManagerOffload::DiscoveredPrefixTable::ContainsNonUlaOnLinkPrefix(void) const
{
    return Contains(Entry::Checker(Entry::Checker::kIsNotUla, Entry::kTypeOnLink));
}

bool RoutingManagerOffload::DiscoveredPrefixTable::ContainsUlaOnLinkPrefix(void) const
{
    return Contains(Entry::Checker(Entry::Checker::kIsUla, Entry::kTypeOnLink));
}

void RoutingManagerOffload::DiscoveredPrefixTable::FindFavoredOnLinkPrefix(Ip6::Prefix &aPrefix) const
{
    // Find the smallest preferred on-link prefix entry in the table
    // and return it in `aPrefix`. If there is none, `aPrefix` is
    // cleared (prefix length is set to zero).

    aPrefix.Clear();

    for (const Router &router : mRouters)
    {
        for (const Entry &entry : router.mEntries)
        {
            if (!entry.IsOnLinkPrefix() || entry.IsDeprecated())
            {
                continue;
            }

            if ((aPrefix.GetLength() == 0) || (entry.GetPrefix() < aPrefix))
            {
                aPrefix = entry.GetPrefix();
            }
        }
    }
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveOnLinkPrefix(const Ip6::Prefix &aPrefix)
{
    RemovePrefix(Entry::Matcher(aPrefix, Entry::kTypeOnLink));
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveRoutePrefix(const Ip6::Prefix &aPrefix)
{
    RemovePrefix(Entry::Matcher(aPrefix, Entry::kTypeRoute));
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemovePrefix(const Entry::Matcher &aMatcher)
{
    // Removes all entries matching a given prefix from the table.

    LinkedList<Entry> removedEntries;

    for (Router &router : mRouters)
    {
        router.mEntries.RemoveAllMatching(aMatcher, removedEntries);
    }

    VerifyOrExit(!removedEntries.IsEmpty());

    FreeEntries(removedEntries);
    RemoveRoutersWithNoEntries();

    SignalTableChanged();

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveAllEntries(void)
{
    // Remove all entries from the table and unpublish them
    // from Network Data.

    for (Router &router : mRouters)
    {
        if (!router.mEntries.IsEmpty())
        {
            SignalTableChanged();
        }

        FreeEntries(router.mEntries);
    }

    RemoveRoutersWithNoEntries();
    mEntryTimer.Stop();
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveOrDeprecateOldEntries(TimeMilli aTimeThreshold)
{
    // Remove route prefix entries and deprecate on-link entries in
    // the table that are old (not updated since `aTimeThreshold`).

    for (Router &router : mRouters)
    {
        for (Entry &entry : router.mEntries)
        {
            if (entry.GetLastUpdateTime() <= aTimeThreshold)
            {
                if (entry.IsOnLinkPrefix())
                {
                    entry.ClearPreferredLifetime();
                }
                else
                {
                    entry.ClearValidLifetime();
                }

                SignalTableChanged();
            }
        }
    }

    RemoveExpiredEntries();
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveOrDeprecateEntriesFromInactiveRouters(void)
{
    // Remove route prefix entries and deprecate on-link prefix entries
    // in the table for routers that have reached the max NS probe
    // attempts and considered as inactive.

    for (Router &router : mRouters)
    {
        if (router.mNsProbeCount <= Router::kMaxNsProbes)
        {
            continue;
        }

        for (Entry &entry : router.mEntries)
        {
            if (entry.IsOnLinkPrefix())
            {
                if (!entry.IsDeprecated())
                {
                    entry.ClearPreferredLifetime();
                    SignalTableChanged();
                }
            }
            else
            {
                entry.ClearValidLifetime();
            }
        }
    }

    RemoveExpiredEntries();
}

TimeMilli RoutingManagerOffload::DiscoveredPrefixTable::CalculateNextStaleTime(TimeMilli aNow) const
{
    TimeMilli onLinkStaleTime = aNow;
    TimeMilli routeStaleTime  = aNow.GetDistantFuture();
    bool      foundOnLink     = false;

    // For on-link prefixes, we consider stale time as when all on-link
    // prefixes become stale (the latest stale time) but for route
    // prefixes we consider the earliest stale time.

    for (const Router &router : mRouters)
    {
        for (const Entry &entry : router.mEntries)
        {
            TimeMilli entryStaleTime = Max(aNow, entry.GetStaleTime());

            if (entry.IsOnLinkPrefix() && !entry.IsDeprecated())
            {
                onLinkStaleTime = Max(onLinkStaleTime, entryStaleTime);
                foundOnLink     = true;
            }

            if (!entry.IsOnLinkPrefix())
            {
                routeStaleTime = Min(routeStaleTime, entryStaleTime);
            }
        }
    }

    return foundOnLink ? Min(onLinkStaleTime, routeStaleTime) : routeStaleTime;
}

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveRoutersWithNoEntries(void)
{
    LinkedList<Router> routersToFree;

    mRouters.RemoveAllMatching(Router::kContainsNoEntries, routersToFree);
    FreeRouters(routersToFree);
}

void RoutingManagerOffload::DiscoveredPrefixTable::FreeRouters(LinkedList<Router> &aRouters)
{
    // Frees all routers in the given list `aRouters`

    Router *router;

    while ((router = aRouters.Pop()) != nullptr)
    {
        FreeRouter(*router);
    }
}

void RoutingManagerOffload::DiscoveredPrefixTable::FreeEntries(LinkedList<Entry> &aEntries)
{
    // Frees all entries in the given list `aEntries`.

    Entry *entry;

    while ((entry = aEntries.Pop()) != nullptr)
    {
        FreeEntry(*entry);
    }
}

const RoutingManagerOffload::DiscoveredPrefixTable::Entry *RoutingManagerOffload::DiscoveredPrefixTable::FindFavoredEntryToPublish(
    const Ip6::Prefix &aPrefix) const
{
    // Finds the favored entry matching a given `aPrefix` in the table
    // to publish in the Network Data. We can have multiple entries
    // in the table matching the same `aPrefix` from different
    // routers and potentially with different preference values. We
    // select the one with the highest preference as the favored
    // entry to publish.

    const Entry *favoredEntry = nullptr;

    for (const Router &router : mRouters)
    {
        for (const Entry &entry : router.mEntries)
        {
            if (entry.GetPrefix() != aPrefix)
            {
                continue;
            }

            if ((favoredEntry == nullptr) || (entry.GetPreference() > favoredEntry->GetPreference()))
            {
                favoredEntry = &entry;
            }
        }
    }

    return favoredEntry;
}

void RoutingManagerOffload::DiscoveredPrefixTable::HandleEntryTimer(void) { RemoveExpiredEntries(); }

void RoutingManagerOffload::DiscoveredPrefixTable::RemoveExpiredEntries(void)
{
    TimeMilli         now            = TimerMilli::GetNow();
    TimeMilli         nextExpireTime = now.GetDistantFuture();
    LinkedList<Entry> expiredEntries;

    for (Router &router : mRouters)
    {
        router.mEntries.RemoveAllMatching(Entry::ExpirationChecker(now), expiredEntries);
    }

    RemoveRoutersWithNoEntries();

    if (!expiredEntries.IsEmpty())
    {
        SignalTableChanged();
    }

    FreeEntries(expiredEntries);

    // Determine the next expire time and schedule timer.

    for (const Router &router : mRouters)
    {
        for (const Entry &entry : router.mEntries)
        {
            nextExpireTime = Min(nextExpireTime, entry.GetExpireTime());
        }
    }

    if (nextExpireTime != now.GetDistantFuture())
    {
        mEntryTimer.FireAt(nextExpireTime);
    }
}

void RoutingManagerOffload::DiscoveredPrefixTable::SignalTableChanged(void) { mSignalTask.Post(); }

void RoutingManagerOffload::DiscoveredPrefixTable::ProcessNeighborAdvertMessage(
    const Ip6::Nd::NeighborAdvertMessage &aNaMessage)
{
    Router *router;

    VerifyOrExit(aNaMessage.IsValid());

    router = mRouters.FindMatching(aNaMessage.GetTargetAddress());
    VerifyOrExit(router != nullptr);

    LogInfo("Received NA from router %s", router->mAddress.ToString().AsCString());

    UpdateRouterOnRx(*router);

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::UpdateRouterOnRx(Router &aRouter)
{
    aRouter.mNsProbeCount = 0;
    aRouter.mTimeout = TimerMilli::GetNow() + Random::NonCrypto::AddJitter(Router::kActiveTimeout, Router::kJitter);

    mRouterTimer.FireAtIfEarlier(aRouter.mTimeout);
}

void RoutingManagerOffload::DiscoveredPrefixTable::HandleRouterTimer(void)
{
    TimeMilli now      = TimerMilli::GetNow();
    TimeMilli nextTime = now.GetDistantFuture();

    for (Router &router : mRouters)
    {
        if (router.mNsProbeCount > Router::kMaxNsProbes)
        {
            continue;
        }

        // If the `router` emitting RA has an address belonging to
        // infra interface, it indicates that the RAs are from
        // same device. In this case we skip performing NS probes.
        // This addresses situation where platform may not be
        // be able to receive and pass the NA message response
        // from device itself.

        if (Get<RoutingManagerOffload>().mInfraIf.HasAddress(router.mAddress))
        {
            continue;
        }

        if (router.mTimeout <= now)
        {
            router.mNsProbeCount++;

            if (router.mNsProbeCount > Router::kMaxNsProbes)
            {
                LogInfo("No response to all Neighbor Solicitations attempts from router %s",
                        router.mAddress.ToString().AsCString());
                continue;
            }

            router.mTimeout = now + ((router.mNsProbeCount < Router::kMaxNsProbes) ? Router::kNsProbeRetryInterval
                                                                                   : Router::kNsProbeTimeout);

            SendNeighborSolicitToRouter(router);
        }

        nextTime = Min(nextTime, router.mTimeout);
    }

    RemoveOrDeprecateEntriesFromInactiveRouters();

    if (nextTime != now.GetDistantFuture())
    {
        mRouterTimer.FireAtIfEarlier(nextTime);
    }
}

void RoutingManagerOffload::DiscoveredPrefixTable::SendNeighborSolicitToRouter(const Router &aRouter)
{
    InfraIf::Icmp6Packet            packet;
    Ip6::Nd::NeighborSolicitMessage neighborSolicitMsg;

    VerifyOrExit(!Get<RoutingManagerOffload>().mRsSender.IsInProgress());

    neighborSolicitMsg.SetTargetAddress(aRouter.mAddress);
    packet.InitFrom(neighborSolicitMsg);

    IgnoreError(Get<RoutingManagerOffload>().mInfraIf.Send(packet, aRouter.mAddress));

    LogInfo("Sent Neighbor Solicitation to %s - attempt:%u/%u", aRouter.mAddress.ToString().AsCString(),
            aRouter.mNsProbeCount, Router::kMaxNsProbes);

exit:
    return;
}

void RoutingManagerOffload::DiscoveredPrefixTable::InitIterator(PrefixTableIterator &aIterator) const
{
    Iterator &iterator = static_cast<Iterator &>(aIterator);

    iterator.SetInitTime();
    iterator.SetRouter(mRouters.GetHead());
    iterator.SetEntry(mRouters.IsEmpty() ? nullptr : mRouters.GetHead()->mEntries.GetHead());
}

Error RoutingManagerOffload::DiscoveredPrefixTable::GetNextEntry(PrefixTableIterator &aIterator,
                                                          PrefixTableEntry    &aEntry) const
{
    Error     error    = kErrorNone;
    Iterator &iterator = static_cast<Iterator &>(aIterator);

    VerifyOrExit(iterator.GetRouter() != nullptr, error = kErrorNotFound);
    OT_ASSERT(iterator.GetEntry() != nullptr);

    aEntry.mRouterAddress       = iterator.GetRouter()->mAddress;
    aEntry.mPrefix              = iterator.GetEntry()->GetPrefix();
    aEntry.mIsOnLink            = iterator.GetEntry()->IsOnLinkPrefix();
    aEntry.mMsecSinceLastUpdate = iterator.GetInitTime() - iterator.GetEntry()->GetLastUpdateTime();
    aEntry.mValidLifetime       = iterator.GetEntry()->GetValidLifetime();
    aEntry.mPreferredLifetime   = aEntry.mIsOnLink ? iterator.GetEntry()->GetPreferredLifetime() : 0;
    aEntry.mRoutePreference =
        static_cast<otRoutePreference>(aEntry.mIsOnLink ? 0 : iterator.GetEntry()->GetRoutePreference());

    // Advance the iterator
    iterator.SetEntry(iterator.GetEntry()->GetNext());

    if (iterator.GetEntry() == nullptr)
    {
        iterator.SetRouter(iterator.GetRouter()->GetNext());

        if (iterator.GetRouter() != nullptr)
        {
            iterator.SetEntry(iterator.GetRouter()->mEntries.GetHead());
        }
    }

exit:
    return error;
}

//---------------------------------------------------------------------------------------------------------------------
// DiscoveredPrefixTable::Entry

void RoutingManagerOffload::DiscoveredPrefixTable::Entry::SetFrom(const Ip6::Nd::RouterAdvertMessage::Header &aRaHeader)
{
    mPrefix.Clear();
    mType                    = kTypeRoute;
    mValidLifetime           = aRaHeader.GetRouterLifetime();
    mShared.mRoutePreference = aRaHeader.GetDefaultRouterPreference();
    mLastUpdateTime          = TimerMilli::GetNow();
}

void RoutingManagerOffload::DiscoveredPrefixTable::Entry::SetFrom(const Ip6::Nd::PrefixInfoOption &aPio)
{
    aPio.GetPrefix(mPrefix);
    mType                      = kTypeOnLink;
    mValidLifetime             = aPio.GetValidLifetime();
    mShared.mPreferredLifetime = aPio.GetPreferredLifetime();
    mLastUpdateTime            = TimerMilli::GetNow();
}

void RoutingManagerOffload::DiscoveredPrefixTable::Entry::SetFrom(const Ip6::Nd::RouteInfoOption &aRio)
{
    aRio.GetPrefix(mPrefix);
    mType                    = kTypeRoute;
    mValidLifetime           = aRio.GetRouteLifetime();
    mShared.mRoutePreference = aRio.GetPreference();
    mLastUpdateTime          = TimerMilli::GetNow();
}

bool RoutingManagerOffload::DiscoveredPrefixTable::Entry::operator==(const Entry &aOther) const
{
    return (mType == aOther.mType) && (mPrefix == aOther.mPrefix);
}

bool RoutingManagerOffload::DiscoveredPrefixTable::Entry::Matches(const Matcher &aMatcher) const
{
    return (mType == aMatcher.mType) && (mPrefix == aMatcher.mPrefix);
}

bool RoutingManagerOffload::DiscoveredPrefixTable::Entry::Matches(const Checker &aChecker) const
{
    return (mType == aChecker.mType) && (mPrefix.IsUniqueLocal() == (aChecker.mMode == Checker::kIsUla));
}

bool RoutingManagerOffload::DiscoveredPrefixTable::Entry::Matches(const ExpirationChecker &aChecker) const
{
    return GetExpireTime() <= aChecker.mNow;
}

TimeMilli RoutingManagerOffload::DiscoveredPrefixTable::Entry::GetExpireTime(void) const
{
    return mLastUpdateTime + CalculateExpireDelay(mValidLifetime);
}

TimeMilli RoutingManagerOffload::DiscoveredPrefixTable::Entry::GetStaleTime(void) const
{
    uint32_t delay = Min(kRtrAdvStaleTime, IsOnLinkPrefix() ? GetPreferredLifetime() : mValidLifetime);

    return mLastUpdateTime + TimeMilli::SecToMsec(delay);
}

bool RoutingManagerOffload::DiscoveredPrefixTable::Entry::IsDeprecated(void) const
{
    OT_ASSERT(IsOnLinkPrefix());

    return mLastUpdateTime + TimeMilli::SecToMsec(GetPreferredLifetime()) <= TimerMilli::GetNow();
}

RoutingManagerOffload::RoutePreference RoutingManagerOffload::DiscoveredPrefixTable::Entry::GetPreference(void) const
{
    // Returns the preference level to use when we publish
    // the prefix entry in Network Data.

    return IsOnLinkPrefix() ? NetworkData::kRoutePreferenceMedium : GetRoutePreference();
}

void RoutingManagerOffload::DiscoveredPrefixTable::Entry::AdoptValidAndPreferredLifetimesFrom(const Entry &aEntry)
{
    constexpr uint32_t kTwoHoursInSeconds = 2 * 3600;

    // Per RFC 4862 section 5.5.3.e:
    //
    // 1.  If the received Valid Lifetime is greater than 2 hours or
    //     greater than RemainingLifetime, set the valid lifetime of the
    //     corresponding address to the advertised Valid Lifetime.
    // 2.  If RemainingLifetime is less than or equal to 2 hours, ignore
    //     the Prefix Information option with regards to the valid
    //     lifetime, unless ...
    // 3.  Otherwise, reset the valid lifetime of the corresponding
    //     address to 2 hours.

    if (aEntry.mValidLifetime > kTwoHoursInSeconds || aEntry.GetExpireTime() > GetExpireTime())
    {
        mValidLifetime = aEntry.mValidLifetime;
    }
    else if (GetExpireTime() > TimerMilli::GetNow() + TimeMilli::SecToMsec(kTwoHoursInSeconds))
    {
        mValidLifetime = kTwoHoursInSeconds;
    }

    mShared.mPreferredLifetime = aEntry.GetPreferredLifetime();
    mLastUpdateTime            = aEntry.GetLastUpdateTime();
}

uint32_t RoutingManagerOffload::DiscoveredPrefixTable::Entry::CalculateExpireDelay(uint32_t aValidLifetime)
{
    uint32_t delay;

    if (aValidLifetime * static_cast<uint64_t>(1000) > Timer::kMaxDelay)
    {
        delay = Timer::kMaxDelay;
    }
    else
    {
        delay = aValidLifetime * 1000;
    }

    return delay;
}

//---------------------------------------------------------------------------------------------------------------------
// FavoredOmrPrefix

bool RoutingManagerOffload::FavoredOmrPrefix::IsInfrastructureDerived(void) const
{
    // Indicate whether the OMR prefix is infrastructure-derived which
    // can be identified as a valid OMR prefix with preference of
    // medium or higher.

    return !IsEmpty() && (mPreference >= NetworkData::kRoutePreferenceMedium);
}

void RoutingManagerOffload::FavoredOmrPrefix::SetFrom(const NetworkData::OnMeshPrefixConfig &aOnMeshPrefixConfig)
{
    mPrefix         = aOnMeshPrefixConfig.GetPrefix();
    mPreference     = aOnMeshPrefixConfig.GetPreference();
    mIsDomainPrefix = aOnMeshPrefixConfig.mDp;
}

void RoutingManagerOffload::FavoredOmrPrefix::SetFrom(const OmrPrefix &aOmrPrefix)
{
    mPrefix         = aOmrPrefix.GetPrefix();
    mPreference     = aOmrPrefix.GetPreference();
    mIsDomainPrefix = aOmrPrefix.IsDomainPrefix();
}

bool RoutingManagerOffload::FavoredOmrPrefix::IsFavoredOver(const NetworkData::OnMeshPrefixConfig &aOmrPrefixConfig) const
{
    // This method determines whether this OMR prefix is favored
    // over another prefix. A prefix with higher preference is
    // favored. If the preference is the same, then the smaller
    // prefix (in the sense defined by `Ip6::Prefix`) is favored.

    bool isFavored = (mPreference > aOmrPrefixConfig.GetPreference());

    OT_ASSERT(IsValidOmrPrefix(aOmrPrefixConfig));

    if (mPreference == aOmrPrefixConfig.GetPreference())
    {
        isFavored = (mPrefix < aOmrPrefixConfig.GetPrefix());
    }

    return isFavored;
}

//---------------------------------------------------------------------------------------------------------------------
// OmrPrefixManager

RoutingManagerOffload::OmrPrefixManager::OmrPrefixManager(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mIsLocalAddedInNetData(false)
    , mDefaultRoute(false)
{
}

void RoutingManagerOffload::OmrPrefixManager::Init(const Ip6::Prefix &aBrUlaPrefix)
{
    mGeneratedPrefix = aBrUlaPrefix;
    mGeneratedPrefix.SetSubnetId(kOmrPrefixSubnetId);
    mGeneratedPrefix.SetLength(kOmrPrefixLength);

    LogInfo("Generated local OMR prefix: %s", mGeneratedPrefix.ToString().AsCString());
}

void RoutingManagerOffload::OmrPrefixManager::Start(void) { DetermineFavoredPrefix(); }

void RoutingManagerOffload::OmrPrefixManager::Stop(void)
{
    RemoveLocalFromNetData();
    mFavoredPrefix.Clear();
}

void RoutingManagerOffload::OmrPrefixManager::DetermineFavoredPrefix(void)
{
    otError error = OT_ERROR_NONE;

    otBorderRouterConfig onMeshPrefixList[10];
    uint8_t count = sizeof(onMeshPrefixList) / sizeof(otBorderRouterConfig);

    mFavoredPrefix.Clear();

    SuccessOrExit(error = otPlatCpNetDataGetOnMeshPrefix(onMeshPrefixList, count));

    for (uint8_t i = 0; i < count; i++)
    {
        NetworkData::OnMeshPrefixConfig prefixConfig = AsCoreType(&onMeshPrefixList[i]);
        if (!IsValidOmrPrefix(prefixConfig) || !prefixConfig.mPreferred)
        {
            continue;
        }

        if (mFavoredPrefix.IsEmpty() || !mFavoredPrefix.IsFavoredOver(prefixConfig))
        {
            mFavoredPrefix.SetFrom(prefixConfig);
        }
    }

exit:
    return;
}

void RoutingManagerOffload::OmrPrefixManager::Evaluate(void)
{
    LogInfo("OmrPrefixManager Evaluate");
    OT_ASSERT(Get<RoutingManagerOffload>().IsRunning());

    DetermineFavoredPrefix();

    // Determine the local prefix and remove outdated prefix published by us.
    if (mLocalPrefix.GetPrefix() != mGeneratedPrefix)
    {
        RemoveLocalFromNetData();
        mLocalPrefix.mPrefix         = mGeneratedPrefix;
        mLocalPrefix.mPreference     = RoutePreference::kRoutePreferenceLow;
        mLocalPrefix.mIsDomainPrefix = false;
        LogInfo("Setting local OMR prefix to generated prefix: %s", mLocalPrefix.GetPrefix().ToString().AsCString());
    }

    // Decide if we need to add or remove our local OMR prefix.
    if (mFavoredPrefix.IsEmpty() || mFavoredPrefix.GetPreference() < mLocalPrefix.GetPreference())
    {
        if (mFavoredPrefix.IsEmpty())
        {
            LogInfo("No favored OMR prefix found in Thread network.");
        }
        else
        {
            LogInfo("Replacing favored OMR prefix %s with higher preference local prefix %s.",
                    mFavoredPrefix.GetPrefix().ToString().AsCString(), mLocalPrefix.GetPrefix().ToString().AsCString());
        }

        // The `mFavoredPrefix` remains empty if we fail to publish
        // the local OMR prefix.
        SuccessOrExit(AddLocalToNetData());

        mFavoredPrefix.SetFrom(mLocalPrefix);
    }
    else if (mFavoredPrefix.GetPrefix() == mLocalPrefix.GetPrefix())
    {
        IgnoreError(AddLocalToNetData());
    }
    else if (mIsLocalAddedInNetData)
    {
        LogInfo("There is already a favored OMR prefix %s in the Thread network",
                mFavoredPrefix.GetPrefix().ToString().AsCString());

        RemoveLocalFromNetData();
    }

exit:
    return;
}

Error RoutingManagerOffload::OmrPrefixManager::AddLocalToNetData(void)
{
    // TODO: NCP add notifier HandleServerDataUpdate
    Error error = kErrorNone;

    VerifyOrExit(!mIsLocalAddedInNetData);
    SuccessOrExit(error = AddOrUpdateLocalInNetData());
    mIsLocalAddedInNetData = true;

exit:
    return error;
}

Error RoutingManagerOffload::OmrPrefixManager::AddOrUpdateLocalInNetData(void)
{
    // Add the local OMR prefix in Thread Network Data or update it
    // (e.g., change default route flag) if it is already added.

    Error                           error;
    NetworkData::OnMeshPrefixConfig config;

    config.Clear();
    config.mPrefix       = mLocalPrefix.GetPrefix();
    config.mStable       = true;
    config.mSlaac        = true;
    config.mPreferred    = true;
    config.mOnMesh       = true;
    config.mDefaultRoute = mDefaultRoute;
    config.mPreference   = mLocalPrefix.GetPreference();

    error = otPlatCpBorderRouterAddOnMeshPrefix(&config);

    if (error != kErrorNone)
    {
        LogWarn("Failed to %s %s in Thread Network Data: %s", !mIsLocalAddedInNetData ? "add" : "update",
                LocalToString().AsCString(), ErrorToString(error));
        ExitNow();
    }

    LogInfo("%s %s in Thread Network Data", !mIsLocalAddedInNetData ? "Added" : "Updated", LocalToString().AsCString());

exit:
    return error;
}

void RoutingManagerOffload::OmrPrefixManager::RemoveLocalFromNetData(void)
{
    Error error = kErrorNone;

    VerifyOrExit(mIsLocalAddedInNetData);

    error = otPlatCpBorderRouterRemoveOnMeshPrefix(&mLocalPrefix.GetPrefix());

    if (error != kErrorNone)
    {
        LogWarn("Failed to remove %s from Thread Network Data: %s", LocalToString().AsCString(), ErrorToString(error));
        ExitNow();
    }

    mIsLocalAddedInNetData = false;
    LogInfo("Removed %s from Thread Network Data", LocalToString().AsCString());

exit:
    return;
}

void RoutingManagerOffload::OmrPrefixManager::UpdateDefaultRouteFlag(bool aDefaultRoute)
{
    VerifyOrExit(aDefaultRoute != mDefaultRoute);

    mDefaultRoute = aDefaultRoute;

    VerifyOrExit(mIsLocalAddedInNetData);
    IgnoreError(AddOrUpdateLocalInNetData());

exit:
    return;
}

RoutingManagerOffload::OmrPrefixManager::InfoString RoutingManagerOffload::OmrPrefixManager::LocalToString(void) const
{
    InfoString string;

    string.Append("local OMR prefix %s (def-route:%s)", mLocalPrefix.GetPrefix().ToString().AsCString(),
                  ToYesNo(mDefaultRoute));
    return string;
}

//---------------------------------------------------------------------------------------------------------------------
// OnLinkPrefixManager

RoutingManagerOffload::OnLinkPrefixManager::OnLinkPrefixManager(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mState(kIdle)
    , mTimer(aInstance)
{
    mLocalPrefix.Clear();
    mFavoredDiscoveredPrefix.Clear();
    mOldLocalPrefixes.Clear();
}

void RoutingManagerOffload::OnLinkPrefixManager::SetState(State aState)
{
    VerifyOrExit(mState != aState);

    LogInfo("Local on-link prefix state: %s -> %s (%s)", StateToString(mState), StateToString(aState),
            mLocalPrefix.ToString().AsCString());
    mState = aState;

    // Mark the Advertising PIO (AP) flag in the published route, when
    // the local on-link prefix is being published, advertised, or
    // deprecated.

    Get<RoutingManagerOffload>().mRoutePublisher.UpdateAdvPioFlags(aState != kIdle);

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::Init(void)
{
     TimeMilli                now = TimerMilli::GetNow();
     Settings::BrOnLinkPrefix savedPrefix;
     bool                     refreshStoredPrefixes = false;

     // Restore old prefixes from `Settings`

     for (int index = 0; Get<Settings>().ReadBrOnLinkPrefix(index, savedPrefix) == kErrorNone; index++)
     {
         uint32_t   lifetime;
         OldPrefix *entry;

         if (mOldLocalPrefixes.ContainsMatching(savedPrefix.GetPrefix()))
         {
             // We should not see duplicate entries in `Settings`
             // but if we do we refresh the stored prefixes to make
             // it consistent.
             refreshStoredPrefixes = true;
             continue;
         }

         entry = mOldLocalPrefixes.PushBack();

         if (entry == nullptr)
         {
             // If there are more stored prefixes, we refresh the
             // prefixes in `Settings` to remove the ones we cannot
             // handle.

             refreshStoredPrefixes = true;
             break;
         }

         lifetime = Min(savedPrefix.GetLifetime(), Time::MsecToSec(TimerMilli::kMaxDelay));

         entry->mPrefix     = savedPrefix.GetPrefix();
         entry->mExpireTime = now + Time::SecToMsec(lifetime);

         LogInfo("Restored old prefix %s, lifetime:%lu", entry->mPrefix.ToString().AsCString(), ToUlong(lifetime));

         mTimer.FireAtIfEarlier(entry->mExpireTime);
     }

     if (refreshStoredPrefixes)
     {
         // We clear the entries in `Settings` and re-write the entries
         // from `mOldLocalPrefixes` array.

         IgnoreError(Get<Settings>().DeleteAllBrOnLinkPrefixes());

         for (OldPrefix &oldPrefix : mOldLocalPrefixes)
         {
             SavePrefix(oldPrefix.mPrefix, oldPrefix.mExpireTime);
         }
     }

     GenerateLocalPrefix();
}

void RoutingManagerOffload::OnLinkPrefixManager::GenerateLocalPrefix(void)
{
    otExtendedPanId extPanId_;
    otPlatCpThreadGetExtendedPanIdCached(&extPanId_);
    const MeshCoP::ExtendedPanId &extPanId = AsCoreType(&extPanId_);
    OldPrefix                    *entry;
    Ip6::Prefix                   oldLocalPrefix = mLocalPrefix;

    // Global ID: 40 most significant bits of Extended PAN ID
    // Subnet ID: 16 least significant bits of Extended PAN ID

    mLocalPrefix.mPrefix.mFields.m8[0] = 0xfd;
    memcpy(mLocalPrefix.mPrefix.mFields.m8 + 1, extPanId.m8, 5);
    memcpy(mLocalPrefix.mPrefix.mFields.m8 + 6, extPanId.m8 + 6, 2);

    mLocalPrefix.SetLength(kOnLinkPrefixLength);

    // We ensure that the local prefix did change, since not all the
    // bytes in Extended PAN ID are used in derivation of the local prefix.

    VerifyOrExit(mLocalPrefix != oldLocalPrefix);

    LogNote("Local on-link prefix: %s", mLocalPrefix.ToString().AsCString());

    // Check if the new local prefix happens to be in `mOldLocalPrefixes` array.
    // If so, we remove it from the array and update the state accordingly.

    entry = mOldLocalPrefixes.FindMatching(mLocalPrefix);

    if (entry != nullptr)
    {
        SetState(kDeprecating);
        mExpireTime = entry->mExpireTime;
        mOldLocalPrefixes.Remove(*entry);
    }
    else
    {
        SetState(kIdle);
    }

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::Start(void) {}

void RoutingManagerOffload::OnLinkPrefixManager::Stop(void)
{
    mFavoredDiscoveredPrefix.Clear();

    switch (GetState())
    {
    case kIdle:
        break;

    case kPublishing:
    case kAdvertising:
    case kDeprecating:
        SetState(kDeprecating);
        break;
    }
}

void RoutingManagerOffload::OnLinkPrefixManager::Evaluate(void)
{
    VerifyOrExit(!Get<RoutingManagerOffload>().mRsSender.IsInProgress());

    Get<RoutingManagerOffload>().mDiscoveredPrefixTable.FindFavoredOnLinkPrefix(mFavoredDiscoveredPrefix);

    if ((mFavoredDiscoveredPrefix.GetLength() == 0) || (mFavoredDiscoveredPrefix == mLocalPrefix))
    {
        // We need to advertise our local on-link prefix when there is
        // no discovered on-link prefix. If the favored discovered
        // prefix is the same as our local on-link prefix we also
        // start advertising the local prefix to add redundancy. Note
        // that local on-link prefix is derived from extended PAN ID
        // and therefore is the same for all BRs on the same Thread
        // mesh.

        PublishAndAdvertise();

        // We remove the local on-link prefix from discovered prefix
        // table, in case it was previously discovered and included in
        // the table (now as a deprecating entry). We remove it with
        // `kKeepInNetData` flag to ensure that the prefix is not
        // unpublished from network data.
        //
        // Note that `ShouldProcessPrefixInfoOption()` will also check
        // not allow the local on-link prefix to be added in the prefix
        // table while we are advertising it.

        Get<RoutingManagerOffload>().mDiscoveredPrefixTable.RemoveOnLinkPrefix(mLocalPrefix);

        mFavoredDiscoveredPrefix.Clear();
    }
    else if (IsPublishingOrAdvertising())
    {
        // When an application-specific on-link prefix is received and
        // it is larger than the local prefix, we will not remove the
        // advertised local prefix. In this case, there will be two
        // on-link prefixes on the infra link. But all BRs will still
        // converge to the same smallest/favored on-link prefix and the
        // application-specific prefix is not used.

        if (!(mLocalPrefix < mFavoredDiscoveredPrefix))
        {
            LogInfo("Found a favored on-link prefix %s", mFavoredDiscoveredPrefix.ToString().AsCString());
            Deprecate();
        }
    }

exit:
    return;
}

bool RoutingManagerOffload::OnLinkPrefixManager::IsInitalEvaluationDone(void) const
{
    // This method indicates whether or not we are done with the
    // initial policy evaluation of the on-link prefixes, i.e., either
    // we have discovered a favored on-link prefix (being advertised by
    // another router on infra link) or we are advertising our local
    // on-link prefix.

    return (mFavoredDiscoveredPrefix.GetLength() != 0 || IsPublishingOrAdvertising());
}

void RoutingManagerOffload::OnLinkPrefixManager::HandleDiscoveredPrefixTableChanged(void)
{
    // This is a callback from `mDiscoveredPrefixTable` indicating that
    // there has been a change in the table. If the favored on-link
    // prefix has changed, we trigger a re-evaluation of the routing
    // policy.

    Ip6::Prefix newFavoredPrefix;

    Get<RoutingManagerOffload>().mDiscoveredPrefixTable.FindFavoredOnLinkPrefix(newFavoredPrefix);

    if (newFavoredPrefix != mFavoredDiscoveredPrefix)
    {
        Get<RoutingManagerOffload>().ScheduleRoutingPolicyEvaluation(kAfterRandomDelay);
    }
}

void RoutingManagerOffload::OnLinkPrefixManager::PublishAndAdvertise(void)
{
    // Start publishing and advertising the local on-link prefix if
    // not already.

    switch (GetState())
    {
    case kIdle:
    case kDeprecating:
        break;

    case kPublishing:
    case kAdvertising:
        ExitNow();
    }

    SetState(kPublishing);
    ResetExpireTime(TimerMilli::GetNow());

    // We wait for the ULA `fc00::/7` route or a sub-prefix of it (e.g.,
    // default route) to be added in Network Data before
    // starting to advertise the local on-link prefix in RAs.
    // However, if it is already present in Network Data (e.g.,
    // added by another BR on the same Thread mesh), we can
    // immediately start advertising it.

    if (Get<RoutingManagerOffload>().NetworkDataContainsUlaRoute())
    {
        SetState(kAdvertising);
    }

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::Deprecate(void)
{
    // Deprecate the local on-link prefix if it was being advertised
    // before. While depreciating the prefix, we wait for the lifetime
    // timer to expire before unpublishing the prefix from the Network
    // Data. We also continue to include it as a PIO in the RA message
    // with zero preferred lifetime and the remaining valid lifetime
    // until the timer expires.

    switch (GetState())
    {
    case kPublishing:
    case kAdvertising:
        SetState(kDeprecating);
        break;

    case kIdle:
    case kDeprecating:
        break;
    }
}

bool RoutingManagerOffload::OnLinkPrefixManager::ShouldPublishUlaRoute(void) const
{
    // Determine whether or not we should publish ULA prefix. We need
    // to publish if we are in any of `kPublishing`, `kAdvertising`,
    // or `kDeprecating` states, or if there is at least one old local
    // prefix being deprecated.

    return (GetState() != kIdle) || !mOldLocalPrefixes.IsEmpty();
}

void RoutingManagerOffload::OnLinkPrefixManager::ResetExpireTime(TimeMilli aNow)
{
    mExpireTime = aNow + TimeMilli::SecToMsec(kDefaultOnLinkPrefixLifetime);
    mTimer.FireAtIfEarlier(mExpireTime);
    SavePrefix(mLocalPrefix, mExpireTime);
}

bool RoutingManagerOffload::OnLinkPrefixManager::IsPublishingOrAdvertising(void) const
{
    return (GetState() == kPublishing) || (GetState() == kAdvertising);
}

void RoutingManagerOffload::OnLinkPrefixManager::AppendAsPiosTo(Ip6::Nd::RouterAdvertMessage &aRaMessage)
{
    AppendCurPrefix(aRaMessage);
    AppendOldPrefixes(aRaMessage);
}

void RoutingManagerOffload::OnLinkPrefixManager::AppendCurPrefix(Ip6::Nd::RouterAdvertMessage &aRaMessage)
{
    // Append the local on-link prefix to the `aRaMessage` as a PIO
    // only if it is being advertised or deprecated.
    //
    // If in `kAdvertising` state, we reset the expire time.
    // If in `kDeprecating` state, we include it as PIO with zero
    // preferred lifetime and the remaining valid lifetime.

    uint32_t  validLifetime     = kDefaultOnLinkPrefixLifetime;
    uint32_t  preferredLifetime = kDefaultOnLinkPrefixLifetime;
    TimeMilli now               = TimerMilli::GetNow();

    switch (GetState())
    {
    case kAdvertising:
        ResetExpireTime(now);
        break;

    case kDeprecating:
        VerifyOrExit(mExpireTime > now);
        validLifetime     = TimeMilli::MsecToSec(mExpireTime - now);
        preferredLifetime = 0;
        break;

    case kIdle:
    case kPublishing:
        ExitNow();
    }

    SuccessOrAssert(aRaMessage.AppendPrefixInfoOption(mLocalPrefix, validLifetime, preferredLifetime));

    LogPrefixInfoOption(mLocalPrefix, validLifetime, preferredLifetime);

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::AppendOldPrefixes(Ip6::Nd::RouterAdvertMessage &aRaMessage)
{
    TimeMilli now = TimerMilli::GetNow();
    uint32_t  validLifetime;

    for (const OldPrefix &oldPrefix : mOldLocalPrefixes)
    {
        if (oldPrefix.mExpireTime < now)
        {
            continue;
        }

        validLifetime = TimeMilli::MsecToSec(oldPrefix.mExpireTime - now);
        SuccessOrAssert(aRaMessage.AppendPrefixInfoOption(oldPrefix.mPrefix, validLifetime, 0));

        LogPrefixInfoOption(oldPrefix.mPrefix, validLifetime, 0);
    }
}

void RoutingManagerOffload::OnLinkPrefixManager::HandleNetDataChange(void)
{
    VerifyOrExit(GetState() == kPublishing);

    if (Get<RoutingManagerOffload>().NetworkDataContainsUlaRoute())
    {
        SetState(kAdvertising);
        Get<RoutingManagerOffload>().ScheduleRoutingPolicyEvaluation(kAfterRandomDelay);
    }

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::HandleExtPanIdChange(void)
{
    // If the current local prefix is being advertised or deprecated,
    // we save it in `mOldLocalPrefixes` and keep deprecating it. It will
    // be included in emitted RAs as PIO with zero preferred lifetime.
    // It will still be present in Network Data until its expire time
    // so to allow Thread nodes to continue to communicate with `InfraIf`
    // device using addresses based on this prefix.

    uint16_t    oldState  = GetState();
    Ip6::Prefix oldPrefix = mLocalPrefix;

    GenerateLocalPrefix();

    VerifyOrExit(oldPrefix != mLocalPrefix);

    switch (oldState)
    {
    case kIdle:
    case kPublishing:
        break;

    case kAdvertising:
    case kDeprecating:
        DeprecateOldPrefix(oldPrefix, mExpireTime);
        break;
    }

    if (Get<RoutingManagerOffload>().mIsRunning)
    {
        Get<RoutingManagerOffload>().mRoutePublisher.Evaluate();
        Get<RoutingManagerOffload>().ScheduleRoutingPolicyEvaluation(kAfterRandomDelay);
    }

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::DeprecateOldPrefix(const Ip6::Prefix &aPrefix, TimeMilli aExpireTime)
{
    OldPrefix  *entry = nullptr;
    Ip6::Prefix removedPrefix;

    removedPrefix.Clear();

    VerifyOrExit(!mOldLocalPrefixes.ContainsMatching(aPrefix));

    LogInfo("Deprecating old on-link prefix %s", aPrefix.ToString().AsCString());

    if (!mOldLocalPrefixes.IsFull())
    {
        entry = mOldLocalPrefixes.PushBack();
    }
    else
    {
        // If there is no more room in `mOldLocalPrefixes` array
        // we evict the entry with the earliest expiration time.

        entry = &mOldLocalPrefixes[0];

        for (OldPrefix &oldPrefix : mOldLocalPrefixes)
        {
            if ((oldPrefix.mExpireTime < entry->mExpireTime))
            {
                entry = &oldPrefix;
            }
        }

        removedPrefix = entry->mPrefix;

        IgnoreError(Get<Settings>().RemoveBrOnLinkPrefix(removedPrefix));
    }

    entry->mPrefix     = aPrefix;
    entry->mExpireTime = aExpireTime;
    mTimer.FireAtIfEarlier(aExpireTime);

    SavePrefix(aPrefix, aExpireTime);

exit:
    return;
}

void RoutingManagerOffload::OnLinkPrefixManager::SavePrefix(const Ip6::Prefix &aPrefix, TimeMilli aExpireTime)
{
    Settings::BrOnLinkPrefix savedPrefix;

    savedPrefix.SetPrefix(aPrefix);
    savedPrefix.SetLifetime(TimeMilli::MsecToSec(aExpireTime - TimerMilli::GetNow()));
    IgnoreError(Get<Settings>().AddOrUpdateBrOnLinkPrefix(savedPrefix));
}

void RoutingManagerOffload::OnLinkPrefixManager::HandleTimer(void)
{
    TimeMilli                           now            = TimerMilli::GetNow();
    TimeMilli                           nextExpireTime = now.GetDistantFuture();
    Array<Ip6::Prefix, kMaxOldPrefixes> expiredPrefixes;

    switch (GetState())
    {
    case kIdle:
        break;
    case kPublishing:
    case kAdvertising:
    case kDeprecating:
        if (now >= mExpireTime)
        {
            IgnoreError(Get<Settings>().RemoveBrOnLinkPrefix(mLocalPrefix));
            SetState(kIdle);
        }
        else
        {
            nextExpireTime = mExpireTime;
        }
        break;
    }

    for (OldPrefix &entry : mOldLocalPrefixes)
    {
        if (now >= entry.mExpireTime)
        {
            SuccessOrAssert(expiredPrefixes.PushBack(entry.mPrefix));
        }
        else
        {
            nextExpireTime = Min(nextExpireTime, entry.mExpireTime);
        }
    }

    for (const Ip6::Prefix &prefix : expiredPrefixes)
    {
        LogInfo("Old local on-link prefix %s expired", prefix.ToString().AsCString());
        IgnoreError(Get<Settings>().RemoveBrOnLinkPrefix(prefix));
        mOldLocalPrefixes.RemoveMatching(prefix);
    }

    if (nextExpireTime != now.GetDistantFuture())
    {
        mTimer.FireAtIfEarlier(nextExpireTime);
    }

    Get<RoutingManagerOffload>().mRoutePublisher.Evaluate();
}

const char *RoutingManagerOffload::OnLinkPrefixManager::StateToString(State aState)
{
    static const char *const kStateStrings[] = {
        "Removed",     // (0) kIdle
        "Publishing",  // (1) kPublishing
        "Advertising", // (2) kAdvertising
        "Deprecating", // (3) kDeprecating
    };

    static_assert(0 == kIdle, "kIdle value is incorrect");
    static_assert(1 == kPublishing, "kPublishing value is incorrect");
    static_assert(2 == kAdvertising, "kAdvertising value is incorrect");
    static_assert(3 == kDeprecating, "kDeprecating value is incorrect");

    return kStateStrings[aState];
}

//---------------------------------------------------------------------------------------------------------------------
// OnMeshPrefixArray

void RoutingManagerOffload::OnMeshPrefixArray::Add(const OnMeshPrefix &aPrefix)
{
    // Checks if `aPrefix` is already present in the array and if not
    // adds it as new entry.

    Error error;

    VerifyOrExit(!Contains(aPrefix));

    error = PushBack(aPrefix);

    if (error != kErrorNone)
    {
        LogWarn("Too many on-mesh prefixes in net data, ignoring prefix %s", aPrefix.ToString().AsCString());
    }

exit:
    return;
}

void RoutingManagerOffload::OnMeshPrefixArray::MarkAsDeleted(const OnMeshPrefix &aPrefix)
{
    // Searches for a matching entry to `aPrefix` and if found marks
    // it as deleted by setting prefix length to zero.

    OnMeshPrefix *entry = Find(aPrefix);

    if (entry != nullptr)
    {
        entry->SetLength(0);
    }
}

//---------------------------------------------------------------------------------------------------------------------
// RoutePublisher

const otIp6Prefix RoutingManagerOffload::RoutePublisher::kUlaPrefix = {
    {{{0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}},
    7,
};

RoutingManagerOffload::RoutePublisher::RoutePublisher(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mState(kDoNotPublish)
    , mPreference(NetworkData::kRoutePreferenceMedium)
    , mUserSetPreference(false)
    , mAdvPioFlag(false)
    , mTimer(aInstance)
{
}

void RoutingManagerOffload::RoutePublisher::Evaluate(void)
{
    State newState = kDoNotPublish;

    VerifyOrExit(Get<RoutingManagerOffload>().IsRunning());

    if (Get<RoutingManagerOffload>().mOmrPrefixManager.GetFavoredPrefix().IsInfrastructureDerived() &&
        Get<RoutingManagerOffload>().mDiscoveredPrefixTable.ContainsDefaultOrNonUlaRoutePrefix())
    {
        newState = kPublishDefault;
    }
    else if (Get<RoutingManagerOffload>().mDiscoveredPrefixTable.ContainsNonUlaOnLinkPrefix())
    {
        newState = kPublishDefault;
    }
    else if (Get<RoutingManagerOffload>().mDiscoveredPrefixTable.ContainsUlaOnLinkPrefix() ||
             Get<RoutingManagerOffload>().mOnLinkPrefixManager.ShouldPublishUlaRoute())
    {
        newState = kPublishUla;
    }

exit:
    if (newState != mState)
    {
        LogInfo("RoutePublisher state: %s -> %s", StateToString(mState), StateToString(newState));
        UpdatePublishedRoute(newState);
        Get<RoutingManagerOffload>().mOmrPrefixManager.UpdateDefaultRouteFlag(newState == kPublishDefault);
    }
}

void RoutingManagerOffload::RoutePublisher::DeterminePrefixFor(State aState, Ip6::Prefix &aPrefix) const
{
    aPrefix.Clear();

    switch (aState)
    {
    case kDoNotPublish:
    case kPublishDefault:
        // `Clear()` will set the prefix `::/0`.
        break;
    case kPublishUla:
        aPrefix = GetUlaPrefix();
        break;
    }
}

void RoutingManagerOffload::RoutePublisher::UpdatePublishedRoute(State aNewState)
{
    // Updates the published route entry in Network Data, transitioning
    // from current `mState` to new `aNewState`. This method can be used
    // when there is no change to `mState` but a change to `mPreference`
    // or `mAdvPioFlag`.

    Ip6::Prefix                      oldPrefix;
    NetworkData::ExternalRouteConfig routeConfig;

    DeterminePrefixFor(mState, oldPrefix);

    if (aNewState == kDoNotPublish)
    {
        VerifyOrExit(mState != kDoNotPublish);
        IgnoreError(otPlatCpNetDataUnpublishPrefix(&oldPrefix));
        ExitNow();
    }

    routeConfig.Clear();
    routeConfig.mPreference = mPreference;
    routeConfig.mAdvPio     = mAdvPioFlag;
    routeConfig.mStable     = true;
    DeterminePrefixFor(aNewState, routeConfig.GetPrefix());

    // If we were not publishing a route prefix before, publish the new
    // `routeConfig`. Otherwise, use `ReplacePublishedExternalRoute()` to
    // replace the previously published prefix entry. This ensures that we do
    // not have a situation where the previous route is removed while the new
    // one is not yet added in the Network Data.

    if (mState == kDoNotPublish)
    {
        SuccessOrAssert(otPlatCpNetDataPublishExternalRoute(&routeConfig));
    }
    else
    {
        SuccessOrAssert(otPlatCpNetDataReplacePublishedExternalRoute(&oldPrefix, &routeConfig));
    }

exit:
    mState = aNewState;
}

void RoutingManagerOffload::RoutePublisher::Unpublish(void)
{
    // Unpublish the previously published route based on `mState`
    // and update `mState`.

    Ip6::Prefix prefix;

    VerifyOrExit(mState != kDoNotPublish);
    DeterminePrefixFor(mState, prefix);
    IgnoreError(otPlatCpNetDataUnpublishPrefix(&prefix));
    mState = kDoNotPublish;

exit:
    return;
}

void RoutingManagerOffload::RoutePublisher::UpdateAdvPioFlags(bool aAdvPioFlag)
{
    VerifyOrExit(mAdvPioFlag != aAdvPioFlag);
    mAdvPioFlag = aAdvPioFlag;
    UpdatePublishedRoute(mState);

exit:
    return;
}

void RoutingManagerOffload::RoutePublisher::SetPreference(RoutePreference aPreference)
{
    LogInfo("User explicitly set published route preference to %s", RoutePreferenceToString(aPreference));
    mUserSetPreference = true;
    mTimer.Stop();
    UpdatePreference(aPreference);
}

void RoutingManagerOffload::RoutePublisher::ClearPreference(void)
{
    VerifyOrExit(mUserSetPreference);

    LogInfo("User cleared explicitly set published route preference - set based on role");
    mUserSetPreference = false;
    SetPreferenceBasedOnRole();

exit:
    return;
}

void RoutingManagerOffload::RoutePublisher::SetPreferenceBasedOnRole(void)
{
    RoutePreference preference = NetworkData::kRoutePreferenceMedium;
    otDeviceRole role = otPlatCpGetDeviceRoleCached(nullptr);

    if (role == OT_DEVICE_ROLE_CHILD) // two way link quality
    {
        preference = NetworkData::kRoutePreferenceLow;
    }

    UpdatePreference(preference);
    mTimer.Stop();
}

void RoutingManagerOffload::RoutePublisher::HandleNotifierEvents(Events aEvents)
{
    VerifyOrExit(!mUserSetPreference);

    if (aEvents.Contains(kEventThreadRoleChanged))
    {
        SetPreferenceBasedOnRole();
    }

    if (aEvents.Contains(kEventParentLinkQualityChanged))
    {
        otDeviceRole role = otPlatCpGetDeviceRoleCached(nullptr);
        VerifyOrExit(role == OT_DEVICE_ROLE_CHILD);

        // TODO: Link Quality Change
        //if (Get<Mle::Mle>().GetParent().GetTwoWayLinkQuality() == kLinkQuality3)
        //{
            //VerifyOrExit(!mTimer.IsRunning());
            //mTimer.Start(kDelayBeforePrfUpdateOnLinkQuality3);
        //}
        //else
        //{
            //UpdatePreference(NetworkData::kRoutePreferenceLow);
            //mTimer.Stop();
        //}
    }

exit:
    return;
}

void RoutingManagerOffload::RoutePublisher::HandleTimer(void) { SetPreferenceBasedOnRole(); }

void RoutingManagerOffload::RoutePublisher::UpdatePreference(RoutePreference aPreference)
{
    VerifyOrExit(mPreference != aPreference);

    LogInfo("Published route preference changed: %s -> %s", RoutePreferenceToString(mPreference),
            RoutePreferenceToString(aPreference));
    mPreference = aPreference;
    UpdatePublishedRoute(mState);

exit:
    return;
}

const char *RoutingManagerOffload::RoutePublisher::StateToString(State aState)
{
    static const char *const kStateStrings[] = {
        "none",      // (0) kDoNotPublish
        "def-route", // (1) kPublishDefault
        "ula",       // (2) kPublishUla
    };

    static_assert(0 == kDoNotPublish, "kDoNotPublish value is incorrect");
    static_assert(1 == kPublishDefault, "kPublishDefault value is incorrect");
    static_assert(2 == kPublishUla, "kPublishUla value is incorrect");

    return kStateStrings[aState];
}

//---------------------------------------------------------------------------------------------------------------------
// RsSender

RoutingManagerOffload::RsSender::RsSender(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mTxCount(0)
    , mTimer(aInstance)
{
}

void RoutingManagerOffload::RsSender::Start(void)
{
    uint32_t delay;

    VerifyOrExit(!IsInProgress());

    delay = Random::NonCrypto::GetUint32InRange(0, kMaxStartDelay);

    LogInfo("RsSender: Starting - will send first RS in %lu msec", ToUlong(delay));

    mTxCount   = 0;
    mStartTime = TimerMilli::GetNow();
    mTimer.Start(delay);

exit:
    return;
}

void RoutingManagerOffload::RsSender::Stop(void) { mTimer.Stop(); }

Error RoutingManagerOffload::RsSender::SendRs(void)
{
    Ip6::Address                  destAddress;
    Ip6::Nd::RouterSolicitMessage routerSolicit;
    InfraIf::Icmp6Packet          packet;
    Error                         error;

    LogInfo("RsSender SenderRs");
    packet.InitFrom(routerSolicit);
    destAddress.SetToLinkLocalAllRoutersMulticast();

    error = Get<RoutingManagerOffload>().mInfraIf.Send(packet, destAddress);

    // TODO: Ip6 Counters
    return error;
}

void RoutingManagerOffload::RsSender::HandleTimer(void)
{
    Error    error;
    uint32_t delay;

    if (mTxCount >= kMaxTxCount)
    {
        LogInfo("RsSender: Finished sending RS msgs and waiting for RAs");
        Get<RoutingManagerOffload>().HandleRsSenderFinished(mStartTime);
        ExitNow();
    }

    error = SendRs();

    if (error == kErrorNone)
    {
        mTxCount++;
        delay = (mTxCount == kMaxTxCount) ? kWaitOnLastAttempt : kTxInterval;
        LogInfo("RsSender: Sent RS %u/%u", mTxCount, kMaxTxCount);
    }
    else
    {
        LogCrit("RsSender: Failed to send RS %u/%u: %s", mTxCount + 1, kMaxTxCount, ErrorToString(error));

        // Note that `mTxCount` is intentionally not incremented
        // if the tx fails.
        delay = kRetryDelay;
    }

    mTimer.Start(delay);

exit:
    return;
}

} // namespace BorderRouter

} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
