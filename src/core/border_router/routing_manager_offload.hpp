/*
 *  Copyright (c) 2023, The OpenThread Authors.
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
 *   This file includes definitions for the RA-based routing management.
 *
 */

#ifndef ROUTING_MANAGER_OFFLOAD_HPP_
#define ROUTING_MANAGER_OFFLOAD_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#if !OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE
#error "OPENTHREAD_CONFIG_BORDER_ROUTER_ENABLE is required for OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE."
#endif

#if !OPENTHREAD_CONFIG_IP6_SLAAC_ENABLE
#error "OPENTHREAD_CONFIG_IP6_SLAAC_ENABLE is required for OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE."
#endif

#include <openthread/nat64.h>
#include <openthread/netdata.h>

#include "border_router/infra_if.hpp"
#include "common/array.hpp"
#include "common/error.hpp"
#include "common/heap_allocatable.hpp"
#include "common/linked_list.hpp"
#include "common/locator.hpp"
#include "common/message.hpp"
#include "common/notifier.hpp"
#include "common/pool.hpp"
#include "common/string.hpp"
#include "common/timer.hpp"
#include "net/ip6.hpp"
#include "net/nat64_translator.hpp"
#include "net/nd6.hpp"
#include "thread/network_data.hpp"

namespace ot {

namespace BorderRouter {

/**
 * Implements bi-directional routing between Thread and Infrastructure networks.
 *
 * The Border Routing manager works on both Thread interface and infrastructure interface.
 * All ICMPv6 messages are sent/received on the infrastructure interface.
 *
 */
class RoutingManagerOffload : public InstanceLocator
{
    friend class ot::Notifier;
    friend class ot::Instance;
public:

    typedef NetworkData::RoutePreference       RoutePreference;     ///< Route preference (high, medium, low).
    typedef otBorderRoutingPrefixTableIterator PrefixTableIterator; ///< Prefix Table Iterator.
    typedef otBorderRoutingPrefixTableEntry    PrefixTableEntry;    ///< Prefix Table Entry.

    static constexpr uint16_t kMaxPublishedPrefixes = 3;

    enum State : uint8_t
    {
        kStateUninitialized = OT_BORDER_ROUTING_STATE_UNINITIALIZED, ///< Uninitialized.
        kStateDisabled      = OT_BORDER_ROUTING_STATE_DISABLED,      ///< Initialized but disabled.
        kStateStopped       = OT_BORDER_ROUTING_STATE_STOPPED,       ///< Initialized & enabled, but currently stopped.
        kStateRunning       = OT_BORDER_ROUTING_STATE_RUNNING,       ///< Initialized, enabled, and running.
    };

    explicit RoutingManagerOffload(Instance &aInstance);
    Error Init(uint32_t aInfraIfIndex, bool aInfraIfIsRunning);
    Error SetEnabled(bool aEnabled);
    bool IsRunning(void) const { return mIsRunning; }
    State GetState(void) const;
    void RequestStop(void) { Stop(); }
    RoutePreference GetRouteInfoOptionPreference(void) const { return mRioPreference; }
    void SetRouteInfoOptionPreference(RoutePreference aPreference);
    void ClearRouteInfoOptionPreference(void);
    RoutePreference GetRoutePreference(void) const { return mRoutePublisher.GetPreference(); }
    void SetRoutePreference(RoutePreference aPreference) { mRoutePublisher.SetPreference(aPreference); }
    void ClearRoutePreference(void) { mRoutePublisher.ClearPreference(); }

    Error GetOmrPrefix(Ip6::Prefix &aPrefix) const;
    Error GetFavoredOmrPrefix(Ip6::Prefix &aPrefix, RoutePreference &aPreference) const;
    Error GetOnLinkPrefix(Ip6::Prefix &aPrefix) const;
    Error GetFavoredOnLinkPrefix(Ip6::Prefix &aPrefix) const;

    void HandleReceived(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress);
    void HandleInfraIfStateChanged(void) { EvaluateState(); }
    static bool IsValidOmrPrefix(const NetworkData::OnMeshPrefixConfig &aOnMeshPrefixConfig);
    static bool IsValidOmrPrefix(const Ip6::Prefix &aPrefix);

private:
    static constexpr uint8_t kMaxOnMeshPrefixes = OPENTHREAD_CONFIG_BORDER_ROUTING_MAX_ON_MESH_PREFIXES;

    static constexpr uint8_t kOmrPrefixLength    = OT_IP6_PREFIX_BITSIZE; // The length of an OMR prefix. In bits.
    static constexpr uint8_t kOnLinkPrefixLength = OT_IP6_PREFIX_BITSIZE; // The length of an On-link prefix. In bits.
    static constexpr uint8_t kBrUlaPrefixLength  = 48;                    // The length of a BR ULA prefix. In bits.

    static constexpr uint16_t kOmrPrefixSubnetId   = 1; // The subnet ID of an OMR prefix within a BR ULA prefix.
    static constexpr uint16_t kNat64PrefixSubnetId = 2; // The subnet ID of a NAT64 prefix within a BR ULA prefix.

    // The maximum number of initial Router Advertisements.
    static constexpr uint32_t kMaxInitRtrAdvertisements = 3;

    static constexpr uint32_t kDefaultOmrPrefixLifetime    = 1800; // The default OMR prefix valid lifetime. In sec.
    static constexpr uint32_t kDefaultOnLinkPrefixLifetime = 1800; // The default on-link prefix valid lifetime. In sec.
    static constexpr uint32_t kDefaultNat64PrefixLifetime  = 300;  // The default NAT64 prefix valid lifetime. In sec.
    static constexpr uint32_t kMaxRtrAdvInterval           = 600;  // Max Router Advertisement Interval. In sec.
    static constexpr uint32_t kMinRtrAdvInterval           = kMaxRtrAdvInterval / 3; // Min RA Interval. In sec.
    static constexpr uint32_t kMaxInitRtrAdvInterval       = 16;                     // Max Initial RA Interval. In sec.
    static constexpr uint32_t kRaReplyJitter               = 500;  // Jitter for sending RA after rx RS. In msec.
    static constexpr uint32_t kPolicyEvaluationMinDelay    = 2000; // Min delay for policy evaluation. In msec.
    static constexpr uint32_t kPolicyEvaluationMaxDelay    = 4000; // Max delay for policy evaluation. In msec.
    static constexpr uint32_t kMinDelayBetweenRtrAdvs      = 3000; // Min delay (msec) between consecutive RAs.
    static constexpr uint32_t kRtrAdvStaleTime = 1800;

    static_assert(kMinRtrAdvInterval <= 3 * kMaxRtrAdvInterval / 4, "invalid RA intervals");
    static_assert(kDefaultOmrPrefixLifetime >= kMaxRtrAdvInterval, "invalid default OMR prefix lifetime");
    static_assert(kDefaultOnLinkPrefixLifetime >= kMaxRtrAdvInterval, "invalid default on-link prefix lifetime");
    static_assert(kRtrAdvStaleTime >= 1800 && kRtrAdvStaleTime <= kDefaultOnLinkPrefixLifetime,
                  "invalid RA STALE time");
    static_assert(kPolicyEvaluationMaxDelay > kPolicyEvaluationMinDelay,
                  "kPolicyEvaluationMaxDelay must be larger than kPolicyEvaluationMinDelay");

    enum RouterAdvTxMode : uint8_t // Used in `SendRouterAdvertisement()`
    {
        kInvalidateAllPrevPrefixes,
        kAdvPrefixesFromNetData,
    };

    enum ScheduleMode : uint8_t // Used in `ScheduleRoutingPolicyEvaluation()`
    {
        kImmediately,
        kForNextRa,
        kAfterRandomDelay,
        kToReplyToRs,
    };

    void HandleDiscoveredPrefixTableChanged(void); // Declare early so we can use in `mSignalTask`
    void HandleDiscoveredPrefixTableEntryTimer(void) { mDiscoveredPrefixTable.HandleEntryTimer(); }
    void HandleDiscoveredPrefixTableRouterTimer(void) { mDiscoveredPrefixTable.HandleRouterTimer(); }

    class DiscoveredPrefixTable : public InstanceLocator
    {
        // This class maintains the discovered on-link and route prefixes
        // from the received RA messages by processing PIO and RIO options
        // from the message. It takes care of processing the RA message but
        // delegates the decision whether to include or exclude a prefix to
        // `RoutingManager` by calling its `ShouldProcessPrefixInfoOption()`
        // and `ShouldProcessRouteInfoOption()` methods.
        //
        // It manages the lifetime of the discovered entries and publishes
        // and unpublishes the prefixes in the Network Data (as external
        // route) as they are added or removed.
        //
        // When there is any change in the table (an entry is added, removed,
        // or modified), it signals the change to `RoutingManager` by calling
        // `HandleDiscoveredPrefixTableChanged()` callback. A `Tasklet` is
        // used for signalling which ensures that if there are multiple
        // changes within the same flow of execution, the callback is
        // invoked after all the changes are processed.

#if OPENTHREAD_CONFIG_BORDER_ROUTING_DHCP6_PD_ENABLE
        friend class PdPrefixManager; // For DiscoveredPrefixTable::Entry
#endif

    public:
        explicit DiscoveredPrefixTable(Instance &aInstance);

        void ProcessRouterAdvertMessage(const Ip6::Nd::RouterAdvertMessage &aRaMessage,
                                        const Ip6::Address                 &aSrcAddress);
        void ProcessNeighborAdvertMessage(const Ip6::Nd::NeighborAdvertMessage &aNaMessage);

        bool ContainsDefaultOrNonUlaRoutePrefix(void) const;
        bool ContainsNonUlaOnLinkPrefix(void) const;
        bool ContainsUlaOnLinkPrefix(void) const;

        void FindFavoredOnLinkPrefix(Ip6::Prefix &aPrefix) const;

        void RemoveOnLinkPrefix(const Ip6::Prefix &aPrefix);
        void RemoveRoutePrefix(const Ip6::Prefix &aPrefix);

        void RemoveAllEntries(void);
        void RemoveOrDeprecateOldEntries(TimeMilli aTimeThreshold);

        TimeMilli CalculateNextStaleTime(TimeMilli aNow) const;

        void  InitIterator(PrefixTableIterator &aIterator) const;
        Error GetNextEntry(PrefixTableIterator &aIterator, PrefixTableEntry &aEntry) const;

        void HandleEntryTimer(void);
        void HandleRouterTimer(void);

    private:
#if !OPENTHREAD_CONFIG_BORDER_ROUTING_USE_HEAP_ENABLE
        static constexpr uint16_t kMaxRouters = OPENTHREAD_CONFIG_BORDER_ROUTING_MAX_DISCOVERED_ROUTERS;
        static constexpr uint16_t kMaxEntries = OPENTHREAD_CONFIG_BORDER_ROUTING_MAX_DISCOVERED_PREFIXES;
#endif

        class Entry : public LinkedListEntry<Entry>,
                      public Unequatable<Entry>,
#if OPENTHREAD_CONFIG_BORDER_ROUTING_USE_HEAP_ENABLE
                      public Heap::Allocatable<Entry>,
#endif
                      private Clearable<Entry>
        {
            friend class LinkedListEntry<Entry>;
            friend class Clearable<Entry>;
#if OPENTHREAD_CONFIG_BORDER_ROUTING_DHCP6_PD_ENABLE
            friend class PdPrefixManager;
#endif

        public:
            enum Type : uint8_t
            {
                kTypeOnLink,
                kTypeRoute,
            };

            struct Matcher
            {
                Matcher(const Ip6::Prefix &aPrefix, Type aType)
                    : mPrefix(aPrefix)
                    , mType(aType)
                {
                }

                const Ip6::Prefix &mPrefix;
                Type               mType;
            };

            struct Checker
            {
                enum Mode : uint8_t
                {
                    kIsUla,
                    kIsNotUla,
                };

                Checker(Mode aMode, Type aType)
                    : mMode(aMode)
                    , mType(aType)

                {
                }

                Mode mMode;
                Type mType;
            };

            struct ExpirationChecker
            {
                explicit ExpirationChecker(TimeMilli aNow)
                    : mNow(aNow)
                {
                }

                TimeMilli mNow;
            };

            void               SetFrom(const Ip6::Nd::RouterAdvertMessage::Header &aRaHeader);
            void               SetFrom(const Ip6::Nd::PrefixInfoOption &aPio);
            void               SetFrom(const Ip6::Nd::RouteInfoOption &aRio);
            Type               GetType(void) const { return mType; }
            bool               IsOnLinkPrefix(void) const { return (mType == kTypeOnLink); }
            bool               IsRoutePrefix(void) const { return (mType == kTypeRoute); }
            const Ip6::Prefix &GetPrefix(void) const { return mPrefix; }
            const TimeMilli   &GetLastUpdateTime(void) const { return mLastUpdateTime; }
            uint32_t           GetValidLifetime(void) const { return mValidLifetime; }
            void               ClearValidLifetime(void) { mValidLifetime = 0; }
            TimeMilli          GetExpireTime(void) const;
            TimeMilli          GetStaleTime(void) const;
            RoutePreference    GetPreference(void) const;
            bool               operator==(const Entry &aOther) const;
            bool               Matches(const Matcher &aMatcher) const;
            bool               Matches(const Checker &aChecker) const;
            bool               Matches(const ExpirationChecker &aChecker) const;

            // Methods to use when `IsOnLinkPrefix()`
            uint32_t GetPreferredLifetime(void) const { return mShared.mPreferredLifetime; }
            void     ClearPreferredLifetime(void) { mShared.mPreferredLifetime = 0; }
            bool     IsDeprecated(void) const;
            void     AdoptValidAndPreferredLifetimesFrom(const Entry &aEntry);

            // Method to use when `!IsOnlinkPrefix()`
            RoutePreference GetRoutePreference(void) const { return mShared.mRoutePreference; }

        private:
            static uint32_t CalculateExpireDelay(uint32_t aValidLifetime);

            Entry      *mNext;
            Ip6::Prefix mPrefix;
            Type        mType;
            TimeMilli   mLastUpdateTime;
            uint32_t    mValidLifetime;
            union
            {
                uint32_t        mPreferredLifetime; // Applicable when prefix is on-link.
                RoutePreference mRoutePreference;   // Applicable when prefix is not on-link
            } mShared;
        };

        struct Router : public LinkedListEntry<Router>,
#if OPENTHREAD_CONFIG_BORDER_ROUTING_USE_HEAP_ENABLE
                        public Heap::Allocatable<Router>,
#endif
                        public Clearable<Router>
        {
            // The timeout (in msec) for router staying in active state
            // before starting the Neighbor Solicitation (NS) probes.
            static constexpr uint32_t kActiveTimeout = OPENTHREAD_CONFIG_BORDER_ROUTING_ROUTER_ACTIVE_CHECK_TIMEOUT;

            static constexpr uint8_t  kMaxNsProbes          = 5;    // Max number of NS probe attempts.
            static constexpr uint32_t kNsProbeRetryInterval = 1000; // In msec. Time between NS probe attempts.
            static constexpr uint32_t kNsProbeTimeout       = 2000; // In msec. Max Wait time after last NS probe.
            static constexpr uint32_t kJitter               = 2000; // In msec. Jitter to randomize probe starts.

            static_assert(kMaxNsProbes < 255, "kMaxNsProbes MUST not be 255");

            enum EmptyChecker : uint8_t
            {
                kContainsNoEntries
            };

            bool Matches(const Ip6::Address &aAddress) const { return aAddress == mAddress; }
            bool Matches(EmptyChecker) const { return mEntries.IsEmpty(); }

            Router           *mNext;
            Ip6::Address      mAddress;
            LinkedList<Entry> mEntries;
            TimeMilli         mTimeout;
            uint8_t           mNsProbeCount;
            bool              mManagedAddressConfigFlag : 1;
            bool              mOtherConfigFlag : 1;
            bool              mStubRouterFlag : 1;
        };

        class Iterator : public PrefixTableIterator
        {
        public:
            const Router *GetRouter(void) const { return static_cast<const Router *>(mPtr1); }
            void          SetRouter(const Router *aRouter) { mPtr1 = aRouter; }
            const Entry  *GetEntry(void) const { return static_cast<const Entry *>(mPtr2); }
            void          SetEntry(const Entry *aEntry) { mPtr2 = aEntry; }
            TimeMilli     GetInitTime(void) const { return TimeMilli(mData32); }
            void          SetInitTime(void) { mData32 = TimerMilli::GetNow().GetValue(); }
        };

        void         ProcessRaHeader(const Ip6::Nd::RouterAdvertMessage::Header &aRaHeader, Router &aRouter);
        void         ProcessPrefixInfoOption(const Ip6::Nd::PrefixInfoOption &aPio, Router &aRouter);
        void         ProcessRouteInfoOption(const Ip6::Nd::RouteInfoOption &aRio, Router &aRouter);
        void         ProcessRaFlagsExtOption(const Ip6::Nd::RaFlagsExtOption &aFlagsOption, Router &aRouter);
        bool         Contains(const Entry::Checker &aChecker) const;
        void         RemovePrefix(const Entry::Matcher &aMatcher);
        void         RemoveOrDeprecateEntriesFromInactiveRouters(void);
        void         RemoveRoutersWithNoEntries(void);
        void         FreeRouters(LinkedList<Router> &aRouters);
        void         FreeEntries(LinkedList<Entry> &aEntries);
        void         UpdateNetworkDataOnChangeTo(Entry &aEntry);
        const Entry *FindFavoredEntryToPublish(const Ip6::Prefix &aPrefix) const;
        void         RemoveExpiredEntries(void);
        void         SignalTableChanged(void);
        void         UpdateRouterOnRx(Router &aRouter);
        void         SendNeighborSolicitToRouter(const Router &aRouter);
#if OPENTHREAD_CONFIG_BORDER_ROUTING_USE_HEAP_ENABLE
        Router *AllocateRouter(void) { return Router::Allocate(); }
        Entry  *AllocateEntry(void) { return Entry::Allocate(); }
        void    FreeRouter(Router &aRouter) { aRouter.Free(); }
        void    FreeEntry(Entry &aEntry) { aEntry.Free(); }
#else
        Router *AllocateRouter(void) { return mRouterPool.Allocate(); }
        Entry  *AllocateEntry(void) { return mEntryPool.Allocate(); }
        void    FreeRouter(Router &aRouter) { mRouterPool.Free(aRouter); }
        void    FreeEntry(Entry &aEntry) { mEntryPool.Free(aEntry); }
#endif

        using SignalTask  = TaskletIn<RoutingManagerOffload, &RoutingManagerOffload::HandleDiscoveredPrefixTableChanged>;
        using EntryTimer  = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::HandleDiscoveredPrefixTableEntryTimer>;
        using RouterTimer = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::HandleDiscoveredPrefixTableRouterTimer>;

        LinkedList<Router> mRouters;
        EntryTimer         mEntryTimer;
        RouterTimer        mRouterTimer;
        SignalTask         mSignalTask;
#if !OPENTHREAD_CONFIG_BORDER_ROUTING_USE_HEAP_ENABLE
        Pool<Entry, kMaxEntries>  mEntryPool;
        Pool<Router, kMaxRouters> mRouterPool;
#endif
    };

    class OmrPrefixManager;

    class OmrPrefix : public Clearable<OmrPrefix>
    {
        friend class OmrPrefixManager;

    public:
        OmrPrefix(void) { Clear(); }

        bool               IsEmpty(void) const { return (mPrefix.GetLength() == 0); }
        const Ip6::Prefix &GetPrefix(void) const { return mPrefix; }
        RoutePreference    GetPreference(void) const { return mPreference; }
        bool               IsDomainPrefix(void) const { return mIsDomainPrefix; }

    protected:
        Ip6::Prefix     mPrefix;
        RoutePreference mPreference;
        bool            mIsDomainPrefix;
    };

    class FavoredOmrPrefix : public OmrPrefix
    {
        friend class OmrPrefixManager;

    public:
        bool IsInfrastructureDerived(void) const;

    private:
        void SetFrom(const NetworkData::OnMeshPrefixConfig &aOnMeshPrefixConfig);
        void SetFrom(const OmrPrefix &aOmrPrefix);
        bool IsFavoredOver(const NetworkData::OnMeshPrefixConfig &aOmrPrefixConfig) const;
    };

    class OmrPrefixManager : public InstanceLocator
    {
    public:
        explicit OmrPrefixManager(Instance &aInstance);

        void                    Init(const Ip6::Prefix &aBrUlaPrefix);
        void                    Start(void);
        void                    Stop(void);
        void                    Evaluate(void);
        void                    UpdateDefaultRouteFlag(bool aDefaultRoute);
        bool                    IsLocalAddedInNetData(void) const { return mIsLocalAddedInNetData; }
        const Ip6::Prefix      &GetGeneratedPrefix(void) const { return mGeneratedPrefix; }
        const OmrPrefix        &GetLocalPrefix(void) const { return mLocalPrefix; }
        const FavoredOmrPrefix &GetFavoredPrefix(void) const { return mFavoredPrefix; }

    private:
        static constexpr uint16_t kInfoStringSize = 85;

        typedef String<kInfoStringSize> InfoString;

        void       DetermineFavoredPrefix(void);
        Error      AddLocalToNetData(void);
        Error      AddOrUpdateLocalInNetData(void);
        void       RemoveLocalFromNetData(void);
        InfoString LocalToString(void) const;

        OmrPrefix        mLocalPrefix;
        Ip6::Prefix      mGeneratedPrefix;
        FavoredOmrPrefix mFavoredPrefix;
        bool             mIsLocalAddedInNetData;
        bool             mDefaultRoute;
    };

    void HandleOnLinkPrefixManagerTimer(void) { mOnLinkPrefixManager.HandleTimer(); }

    class OnLinkPrefixManager : public InstanceLocator
    {
    public:
        explicit OnLinkPrefixManager(Instance &aInstance);

        // Max number of old on-link prefixes to retain to deprecate.
        static constexpr uint16_t kMaxOldPrefixes = OPENTHREAD_CONFIG_BORDER_ROUTING_MAX_OLD_ON_LINK_PREFIXES;

        void               Init(void);
        void               Start(void);
        void               Stop(void);
        void               Evaluate(void);
        const Ip6::Prefix &GetLocalPrefix(void) const { return mLocalPrefix; }
        const Ip6::Prefix &GetFavoredDiscoveredPrefix(void) const { return mFavoredDiscoveredPrefix; }
        bool               IsInitalEvaluationDone(void) const;
        void               HandleDiscoveredPrefixTableChanged(void);
        bool               ShouldPublishUlaRoute(void) const;
        void               AppendAsPiosTo(Ip6::Nd::RouterAdvertMessage &aRaMessage);
        bool               IsPublishingOrAdvertising(void) const;
        void               HandleNetDataChange(void);
        void               HandleExtPanIdChange(void);
        void               HandleTimer(void);

    private:
        enum State : uint8_t // State of `mLocalPrefix`
        {
            kIdle,
            kPublishing,
            kAdvertising,
            kDeprecating,
        };

        struct OldPrefix
        {
            bool Matches(const Ip6::Prefix &aPrefix) const { return mPrefix == aPrefix; }

            Ip6::Prefix mPrefix;
            TimeMilli   mExpireTime;
        };

        State GetState(void) const { return mState; }
        void  SetState(State aState);
        void  GenerateLocalPrefix(void);
        void  PublishAndAdvertise(void);
        void  Deprecate(void);
        void  ResetExpireTime(TimeMilli aNow);
        void  AppendCurPrefix(Ip6::Nd::RouterAdvertMessage &aRaMessage);
        void  AppendOldPrefixes(Ip6::Nd::RouterAdvertMessage &aRaMessage);
        void  DeprecateOldPrefix(const Ip6::Prefix &aPrefix, TimeMilli aExpireTime);
        void  SavePrefix(const Ip6::Prefix &aPrefix, TimeMilli aExpireTime);

        static const char *StateToString(State aState);

        using ExpireTimer = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::HandleOnLinkPrefixManagerTimer>;

        Ip6::Prefix                       mLocalPrefix;
        State                             mState;
        TimeMilli                         mExpireTime;
        Ip6::Prefix                       mFavoredDiscoveredPrefix;
        Array<OldPrefix, kMaxOldPrefixes> mOldLocalPrefixes;
        ExpireTimer                       mTimer;
    };

    typedef Ip6::Prefix OnMeshPrefix;

    class OnMeshPrefixArray : public Array<OnMeshPrefix, kMaxOnMeshPrefixes>
    {
    public:
        void Add(const OnMeshPrefix &aPrefix);
        void MarkAsDeleted(const OnMeshPrefix &aPrefix);
    };

    void HandleRoutePublisherTimer(void) { mRoutePublisher.HandleTimer(); }

    class RoutePublisher : public InstanceLocator // Manages the routes that are published in net data
    {
    public:
        explicit RoutePublisher(Instance &aInstance);

        void Start(void) { Evaluate(); }
        void Stop(void) { Unpublish(); }
        void Evaluate(void);

        void UpdateAdvPioFlags(bool aAdvPioFlag);

        RoutePreference GetPreference(void) const { return mPreference; }
        void            SetPreference(RoutePreference aPreference);
        void            ClearPreference(void);

        void HandleNotifierEvents(Events aEvents);
        void HandleTimer(void);

        static const Ip6::Prefix &GetUlaPrefix(void) { return AsCoreType(&kUlaPrefix); }

    private:
        static constexpr uint32_t kDelayBeforePrfUpdateOnLinkQuality3 = TimeMilli::SecToMsec(5 * 60);

        static const otIp6Prefix kUlaPrefix;

        enum State : uint8_t
        {
            kDoNotPublish,   // Do not publish any routes in network data.
            kPublishDefault, // Publish "::/0" route in network data.
            kPublishUla,     // Publish "fc00::/7" route in network data.
        };

        void DeterminePrefixFor(State aState, Ip6::Prefix &aPrefix) const;
        void UpdatePublishedRoute(State aNewState);
        void Unpublish(void);
        void SetPreferenceBasedOnRole(void);
        void UpdatePreference(RoutePreference aPreference);

        static const char *StateToString(State aState);

        using DelayTimer = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::HandleRoutePublisherTimer>;

        State           mState;
        RoutePreference mPreference;
        bool            mUserSetPreference;
        bool            mAdvPioFlag;
        DelayTimer      mTimer;
    };

    struct RaInfo
    {
        // Tracks info about emitted RA messages: Number of RAs sent,
        // last tx time, header to use and whether the header is
        // discovered from receiving RAs from the host itself. This
        // ensures that if an entity on host is advertising certain
        // info in its RA header (e.g., a default route), the RAs we
        // emit from `RoutingManager` also include the same header.

        RaInfo(void)
            : mHeaderUpdateTime(TimerMilli::GetNow())
            , mIsHeaderFromHost(false)
            , mTxCount(0)
            , mLastTxTime(TimerMilli::GetNow() - kMinDelayBetweenRtrAdvs)
        {
        }

        Ip6::Nd::RouterAdvertMessage::Header mHeader;
        TimeMilli                            mHeaderUpdateTime;
        bool                                 mIsHeaderFromHost;
        uint32_t                             mTxCount;
        TimeMilli                            mLastTxTime;
    };

    void HandleRsSenderTimer(void) { mRsSender.HandleTimer(); }

    class RsSender : public InstanceLocator
    {
    public:
        // This class implements tx of Router Solicitation (RS)
        // messages to discover other routers. `Start()` schedules
        // a cycle of RS transmissions of `kMaxTxCount` separated
        // by `kTxInterval`. At the end of cycle the callback
        // `HandleRsSenderFinished()` is invoked to inform end of
        // the cycle to `RoutingManager`.

        explicit RsSender(Instance &aInstance);

        bool IsInProgress(void) const { return mTimer.IsRunning(); }
        void Start(void);
        void Stop(void);
        void HandleTimer(void);

    private:
        // All time intervals are in msec.
        static constexpr uint32_t kMaxStartDelay     = 1000;        // Max random delay to send the first RS.
        static constexpr uint32_t kTxInterval        = 4000;        // Interval between RS tx.
        static constexpr uint32_t kRetryDelay        = kTxInterval; // Interval to wait to retry a failed RS tx.
        static constexpr uint32_t kWaitOnLastAttempt = 1000;        // Wait interval after last RS tx.
        static constexpr uint8_t  kMaxTxCount        = 3;           // Number of RS tx in one cycle.

        Error SendRs(void);

        using RsTimer = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::HandleRsSenderTimer>;

        uint8_t   mTxCount;
        RsTimer   mTimer;
        TimeMilli mStartTime;
    };

    void  EvaluateState(void);
    void  Start(void);
    void  Stop(void);
    void  HandleNotifierEvents(Events aEvents);
    bool  IsInitialized(void) const { return mInfraIf.IsInitialized(); }
    bool  IsEnabled(void) const { return mIsEnabled; }
    void  SetRioPreferenceBasedOnRole(void);
    void  UpdateRioPreference(RoutePreference aPreference);
    Error LoadOrGenerateRandomBrUlaPrefix(void);

    void EvaluateRoutingPolicy(void);
    bool IsInitalPolicyEvaluationDone(void) const;
    void ScheduleRoutingPolicyEvaluation(ScheduleMode aMode);
    void HandleRsSenderFinished(TimeMilli aStartTime);
    void SendRouterAdvertisement(RouterAdvTxMode aRaTxMode);

    void HandleDiscoveredPrefixStaleTimer(void);

    void HandleRouterAdvertisement(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress);
    void HandleRouterSolicit(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress);
    void HandleNeighborAdvertisement(const InfraIf::Icmp6Packet &aPacket);
    bool ShouldProcessPrefixInfoOption(const Ip6::Nd::PrefixInfoOption &aPio, const Ip6::Prefix &aPrefix);
    bool ShouldProcessRouteInfoOption(const Ip6::Nd::RouteInfoOption &aRio, const Ip6::Prefix &aPrefix);
    void UpdateDiscoveredPrefixTableOnNetDataChange(void);
    bool NetworkDataContainsOmrPrefix(const Ip6::Prefix &aPrefix) const;
    bool NetworkDataContainsUlaRoute(void) const;
    void UpdateRouterAdvertHeader(const Ip6::Nd::RouterAdvertMessage *aRouterAdvertMessage);
    bool IsReceivedRouterAdvertFromManager(const Ip6::Nd::RouterAdvertMessage &aRaMessage) const;
    void ResetDiscoveredPrefixStaleTimer(void);

    static bool IsValidBrUlaPrefix(const Ip6::Prefix &aBrUlaPrefix);
    static bool IsValidOnLinkPrefix(const Ip6::Nd::PrefixInfoOption &aPio);
    static bool IsValidOnLinkPrefix(const Ip6::Prefix &aOnLinkPrefix);

    static void LogPrefixInfoOption(const Ip6::Prefix &aPrefix, uint32_t aValidLifetime, uint32_t aPreferredLifetime);
    static void LogRouteInfoOption(const Ip6::Prefix &aPrefix, uint32_t aLifetime, RoutePreference aPreference);

    using RoutingPolicyTimer         = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::EvaluateRoutingPolicy>;
    using DiscoveredPrefixStaleTimer = TimerMilliIn<RoutingManagerOffload, &RoutingManagerOffload::HandleDiscoveredPrefixStaleTimer>;

    bool mIsRunning;
    bool mIsEnabled;

    InfraIf mInfraIf;

    Ip6::Prefix mBrUlaPrefix;

    OmrPrefixManager mOmrPrefixManager;
    OnMeshPrefixArray mAdvertisedPrefixes;

    RoutePreference mRioPreference;
    bool            mUserSetRioPreference;

    OnLinkPrefixManager mOnLinkPrefixManager;

    DiscoveredPrefixTable mDiscoveredPrefixTable;

    RoutePublisher mRoutePublisher;

    RaInfo   mRaInfo;
    RsSender mRsSender;

    DiscoveredPrefixStaleTimer mDiscoveredPrefixStaleTimer;
    RoutingPolicyTimer         mRoutingPolicyTimer;
};


} // namespace BorderRouter

DefineMapEnum(otBorderRoutingState, BorderRouter::RoutingManagerOffload::State);

} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
#endif // ROUTING_MANAGER_OFFLOAD_HPP_
