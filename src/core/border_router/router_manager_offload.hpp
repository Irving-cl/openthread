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
public:

    static constexpr uint16_t kMaxPublishedPrefixes = 3;

    explicit RoutingManager(Instance &aInstance);
    Error Init(uint32_t aInfraIfIndex, bool aInfraIfIsRunning);
    Error SetEnabled(bool aEnabled);
    bool IsRunning(void) const { return mIsRunning; }
    State GetState(void) const;
    void RequestStop(void) { Stop(); }
    RoutePreference GetRouteInfoOptionPreference(void) const { return mRioPreference; }
    void SetRouteInfoOptionPreference(RoutePreference aPreference);
    void ClearRouteInfoOptionPreference(void);

    Error GetOmrPrefix(Ip6::Prefix &aPrefix) const;
    Error GetFavoredOmrPrefix(Ip6::Prefix &aPrefix, RoutePreference &aPreference) const;
    Error GetOnLinkPrefix(Ip6::Prefix &aPrefix) const;
    Error GetFavoredOnLinkPrefix(Ip6::Prefix &aPrefix) const;

    void HandleReceived(const InfraIf::Icmp6Packet &aPacket, const Ip6::Address &aSrcAddress);
    // void HandleInfraIfStateChanged(void) { EvaluateState(); }

private:

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

        using ExpireTimer = TimerMilliIn<RoutingManager, &RoutingManager::HandleOnLinkPrefixManagerTimer>;

        Ip6::Prefix                       mLocalPrefix;
        State                             mState;
        TimeMilli                         mExpireTime;
        Ip6::Prefix                       mFavoredDiscoveredPrefix;
        Array<OldPrefix, kMaxOldPrefixes> mOldLocalPrefixes;
        ExpireTimer                       mTimer;
    };

    bool mIsRunning;
    bool mIsEnabled;

    InfraIf mInfraIf;

    OmrPrefixManager mOmrPrefixManager;
    OnLinkPrefixManager mOnLinkPrefixManager;
};


} // namespace BorderRouter

} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
#endif // ROUTING_MANAGER_OFFLOAD_HPP_