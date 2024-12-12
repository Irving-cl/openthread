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
 *   This file includes definitions for the Border Agent Service Publisher.
 */

#ifndef BORDER_AGENT_PUBLISHER_HPP_
#define BORDER_AGENT_PUBLISHER_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE && OPENTHREAD_CONFIG_PLATFORM_DNSSD_ENABLE

#include <openthread/mdns.h>

#include "common/error.hpp"
#include "common/heap_data.hpp"
#include "common/locator.hpp"
#include "common/message.hpp"
#include "common/non_copyable.hpp"
#include "common/notifier.hpp"
#include "common/string.hpp"
#include "meshcop/border_agent.hpp"
#include "net/dns_types.hpp"
#include "net/dnssd.hpp"

namespace ot {
namespace MeshCoP {

class BorderAgentPublisher : public InstanceLocator, private NonCopyable
{
    friend class ot::Dnssd;
    friend class ot::Notifier;
    friend class ot::MeshCoP::BorderAgent;

public:
    explicit BorderAgentPublisher(Instance &aInstance);

    /**
     * This method enables/disables the Border Agent Publisher.
     *
     * @param[in] aEnabled  Whether to enable the Border Agent Publisher.
     */
    void SetEnabled(bool aEnabled);

    Error SetMeshCopServiceValues(const char                        *aBaseServiceInstanceName,
                                  const char                        *aProductName,
                                  const otBorderAgentVendorTxtEntry *aVendorTxtEntries,
                                  uint8_t                            aLength);

private:
    static constexpr size_t kServiceInstanceNameSuffixLength = 6; // Example: " #1234"
    static constexpr size_t kMaxServiceInstanceNameLength    = 256;
    static constexpr size_t kMaxBaseServiceInstanceNameLength =
        kMaxServiceInstanceNameLength - kServiceInstanceNameSuffixLength;
    static constexpr size_t kMaxProductNameLength   = 24;
    static constexpr size_t kMaxVendorNameLength    = 24;
    static constexpr size_t kVendorOuiLength        = 3;
    static constexpr size_t kMaxVendorTxtDataLength = 256;

    enum : uint8_t
    {
        kConnectionModeDisabled = 0,
        kConnectionModePskc     = 1,
        kConnectionModePskd     = 2,
        kConnectionModeVendor   = 3,
        kConnectionModeX509     = 4,
    };

    enum : uint8_t
    {
        kThreadIfStatusNotInitialized = 0,
        kThreadIfStatusInitialized    = 1,
        kThreadIfStatusActive         = 2,
    };

    enum : uint8_t
    {
        kThreadRoleDisabledOrDetached = 0,
        kThreadRoleChild              = 1,
        kThreadRoleRouter             = 2,
        kThreadRoleLeader             = 3,
    };

    enum : uint8_t
    {
        kAvailabilityInfrequent = 0,
        kAvailabilityHigh       = 1,
    };

    struct StateBitmap
    {
        uint32_t mConnectionMode : 3;
        uint32_t mThreadIfStatus : 2;
        uint32_t mAvailability : 2;
        uint32_t mBbrIsActive : 1;
        uint32_t mBbrIsPrimary : 1;
        uint32_t mThreadRole : 2;
        uint32_t mEpskcSupported : 1;

        StateBitmap(void)
            : mConnectionMode(0)
            , mThreadIfStatus(0)
            , mAvailability(0)
            , mBbrIsActive(0)
            , mBbrIsPrimary(0)
            , mThreadRole(kThreadRoleDisabledOrDetached)
            , mEpskcSupported(0)
        {
        }

        uint32_t ToUint32(void) const
        {
            uint32_t bitmap = 0;

            bitmap |= mConnectionMode << 0;
            bitmap |= mThreadIfStatus << 3;
            bitmap |= mAvailability << 5;
            bitmap |= mBbrIsActive << 7;
            bitmap |= mBbrIsPrimary << 8;
            bitmap |= mThreadRole << 9;
            bitmap |= mEpskcSupported << 11;
            return bitmap;
        }
    };

    void InitializeDefaultName(void);

    void WriteServiceInstanceNameWithExtAddr(StringWriter &aStringWriter);
    void GenerateAlternativeServiceInstanceName(void);

    bool IsThreadStarted(void);
    void HandleNotifierEvents(Events aEvents);
    void UpdateMeshCopService(void);
    void PublishMeshCopService(void);
    void UnpublishMeshCopService(void);

    void HandleEpskcStateChanged(void);
    void PublishEpskcService(void);
    void UnpublishEpskcService(void);

    void HandleDnssdPlatformStateChange(void);

    StateBitmap GetStateBitmap(void);

    void AppendActiveTimestampTxtEntry(Dns::TxtEntry &txtEntry, Message &aMessage);
#if OPENTHREAD_CONFIG_BACKBONE_ROUTER_ENABLE
    void AppendBbrTxtEntry(StateBitmap aState, Dns::TxtEntry &txtEntry, Message &aMessage);
#endif
#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
    void AppendOmrTxtEntry(Dns::TxtEntry &txtEntry, Message &aMessage);
#endif

    static void PublishMeshCopServiceCallback(otInstance *aInstance, otPlatDnssdRequestId aRequestId, otError aError);
    void        PublishMeshCopServiceCallback(otPlatDnssdRequestId aRequestId, otError aError);
    static void UnpublishMeshCopServiceCallback(otInstance *aInstance, otPlatDnssdRequestId aRequestId, otError aError);
    void        UnpublishMeshCopServiceCallback(otPlatDnssdRequestId aRequestId, otError aError);
    static void PublishEpskcServiceCallback(otInstance *aInstance, otPlatDnssdRequestId aRequestId, otError aError);
    void        PublishEpskcServiceCallback(otPlatDnssdRequestId aRequestId, otError aError);
    static void UnpublishEpskcServiceCallback(otInstance *aInstance, otPlatDnssdRequestId aRequestId, otError aError);
    void        UnpublishEpskcServiceCallback(otPlatDnssdRequestId aRequestId, otError aError);

    bool mEnabled;
    char mBaseServiceInstanceName[kMaxBaseServiceInstanceNameLength];
    char mServiceInstanceName[kMaxServiceInstanceNameLength];
    char mProductName[kMaxProductNameLength];

    Heap::Data mVendorTxtData;
    Heap::Data mTxtData;

    otPlatDnssdRequestId mRequestId;
};

} // namespace MeshCoP
} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE && OPENTHREAD_CONFIG_PLATFORM_DNSSD_ENABLE

#endif // BORDER_AGENT_PUBLISHER_HPP_
