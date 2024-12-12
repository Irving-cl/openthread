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

#if OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE

#include "common/error.hpp"
#include "common/locator.hpp"
#include "common/non_copyable.hpp"
#include "common/notifier.hpp"

namespace ot {
namespace MeshCoP {

class BorderAgentPublisher : public InstanceLocator, private NonCopyable
{
    friend class ot::Notifier;

public:
    explicit BorderAgentPublisher(Instance &aInstance);

    Error SetMeshCopServiceValues(const char    *aServiceInstanceName,
                                  const char    *aProductName,
                                  const char    *aVendorName,
                                  const uint8_t *aVendorOui);
    Error SetMeshCopSerivceVendorTxtData(const uint8_t *aVendorTxtData, uint16_t aLen);

private:
    static constexpr size_t kMaxServiceInstanceNameLength = 256;
    static constexpr size_t kMaxProductNameLength         = 24;
    static constexpr size_t kMaxVendorNameLength          = 24;
    static constexpr size_t kVendorOuiLength              = 3;
    static constexpr size_t kVendorTxtData                = 256;

    void HandleNotifierEvents(Events aEvents);
    void UpdateMeshCopService(void);
    void PublishMeshCopService(void);

    void Clear(void);
    bool IsMeshCopValuesSet(void);

    char    mBaseServiceInstanceName[kMaxServiceInstanceNameLength];
    char    mProductName[kMaxProductNameLength];
    char    mVendorName[kMaxVendorNameLength];
    uint8_t mVendorOui[kVendorOuiLength];
    uint8_t mVendorTxtData[kVendorTxtData];
};

} // namespace MeshCoP
} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE

#endif // BORDER_AGENT_PUBLISHER_HPP_
