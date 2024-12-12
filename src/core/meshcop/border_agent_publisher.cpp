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

#if OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE

#include "common/error.hpp"
#include "common/string.hpp"
#include "instance/instance.hpp"

namespace ot {
namespace MeshCoP {

RegisterLogModule("BA_Publisher");

BorderAgentPublisher::BorderAgentPublisher(Instance &aInstance)
    : InstanceLocator(aInstance)
{
    Clear();
}

Error BorderAgentPublisher::SetMeshCopServiceValues(const char    *aServiceInstanceName,
                                                    const char    *aProductName,
                                                    const char    *aVendorName,
                                                    const uint8_t *aVendorOui)
{
    Error    error = kErrorNone;
    uint8_t serviceInstanceNameLen;
    uint8_t productNameLen;
    uint8_t vendorNameLen;

    serviceInstanceNameLen = StringLength(aServiceInstanceName, kMaxServiceInstanceNameLength);
    VerifyOrExit(serviceInstanceNameLen < kMaxServiceInstanceNameLength, error = kErrorInvalidArgs);
    VerifyOrExit(IsValidUtf8String(aServiceInstanceName, error = kErrorInvalidArgs));

    productNameLen = StringLength(aProductName, kMaxProductNameLength + 1);
    VerifyOrExit(productNameLen <= kMaxProductNameLength, error = kErrorInvalidArgs);
    VerifyOrExit(IsValidUtf8String(aProductName, error = kErrorInvalidArgs));
    
    vendorNameLen = StringLength(aVendorName, kMaxVendorNameLength + 1);
    VerifyOrExit(vendorNameLen <= kMaxVendorNameLength, error = kErrorInvalidArgs);
    VerifyOrExit(IsValidUtf8String(aVendorName, error = kErrorInvalidArgs));

    memcpy(mBaseServiceInstanceName, aServiceInstanceName, serviceInstanceNameLen);
    memcpy(mProductName, aProductName, productNameLen);
    memcpy(mVendorName, aVendorName, vendorNameLen);
    memcpy(mVendorOui, aVendorOui, kVendorOuiLength);

    UpdateMeshCopService();

exit:
    return error;
}

Error BorderAgentPublisher::SetMeshCopSerivceVendorTxtData(const uint8_t *aVendorTxtData, uint16_t aLen)
{
    Error error = kErrorNone;
    
    VerifyOrExit(aLen <= sizeof(mVendorTxtData), error = kErrorInvalidArgs);
    
    memcpy(mVendorTxtData, aVendorTxtData, aLen);

exit:
    return error;
}

void BorderAgentPublisher::HandleNotifierEvents(Events aEvents)
{
    if (aEvents.ContainsAny(kEventThreadRoleChanged | kEventThreadExtPanIdChanged | kEventThreadNetworkNameChanged |
                            kEventThreadBackboneRouterStateChanged | kEventThreadNetdataChanged))
    {
        UpdateMeshCopService();
    }
}

void BorderAgentPublisher::UpdateMeshCopService(void)
{
    if (IsMeshCopValuesSet())
    {
        PublishMeshCopService();
    }
}

void BorderAgentPublisher::PublishMeshCopService(void) {}

void BorderAgentPublisher::Clear(void)
{
    memset(mBaseServiceInstanceName, 0, sizeof(mBaseServiceInstanceName));
    memset(mProductName, 0, sizeof(mProductName));
    memset(mVendorName, 0, sizeof(mVendorName));
    memset(mVendorOui, 0, sizeof(mVendorOui));
}

bool BorderAgentPublisher::IsMeshCopValuesSet(void)
{
    return mBaseServiceInstanceName[0] != '\0';
}

} // namespace MeshCoP
} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_AGENT_ENABLE
