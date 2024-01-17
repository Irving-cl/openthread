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

#include "offload.hpp"

#include <openthread/platform/offload.h>

#include "common/as_core_type.hpp"
#include "common/locator_getters.hpp"
#include "common/log.hpp"
#include "common/notifier.hpp"
#include "instance/instance.hpp"

namespace ot {

RegisterLogModule("Offload");

Offload::Offload(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mIsActiveScanInProgress(false)
    , mActiveScanHandler(nullptr)
    , mScanHandlerContext(nullptr)
    , mUpdateUnicastAddressesTask(aInstance)
{
    otPlatCpEnable(&GetInstance());
}

Error Offload::ActiveScan(uint32_t aScanChannels, uint16_t aScanDuration, ActiveScanHandler aHandler, void * aContext)
{
    Error error = kErrorNone;

    // VerifyOrExit(!mIsActiveScanInProgress, error = kErrorBusy);
    VerifyOrExit(aHandler != nullptr && aContext != nullptr, error = kErrorInvalidArgs);

    otLogInfoPlat("ActiveScan");
    mActiveScanHandler = aHandler;
    mScanHandlerContext = aContext;
    error = otPlatCpIssueActiveScan(&GetInstance(), aScanChannels, aScanDuration);

exit:
    return error;
}

void Offload::ActiveScanDone(otActiveScanResult *aScanResult)
{
    mActiveScanHandler(aScanResult, mScanHandlerContext);
}

Mle::DeviceRole Offload::GetDeviceRole(void)
{
    return MapEnum(otPlatCpGetDeviceRole(&GetInstance()));
}

Error Offload::DatasetInitNew(otOperationalDataset &aDataset)
{
    return otPlatCpDatasetInitNew(&GetInstance(), &aDataset);
}

Error Offload::ThreadStartStop(bool aStart)
{
    return otPlatCpThreadStartStop(&GetInstance(), aStart);
}

void Offload::DataUpdateIPv6AddressTable(Ip6::Netif::UnicastAddress *aAddressList, uint8_t aCount)
{
    OT_ASSERT(aAddressList != nullptr);

    // Clear Pending Addresses
    while (!mUnicastAddressesPending.IsEmpty())
    {
        Ip6::Netif::UnicastAddress *address = mUnicastAddressesPending.Pop();
        mUnicastAddressPool.Free(*address);
    }

    for (uint8_t i = 0; i < aCount; i++)
    {
        Ip6::Netif::UnicastAddress *address = mUnicastAddressPool.Allocate();
        *address = aAddressList[i];
        mUnicastAddressesPending.Add(*address);
    }

    mUpdateUnicastAddressesTask.Post();
}

void Offload::UpdateUnicastAddresses(void)
{
    for (const Ip6::Netif::UnicastAddress &address : mUnicastAddresses)
    {
        if (!mUnicastAddressesPending.ContainsMatching(address.GetAddress()))
        {
            otIp6AddressInfo info;

            info.mAddress      = &address.mAddress;
            info.mPrefixLength = address.mPrefixLength;
            info.mScope        = address.GetScope();
            info.mPreferred    = address.mPreferred;

            LogInfo("Remove address %s, prefixLen:%u, scope:%u, preferred:%u", address.GetAddress().ToString().AsCString(),
                    info.mPrefixLength, info.mScope, info.mPreferred);
            otPlatNetifProcessAddressChange(&info, false, &GetInstance());
        }
    }

    for (const Ip6::Netif::UnicastAddress &address : mUnicastAddressesPending)
    {
        if (!mUnicastAddresses.ContainsMatching(address.GetAddress()))
        {
            otIp6AddressInfo info;

            info.mAddress      = &address.mAddress;
            info.mPrefixLength = address.mPrefixLength;
            info.mScope        = address.GetScope();
            info.mPreferred    = address.mPreferred;

            LogInfo("Add address %s, prefixLen:%u, scope:%u, preferred:%u", address.GetAddress().ToString().AsCString(),
                    info.mPrefixLength, info.mScope, info.mPreferred);
            otPlatNetifProcessAddressChange(&info, true, &GetInstance());
        }
    }

    LinkedList<Ip6::Netif::UnicastAddress> tmp = mUnicastAddresses;
    mUnicastAddresses = mUnicastAddressesPending;
    mUnicastAddressesPending = tmp;
}

} // namespace ot

