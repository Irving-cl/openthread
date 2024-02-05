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

#ifndef OFFLOAD_HPP_
#define OFFLOAD_HPP_

#include "openthread-core-config.h"

#include <openthread/offload.h>

#include "common/callback.hpp"
#include "common/error.hpp"
#include "common/locator.hpp"
#include "common/linked_list.hpp"
#include "common/non_copyable.hpp"
#include "common/pool.hpp"
#include "net/netif.hpp"
//#include "net/srp_server.hpp"
#include "thread/mle_types.hpp"

namespace ot {

typedef otHandleActiveScanResult ActiveScanHandler;

constexpr uint16_t kMaxSrpServerHostFullNameLength = 256;
constexpr uint16_t kMaxSrpServerHostServiceInstanceNameLength = 128;
constexpr uint16_t kMaxSrpServerHostServiceTxtDataLength = 256;
constexpr uint8_t  kMaxSrpServerHostAddrNum        = 10;
constexpr uint8_t  kSrpServerServiceMaxSubTypeNum  = 10;
constexpr uint8_t  kSrpServerServiceMaxSubTypeNameLength = 128;

struct OffloadSrpServerHost
{
    otSrpServerServiceUpdateId mUpdateId;
    char     mFullName[kMaxSrpServerHostFullNameLength];
    uint32_t mLease;
    uint8_t  mAddressNum;
    otIp6Address mAddresses[kMaxSrpServerHostAddrNum];
};

struct OffloadSrpServerService
{
    char     mInstanceName[kMaxSrpServerHostServiceInstanceNameLength];
    bool     mIsDeleted;
    uint16_t mPort;
    uint8_t  mTxtData[kMaxSrpServerHostServiceTxtDataLength];
    uint16_t mTxtDataLen;
    uint8_t  mSubTypeNum;
    char     mSubTypeNames[kSrpServerServiceMaxSubTypeNum][kSrpServerServiceMaxSubTypeNameLength];
};

class Offload : public InstanceLocator, private NonCopyable
{
public:

    explicit Offload(Instance &aInstance);

    Error ActiveScan(uint32_t aScanChannels, uint16_t aScanDuration, ActiveScanHandler aHandler, void *aContext);

    void ActiveScanDone(otActiveScanResult *aScanResult);

    Mle::DeviceRole GetDeviceRole(void);

    Error DatasetInitNew(otOperationalDataset &aDataset);

    Error ThreadStartStop(bool aStart);

    void DataUpdateIPv6AddressTable(Ip6::Netif::UnicastAddress *aAddressList, uint8_t aCount);

    void UpdateSrpServerHost(OffloadSrpServerHost *aHost, OffloadSrpServerService *aServices, uint8_t aSrvCount);

    void SetServiceUpdateHandler(otSrpServerServiceUpdateHandler aServiceHandler, void *aContext) {
        mServiceUpdateHandler.Set(aServiceHandler, aContext);
    }

private:

    static constexpr uint32_t kDefaultEventsHandlerTimeout = OPENTHREAD_CONFIG_SRP_SERVER_SERVICE_UPDATE_TIMEOUT;

    void UpdateUnicastAddresses(void);

    using UpdateUnicastAddressesTask = TaskletIn<Offload, &Offload::UpdateUnicastAddresses>;

    bool mIsActiveScanInProgress;
    ActiveScanHandler mActiveScanHandler;
    void *mScanHandlerContext;

    UpdateUnicastAddressesTask             mUpdateUnicastAddressesTask;
    LinkedList<Ip6::Netif::UnicastAddress> mUnicastAddresses;
    LinkedList<Ip6::Netif::UnicastAddress> mUnicastAddressesPending;

    Pool<Ip6::Netif::UnicastAddress, 20>   mUnicastAddressPool;

    Callback<otSrpServerServiceUpdateHandler> mServiceUpdateHandler;
};

Offload &GetOffload(otInstance *aInstance);

} // namespace ot

#endif // OFFLOAD_HPP_
