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

#ifndef OT_POSIX_OFFLOAD_HPP_
#define OT_POSIX_OFFLOAD_HPP_

#include <openthread/link.h>
#include <openthread/thread.h>

#include "offload_tasklet.hpp"
#include "common/callback.hpp"
#include "common/linked_list.hpp"
#include "common/pool.hpp"
#include "instance/instance_def.hpp"
#include "lib/spinel/ncp_spinel.hpp"
#include "net/netif.hpp"

namespace ot {
namespace Posix {

typedef void (*Ip6ReceiveCallback)(const uint8_t *aData, uint16_t aLen, void *aContext);

class Offload : public otInstance
{
public:
    static Offload &InitSingle(void);

    otError ActiveScan(uint32_t                 aScanChannels,
                       uint16_t                 aScanDuration,
                       otHandleActiveScanResult aHandler,
                       void                    *aContext);

    otDeviceRole GetDeviceRole(void);

    otError DatasetCreateNewNetwork(otOperationalDataset &aDataset);

    otError Ip6SetEnabled(bool aEnabled);

    bool    Ip6IsEnabled(void) { return !mUnicastAddressTable.IsEmpty(); }
    otError ThreadSetEnabled(bool aEnabled);

    otError Ip6Send(const uint8_t *aPacket, size_t aLen);

    void Ip6SetAddressCallback(otIp6AddressCallback aCallback, void *aContext);

    void Ip6SetReceiveCallback(Ip6ReceiveCallback aCallback, void *aContext);

    void NetifSetStateChangedCallback(otStateChangedCallback aCallback, void *aContext);

    void TaskletsProcess(void);

    Tasklet::Scheduler &GetTaskletScheduler(void) { return mTaskletScheduler; }

private:
    static constexpr uint8_t kMaxAddressPoolSize = 20;

    Offload(void);
    void AfterInit(void);

    /**
     * Returns a reference to the OpenThread offload instance.
     *
     * @returns A reference to the OpenThread offload instance.
     *
     */
    static Offload &Get(void);

    static void ActiveScanDone(otInstance *aInstance, otActiveScanResult *aScanResult);
    void        ActiveScanDone(otActiveScanResult *aScanResult);
    static void UpdateIp6AddressTable(otInstance *aInstance, otNetifAddress *aAddressTable, uint8_t aSize);
    void        UpdateIp6AddressTable(otNetifAddress *aAddressTable, uint8_t aSize);
    static void ReceiveIp6(otInstance *aInstance, const uint8_t *aData, uint16_t aLength);
    void        ReceiveIp6(const uint8_t *aData, uint16_t aLength);

    void UpdateUnicastAddressTable(void);
    using UpdateUnicastAddressTableTask = TaskletIn<&Offload::UpdateUnicastAddressTable>;

    Tasklet::Scheduler mTaskletScheduler;
    Spinel::NcpSpinel  mNcpSpinel;

    bool                                   mIsInitialized;
    ot::Callback<otHandleActiveScanResult> mActiveScanHandler;
    bool                                   mIsActiveScanInProgress;
    ot::Callback<otIp6AddressCallback>     mAddressCallback;
    ot::Callback<Ip6ReceiveCallback>       mIp6ReceiveCallback;
    ot::Callback<otStateChangedCallback>   mNetifStateChangedCallback;

    UpdateUnicastAddressTableTask          mUpdateUnicastAddressTableTask;
    LinkedList<Ip6::Netif::UnicastAddress> mUnicastAddressTable;
    LinkedList<Ip6::Netif::UnicastAddress> mUnicastAddressTablePending;

    Pool<Ip6::Netif::UnicastAddress, kMaxAddressPoolSize> mUnicastAddressPool;
};

Offload *offloadInstanceInit(void);

} // namespace Posix
} // namespace ot

#endif // OT_POSIX_OFFLOAD_HPP_
