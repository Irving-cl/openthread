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

#ifndef NCP_SPINEL_HPP_
#define NCP_SPINEL_HPP_

#include <openthread/thread.h>

#include "openthread-spinel-config.h"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_base.hpp"

namespace ot {
namespace Spinel {

struct NcpSpinelCallbacks
{
    void (*mActiveScanDone)(otInstance *aInstance, otActiveScanResult *aScanResult);

    void (*mUpdateIp6AddressTable)(otInstance *aInstance, otNetifAddress *aAddressTable, uint8_t aSize);

    void (*mReceiveIp6)(otInstance *aInstance, const uint8_t *aData, uint16_t aLength);
};

class NcpSpinel : public SpinelCallbacks
{
public:
    NcpSpinel(otInstance *aInstance, SpinelBase &aSpinelBase);

    ~NcpSpinel(void) { Deinit(); }

    void Init(const NcpSpinelCallbacks &aCallbacks);

    void Deinit(void);

    otError GetDeviceRole(otDeviceRole &aRole);

    otError ActiveScan(void);

    otError Ip6Send(const uint8_t *aBuffer, uint16_t aLen);

    otError DatasetInitNew(otOperationalDataset *aDataset);

    otError Ip6SetEnabled(bool aEnabled);

    otError ThreadSetEnabled(bool aEnabled);

private:
    static constexpr uint32_t kIp6MaxDatagramLength = OPENTHREAD_CONFIG_IP6_MAX_DATAGRAM_LENGTH;
    static constexpr uint8_t  kMaxAddressNum        = 10;

    otError HandleCmdFromNotification(uint32_t          aCmd,
                                      spinel_prop_key_t aKey,
                                      const uint8_t    *aData,
                                      spinel_size_t     aLength,
                                      bool             &aShouldSaveFrame);
    otError HandleCmdFromSavedNotification(uint32_t          aCmd,
                                           spinel_prop_key_t aKey,
                                           const uint8_t    *aData,
                                           spinel_size_t     aLength);
    void HandleStreamRawResponse(uint32_t aCmd, spinel_prop_key_t aKey, const uint8_t *aBuffer, spinel_size_t aLength);
    otError HandleStreamMfgResponse(const uint8_t *aBuffer, spinel_size_t aLength);
    void    HandleCpTimeout(void);
    void    RecoverFromCpFailure(void);

    void HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void HandleValueInserted(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    otInstance        *mInstance;
    SpinelBase        &mSpinelBase;
    NcpSpinelCallbacks mCallbacks;

    uint8_t  mIpDatagramRecv[kIp6MaxDatagramLength];
    uint16_t mIpDatagramRecvLength;

    otNetifAddress mAddressTable[kMaxAddressNum];
};

} // namespace Spinel
} // namespace ot

#endif // NCP_SPINEL_HPP_
