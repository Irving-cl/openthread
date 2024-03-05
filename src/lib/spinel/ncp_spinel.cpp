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

#include "ncp_spinel.hpp"

#include <openthread/thread.h>

#include "common/num_utils.hpp"
#include "lib/platform/exit_code.h"
#include "lib/spinel/log.hpp"
#include "lib/spinel/ncp_spinel_parser.hpp"

namespace ot {
namespace Spinel {

static uint8_t  sBuffer[256];
static uint16_t sLen;

NcpSpinel::NcpSpinel(otInstance *aInstance, SpinelBase &aSpinelBase)
    : mInstance(aInstance)
    , mSpinelBase(aSpinelBase)
    , mIpDatagramRecvLength(0)
{
    memset(mIpDatagramRecv, 0, sizeof(mIpDatagramRecv));
}

void NcpSpinel::Init(const NcpSpinelCallbacks &aCallbacks)
{
    mSpinelBase.SetSpinelCallbacks(this);

    assert(aCallbacks.mActiveScanDone != nullptr);

    mCallbacks = aCallbacks;
}

void NcpSpinel::Deinit(void) {}

otError NcpSpinel::GetDeviceRole(otDeviceRole &aRole)
{
    otError           error = OT_ERROR_NONE;
    spinel_net_role_t spinelRole;

    SuccessOrExit(mSpinelBase.Get(SPINEL_PROP_NET_ROLE, SPINEL_DATATYPE_UINT_PACKED_S, &spinelRole));

    switch (spinelRole)
    {
    case SPINEL_NET_ROLE_DETACHED:
        aRole = OT_DEVICE_ROLE_DETACHED;
        break;
    case SPINEL_NET_ROLE_CHILD:
        aRole = OT_DEVICE_ROLE_CHILD;
        break;
    case SPINEL_NET_ROLE_ROUTER:
        aRole = OT_DEVICE_ROLE_ROUTER;
        break;
    case SPINEL_NET_ROLE_LEADER:
        aRole = OT_DEVICE_ROLE_LEADER;
        break;
    case SPINEL_NET_ROLE_DISABLED:
        aRole = OT_DEVICE_ROLE_DISABLED;
        break;
    }

exit:
    return error;
}

otError NcpSpinel::ActiveScan(void)
{
    otError error;

    // SuccessOrExit(error = Set(SPINEL_PROP_MAC_SCAN_MASK, SPINEL_DATATYPE_DATA_S, aScanChannel, sizeof(uint8_t)));
    // SuccessOrExit(error = Set(SPINEL_PROP_MAC_SCAN_PERIOD, SPINEL_DATATYPE_UINT16_S, aScanDuration));
    SuccessOrExit(error =
                      mSpinelBase.Set(SPINEL_PROP_MAC_SCAN_STATE, SPINEL_DATATYPE_UINT8_S, SPINEL_SCAN_STATE_BEACON));

exit:
    return error;
}

otError NcpSpinel::Ip6Send(const uint8_t *aBuffer, uint16_t aLen)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aBuffer != nullptr, error = OT_ERROR_INVALID_ARGS);

    LogInfo("Ip6Send");
    error = mSpinelBase.Request(SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_STREAM_NET, SPINEL_DATATYPE_DATA_WLEN_S, aBuffer,
                                aLen);

exit:
    return error;
}

otError NcpSpinel::DatasetInitNew(otOperationalDataset *aDataset)
{
    otError error = OT_ERROR_NONE;

    sLen  = sizeof(sBuffer);
    error = mSpinelBase.Get(SPINEL_PROP_THREAD_NEW_DATASET, SPINEL_DATATYPE_DATA_S, sBuffer, &sLen);

    SuccessOrExit(error);

    error = ParseOperationalDataset(sBuffer, sLen, aDataset);
exit:
    return error;
}

otError NcpSpinel::Ip6SetEnabled(bool aEnabled)
{
    return mSpinelBase.Set(SPINEL_PROP_NET_IF_UP, SPINEL_DATATYPE_BOOL_S, aEnabled);
}

otError NcpSpinel::ThreadSetEnabled(bool aEnabled)
{
    return mSpinelBase.Set(SPINEL_PROP_NET_STACK_UP, SPINEL_DATATYPE_BOOL_S, aEnabled);
}

otError NcpSpinel::HandleCmdFromNotification(uint32_t          aCmd,
                                             spinel_prop_key_t aKey,
                                             const uint8_t    *aData,
                                             spinel_size_t     aLength,
                                             bool             &aShouldSaveFrame)
{
    otError error = OT_ERROR_NONE;

    switch (aCmd)
    {
    case SPINEL_CMD_PROP_VALUE_IS:
        OT_UNUSED_VARIABLE(aShouldSaveFrame);
        HandleValueIs(aKey, aData, static_cast<uint16_t>(aLength));
        break;

    case SPINEL_CMD_PROP_VALUE_INSERTED:
        HandleValueInserted(aKey, aData, static_cast<uint16_t>(aLength));
        break;

    case SPINEL_CMD_PROP_VALUE_REMOVED:
        LogInfo("Ignored command %lu", ToUlong(aCmd));
        break;

    default:
        ExitNow(error = OT_ERROR_PARSE);
    }

exit:
    return error;
}

otError NcpSpinel::HandleCmdFromSavedNotification(uint32_t          aCmd,
                                                  spinel_prop_key_t aKey,
                                                  const uint8_t    *aData,
                                                  spinel_size_t     aLength)
{
    otError error = OT_ERROR_NONE;
    LogInfo("!!! HandleCmdFromSavedNotification");

    VerifyOrExit(aCmd == SPINEL_CMD_PROP_VALUE_IS, error = OT_ERROR_DROP);
    HandleValueIs(aKey, aData, static_cast<uint16_t>(aLength));

exit:
    LogIfFail("Error processing saved notification", error);

    return error;
}

void NcpSpinel::HandleStreamRawResponse(uint32_t          aCmd,
                                        spinel_prop_key_t aKey,
                                        const uint8_t    *aBuffer,
                                        spinel_size_t     aLength)
{
    OT_UNUSED_VARIABLE(aCmd);
    OT_UNUSED_VARIABLE(aKey);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);

    LogWarn("Unexpected response for STREAM_RAW");
}

otError NcpSpinel::HandleStreamMfgResponse(const uint8_t *aBuffer, spinel_size_t aLength)
{
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);

    LogWarn("Unexpected response for STREAM_MFG");

    return OT_ERROR_NONE;
}

void NcpSpinel::HandleCpTimeout(void) { DieNow(OT_EXIT_CO_PROCESSOR_NO_RESPONSE); }

void NcpSpinel::RecoverFromCpFailure(void)
{
    // Do not recover for now.
    return;
}

void NcpSpinel::HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    otError        error = OT_ERROR_NONE;
    spinel_ssize_t unpacked;

    if (aKey == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t status = SPINEL_STATUS_OK;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, "i", &status);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

        if (status >= SPINEL_STATUS_RESET__BEGIN && status <= SPINEL_STATUS_RESET__END)
        {
            LogInfo("NCP reset: %s", spinel_status_to_cstr(status));
        }
        else
        {
            LogInfo("RCP last status: %s", spinel_status_to_cstr(status));
        }
    }
    else if (aKey == SPINEL_PROP_MAC_SCAN_STATE)
    {
        LogInfo("MAC SCAN STATE");
        uint8_t scanState;
        unpacked = spinel_datatype_unpack(aBuffer, aLength, "C", &scanState);

        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
        VerifyOrExit(scanState == SPINEL_SCAN_STATE_IDLE, error = OT_ERROR_FAILED);

        mCallbacks.mActiveScanDone(mInstance, nullptr);
    }
    else if (aKey == SPINEL_PROP_IPV6_ADDRESS_TABLE)
    {
        uint8_t numAddr;

        SuccessOrExit(error = ParseIp6Addresses(aBuffer, aLength, mAddressTable, numAddr));
        mCallbacks.mUpdateIp6AddressTable(mInstance, mAddressTable, numAddr);

        LogInfo("Receive IPv6 address table");
    }
    else if (aKey == SPINEL_PROP_STREAM_NET)
    {
        LogInfo("Ip6 Receive");
        mIpDatagramRecvLength = sizeof(mIpDatagramRecv);
        unpacked = spinel_datatype_unpack_in_place(aBuffer, aLength, SPINEL_DATATYPE_DATA_WLEN_S, &mIpDatagramRecv[0],
                                                   &mIpDatagramRecvLength);

        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
        mCallbacks.mReceiveIp6(mInstance, mIpDatagramRecv, mIpDatagramRecvLength);
    }

exit:
    LogIfFail("Failed to handle ValueIs", error);
}

void NcpSpinel::HandleValueInserted(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    Error          error = OT_ERROR_NONE;
    spinel_ssize_t unpacked;

    if (aKey == SPINEL_PROP_MAC_SCAN_BEACON)
    {
        otActiveScanResult result;

        spinel_size_t       extPanIdLen;
        spinel_size_t       steeringDataLen;
        uint8_t             flags;
        const otExtAddress *extAddress;
        const char         *networkName;

        result.mDiscover = false;
        unpacked = spinel_datatype_unpack(aBuffer, aLength, "Cct(ESSC)t(iCUdd)", &result.mChannel, &result.mRssi,
                                          &extAddress, nullptr, &result.mPanId, &result.mLqi, nullptr, &flags,
                                          &networkName, nullptr, &extPanIdLen, nullptr, &steeringDataLen);

        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

        memcpy(&result.mExtAddress.m8[0], extAddress, sizeof(result.mExtAddress.m8));
        mCallbacks.mActiveScanDone(mInstance, &result);
    }
    else
    {
        LogInfo("HandleValueInserted, Ignore");
    }

exit:
    LogIfFail("HandleValueInserted", error);
}

} // namespace Spinel
} // namespace ot
