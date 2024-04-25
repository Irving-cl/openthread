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

#include "lib/spinel/logger.hpp"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_driver.hpp"

namespace ot {
namespace Spinel {

constexpr spinel_tid_t sTid = 1; ///< 1 will be used the Tid for all the APIs of NcpSpinel

NcpSpinel::NcpSpinel(void)
    : Logger("NcpSpinel")
    , mSpinelDriver(nullptr)
    , mWaitingKey(SPINEL_PROP_LAST_STATUS)
    , mDeviceRole(OT_DEVICE_ROLE_DISABLED)
{
}

NcpSpinel::~NcpSpinel(void) = default;

void NcpSpinel::Init(SpinelDriver *aSpinelDriver)
{
    mSpinelDriver = aSpinelDriver;
    mSpinelDriver->SetFrameHandler(&HandleReceivedFrame, &HandleSavedFrame, this);
}

void NcpSpinel::Deinit(void) { mSpinelDriver = nullptr; }

void NcpSpinel::ActiveScan(uint32_t aScanChannels, uint16_t aScanDuration, ActiveScanCallback aCallback, void *aContext)
{
    (void)aScanChannels;
    (void)aScanDuration;
    (void)aCallback;
    (void)aContext;
}

void NcpSpinel::SetThreadEnabled(bool aEnabled, OperationResultReceiver aReceiver, void *aContext)
{
    (void)aEnabled;
    (void)aReceiver;
    (void)aContext;
}

void NcpSpinel::Join(otOperationalDatasetTlvs &aActiveOpDatasetTlvs, OperationResultReceiver aReceiver, void *aContext)
{
    (void)aActiveOpDatasetTlvs;
    (void)aReceiver;
    (void)aContext;
}

void NcpSpinel::Leave(OperationResultReceiver aReceiver, void *aContext)
{
    (void)aReceiver;
    (void)aContext;
}

void NcpSpinel::HandleReceivedFrame(const uint8_t *aFrame,
                                      uint16_t       aLength,
                                      uint8_t        aHeader,
                                      bool          &aSave,
                                      void          *aContext)
{
    static_cast<NcpSpinel *>(aContext)->HandleReceivedFrame(aFrame, aLength, aHeader, aSave);
}

void NcpSpinel::HandleReceivedFrame(const uint8_t *aFrame, uint16_t aLength, uint8_t aHeader, bool &aShouldSaveFrame)
{
    if (SPINEL_HEADER_GET_TID(aHeader) == 0)
    {
        HandleNotification(aFrame, aLength);
    }
    else if (SPINEL_HEADER_GET_TID(aHeader) == sTid)
    {
        HandleResponse(aFrame, aLength);
    }
    else
    {
        LogCrit("Received unexpected tid: %u", sTid);
    }

    aShouldSaveFrame = false;
}

void NcpSpinel::HandleSavedFrame(const uint8_t *aFrame, uint16_t aLength, void *aContext)
{
    OT_UNUSED_VARIABLE(aFrame);
    OT_UNUSED_VARIABLE(aLength);
    OT_UNUSED_VARIABLE(aContext);
}

void NcpSpinel::HandleNotification(const uint8_t *aFrame, uint16_t aLength)
{
    spinel_prop_key_t key;
    spinel_size_t     len = 0;
    spinel_ssize_t    unpacked;
    uint8_t          *data = nullptr;
    uint32_t          cmd;
    uint8_t           header;
    otError           error = OT_ERROR_NONE;

    unpacked = spinel_datatype_unpack(aFrame, aLength, "CiiD", &header, &cmd, &key, &data, &len);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
    VerifyOrExit(SPINEL_HEADER_GET_TID(header) == 0, error = OT_ERROR_PARSE);
    VerifyOrExit(cmd == SPINEL_CMD_PROP_VALUE_IS);
    HandleValueIs(key, data, static_cast<uint16_t>(len));

exit:
    LogIfFail("Error processing saved notification", error);
}

void NcpSpinel::HandleResponse(const uint8_t *aFrame, uint16_t aLength)
{
    spinel_prop_key_t key;
    spinel_size_t     len = 0;
    spinel_ssize_t    unpacked;
    uint8_t          *data = nullptr;
    uint32_t          cmd;
    uint8_t           header;
    otError           error = OT_ERROR_NONE;

    unpacked = spinel_datatype_unpack(aFrame, aLength, "CiiD", &header, &cmd, &key, &data, &len);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
    VerifyOrExit(cmd == SPINEL_CMD_PROP_VALUE_IS);
    VerifyOrExit(key == mWaitingKey, error = OT_ERROR_INVALID_STATE);

    mWaitingKey = SPINEL_PROP_LAST_STATUS;

exit:
    if (error == OT_ERROR_INVALID_STATE)
    {
        LogCrit("Received unexpected response with key:%u, waiting key:%u", key, mWaitingKey);
    }
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

        LogInfo("RCP last status: %s", spinel_status_to_cstr(status));
    }
    else if (aKey == SPINEL_PROP_NET_ROLE)
    {
        spinel_net_role_t role;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, "C", &role);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

	switch (role)
	{
	case SPINEL_NET_ROLE_DETACHED:
	    mDeviceRole = OT_DEVICE_ROLE_DETACHED;
	    break;
	case SPINEL_NET_ROLE_CHILD:
	    mDeviceRole = OT_DEVICE_ROLE_CHILD;
	    break;
	case SPINEL_NET_ROLE_ROUTER:
	    mDeviceRole = OT_DEVICE_ROLE_ROUTER;
	    break;
	case SPINEL_NET_ROLE_LEADER:
	    mDeviceRole = OT_DEVICE_ROLE_LEADER;
	    break;
	case SPINEL_NET_ROLE_DISABLED:
	    mDeviceRole = OT_DEVICE_ROLE_DISABLED;
	    break;
	}

        LogInfo("Device role changed to %s", otThreadDeviceRoleToString(mDeviceRole));
    }

exit:
    LogIfFail("");
    return;
}

} // namespace Spinel
} // namespace ot
