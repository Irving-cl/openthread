/*
 *  Copyright (c) 2020, The OpenThread Authors.
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
 *   This file implements the spinel based radio transceiver.
 */

#include "radio_spinel.hpp"

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>

#include <openthread/logging.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>

#include "common/code_utils.hpp"
#include "common/encoding.hpp"
#include "common/new.hpp"
#include "lib/platform/exit_code.h"
#include "lib/spinel/log.hpp"
#include "lib/spinel/spinel_decoder.hpp"

namespace ot {
namespace Spinel {

otExtAddress RadioSpinel::sIeeeEui64;

bool RadioSpinel::sSupportsLogStream =
    false; ///< RCP supports `LOG_STREAM` property with OpenThread log meta-data format.

bool RadioSpinel::sSupportsResetToBootloader = false; ///< RCP supports resetting into bootloader mode.

otRadioCaps RadioSpinel::sRadioCaps = OT_RADIO_CAPS_NONE;

RadioSpinel::RadioSpinel(SpinelBase &aSpinelBase)
    : mSpinelBase(aSpinelBase)
    , mTransmitFrame(nullptr)
    , mShortAddress(0)
    , mPanId(0xffff)
    , mChannel(0)
    , mRxSensitivity(0)
    , mState(kStateDisabled)
    , mIsPromiscuous(false)
    , mRxOnWhenIdle(true)
    , mIsTimeSynced(false)
#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    , mRcpFailureCount(0)
    , mRcpFailure(kRcpFailureNone)
    , mSrcMatchShortEntryCount(0)
    , mSrcMatchExtEntryCount(0)
    , mMacKeySet(false)
    , mCcaEnergyDetectThresholdSet(false)
    , mTransmitPowerSet(false)
    , mCoexEnabledSet(false)
    , mFemLnaGainSet(false)
    , mEnergyScanning(false)
    , mMacFrameCounterSet(false)
#endif
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    , mDiagMode(false)
    , mDiagOutput(nullptr)
    , mDiagOutputMaxLen(0)
#endif
    , mTxRadioEndUs(UINT64_MAX)
    , mRadioTimeRecalcStart(UINT64_MAX)
    , mRadioTimeOffset(UINT64_MAX)
{
    memset(&mRadioSpinelMetrics, 0, sizeof(mRadioSpinelMetrics));
    memset(&mCallbacks, 0, sizeof(mCallbacks));
}

void RadioSpinel::Init(bool aSkipRcpCompatibilityCheck)
{
    otError error = OT_ERROR_NONE;
    bool    supportsRcpApiVersion;
    bool    supportsRcpMinHostApiVersion;

    mSpinelBase.SetSpinelCallbacks(this);

    SuccessOrExit(error = mSpinelBase.Get(SPINEL_PROP_HWADDR, SPINEL_DATATYPE_EUI64_S, sIeeeEui64.m8));

    VerifyOrDie(IsRcp(supportsRcpApiVersion, supportsRcpMinHostApiVersion), OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);

    if (!aSkipRcpCompatibilityCheck)
    {
        SuccessOrDie(CheckRcpApiVersion(supportsRcpApiVersion, supportsRcpMinHostApiVersion));
        SuccessOrDie(CheckRadioCapabilities());
    }

    mRxRadioFrame.mPsdu  = mRxPsdu;
    mTxRadioFrame.mPsdu  = mTxPsdu;
    mAckRadioFrame.mPsdu = mAckPsdu;

exit:
    SuccessOrDie(error);
}

void RadioSpinel::SetCallbacks(const struct RadioSpinelCallbacks &aCallbacks)
{
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    assert(aCallbacks.mDiagReceiveDone != nullptr);
    assert(aCallbacks.mDiagTransmitDone != nullptr);
#endif
    assert(aCallbacks.mEnergyScanDone != nullptr);
    assert(aCallbacks.mReceiveDone != nullptr);
    assert(aCallbacks.mTransmitDone != nullptr);
    assert(aCallbacks.mTxStarted != nullptr);

    mCallbacks = aCallbacks;
}

void RadioSpinel::Deinit(void) { new (this) RadioSpinel(mSpinelBase); }

otError RadioSpinel::ParseRadioFrame(otRadioFrame   &aFrame,
                                     const uint8_t  *aBuffer,
                                     uint16_t        aLength,
                                     spinel_ssize_t &aUnpacked)
{
    otError        error        = OT_ERROR_NONE;
    uint16_t       flags        = 0;
    int8_t         noiseFloor   = -128;
    spinel_size_t  size         = OT_RADIO_FRAME_MAX_SIZE;
    unsigned int   receiveError = 0;
    spinel_ssize_t unpacked;

    VerifyOrExit(aLength > 0, aFrame.mLength = 0);

    unpacked = spinel_datatype_unpack_in_place(aBuffer, aLength,
                                               SPINEL_DATATYPE_DATA_WLEN_S                          // Frame
                                                   SPINEL_DATATYPE_INT8_S                           // RSSI
                                                       SPINEL_DATATYPE_INT8_S                       // Noise Floor
                                                           SPINEL_DATATYPE_UINT16_S                 // Flags
                                                               SPINEL_DATATYPE_STRUCT_S(            // PHY-data
                                                                   SPINEL_DATATYPE_UINT8_S          // 802.15.4 channel
                                                                       SPINEL_DATATYPE_UINT8_S      // 802.15.4 LQI
                                                                           SPINEL_DATATYPE_UINT64_S // Timestamp (us).
                                                                   ) SPINEL_DATATYPE_STRUCT_S(      // Vendor-data
                                                                   SPINEL_DATATYPE_UINT_PACKED_S    // Receive error
                                                                   ),
                                               aFrame.mPsdu, &size, &aFrame.mInfo.mRxInfo.mRssi, &noiseFloor, &flags,
                                               &aFrame.mChannel, &aFrame.mInfo.mRxInfo.mLqi,
                                               &aFrame.mInfo.mRxInfo.mTimestamp, &receiveError);

    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
    aUnpacked = unpacked;

    aBuffer += unpacked;
    aLength -= static_cast<uint16_t>(unpacked);

    if (sRadioCaps & OT_RADIO_CAPS_TRANSMIT_SEC)
    {
        unpacked =
            spinel_datatype_unpack_in_place(aBuffer, aLength,
                                            SPINEL_DATATYPE_STRUCT_S(        // MAC-data
                                                SPINEL_DATATYPE_UINT8_S      // Security key index
                                                    SPINEL_DATATYPE_UINT32_S // Security frame counter
                                                ),
                                            &aFrame.mInfo.mRxInfo.mAckKeyId, &aFrame.mInfo.mRxInfo.mAckFrameCounter);

        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
        aUnpacked += unpacked;

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
        if (flags & SPINEL_MD_FLAG_ACKED_SEC)
        {
            mMacFrameCounterSet = true;
            mMacFrameCounter    = aFrame.mInfo.mRxInfo.mAckFrameCounter;
        }
#endif
    }

    if (receiveError == OT_ERROR_NONE)
    {
        aFrame.mLength = static_cast<uint8_t>(size);

        aFrame.mInfo.mRxInfo.mAckedWithFramePending = ((flags & SPINEL_MD_FLAG_ACKED_FP) != 0);
        aFrame.mInfo.mRxInfo.mAckedWithSecEnhAck    = ((flags & SPINEL_MD_FLAG_ACKED_SEC) != 0);
    }
    else if (receiveError < OT_NUM_ERRORS)
    {
        error = static_cast<otError>(receiveError);
    }
    else
    {
        error = OT_ERROR_PARSE;
    }

exit:
    LogIfFail("Handle radio frame failed", error);
    return error;
}

otError RadioSpinel::HandleCmdFromNotification(uint32_t          aCmd,
                                               spinel_prop_key_t aKey,
                                               const uint8_t    *aData,
                                               spinel_size_t     aLength,
                                               bool             &aShouldSaveFrame)
{
    otError error = OT_ERROR_NONE;

    switch (aCmd)
    {
    case SPINEL_CMD_PROP_VALUE_IS:
        // Some spinel properties cannot be handled during `WaitResponse()`, we must cache these events.
        // `mWaitingTid` is released immediately after received the response. And `mWaitingKey` is be set
        // to `SPINEL_PROP_LAST_STATUS` at the end of `WaitResponse()`.

        if (!IsSafeToHandleNow(aKey))
        {
            ExitNow(aShouldSaveFrame = true);
        }

        HandleValueIs(aKey, aData, static_cast<uint16_t>(aLength));
        break;

    case SPINEL_CMD_PROP_VALUE_INSERTED:
    case SPINEL_CMD_PROP_VALUE_REMOVED:
        LogInfo("Ignored command %lu", ToUlong(aCmd));
        break;

    default:
        ExitNow(error = OT_ERROR_PARSE);
    }

exit:
    return error;
}

otError RadioSpinel::HandleCmdFromSavedNotification(uint32_t          aCmd,
                                                    spinel_prop_key_t aKey,
                                                    const uint8_t    *aData,
                                                    spinel_size_t     aLength)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aCmd == SPINEL_CMD_PROP_VALUE_IS, error = OT_ERROR_DROP);
    HandleValueIs(aKey, aData, static_cast<uint16_t>(aLength));

exit:
    LogIfFail("Error processing saved notification", error);

    return error;
}

void RadioSpinel::HandleStreamRawResponse(uint32_t          aCmd,
                                          spinel_prop_key_t aKey,
                                          const uint8_t    *aBuffer,
                                          spinel_size_t     aLength)
{
    if (mState == kStateTransmitting)
    {
        HandleTransmitDone(aCmd, aKey, aBuffer, static_cast<uint16_t>(aLength));
    }
}

otError RadioSpinel::HandleStreamMfgResponse(const uint8_t *aBuffer, spinel_size_t aLength)
{
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    otError        error = OT_ERROR_NONE;
    spinel_ssize_t unpacked;

    VerifyOrExit(mDiagOutput != nullptr);
    unpacked =
        spinel_datatype_unpack_in_place(aBuffer, aLength, SPINEL_DATATYPE_UTF8_S, mDiagOutput, &mDiagOutputMaxLen);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

exit:
    return error;
#else
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);

    return OT_ERROR_NOT_CAPABLE;
#endif
}

void RadioSpinel::HandleCpTimeout(void)
{
    mRadioSpinelMetrics.mRcpTimeoutCount++;

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mRcpFailure = kRcpFailureTimeout;
#else

    DieNow(OT_EXIT_RADIO_SPINEL_NO_RESPONSE);
#endif
}

void RadioSpinel::RecoverFromCpFailure(void)
{
#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    constexpr int16_t kMaxFailureCount = OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT;
    State             recoveringState  = mState;
    bool              skipReset        = false;

    if (mRcpFailure == kRcpFailureNone)
    {
        ExitNow();
    }

    LogCrit("RecoverFromCpFailure");
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
    skipReset = (mRcpFailure == kRcpFailureUnexpectedReset);
#endif

    mRcpFailure = kRcpFailureNone;

    LogWarn("RCP failure detected");

    ++mRadioSpinelMetrics.mRcpRestorationCount;
    ++mRcpFailureCount;
    if (mRcpFailureCount > kMaxFailureCount)
    {
        LogCrit("Too many rcp failures, exiting");
        DieNow(OT_EXIT_FAILURE);
    }

    LogWarn("Trying to recover (%d/%d)", mRcpFailureCount, kMaxFailureCount);

    mState = kStateDisabled;
    mSpinelBase.Recover(skipReset);

    mIsTimeSynced = false;

    if (!skipReset)
    {
        mSpinelBase.ResetCp(mResetRadioOnStartup);
        mSpinelBase.SetSpinelCallbacks(this);
    }

    SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, true));
    mState = kStateSleep;

    RestoreProperties();

    switch (recoveringState)
    {
    case kStateDisabled:
        mState = kStateDisabled;
        break;
    case kStateSleep:
        break;
    case kStateReceive:
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
        // In case multiple PANs are running, don't force RCP to receive state.
        IgnoreError(mSpinelBase.Set(SPINEL_PROP_MAC_RAW_STREAM_ENABLED, SPINEL_DATATYPE_BOOL_S, true));
#else
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_MAC_RAW_STREAM_ENABLED, SPINEL_DATATYPE_BOOL_S, true));
#endif
        mState = kStateReceive;
        break;
    case kStateTransmitting:
    case kStateTransmitDone:
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
        // In case multiple PANs are running, don't force RCP to receive state.
        IgnoreError(mSpinelBase.Set(SPINEL_PROP_MAC_RAW_STREAM_ENABLED, SPINEL_DATATYPE_BOOL_S, true));
#else
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_MAC_RAW_STREAM_ENABLED, SPINEL_DATATYPE_BOOL_S, true));
#endif
        mTxError = OT_ERROR_ABORT;
        mState   = kStateTransmitDone;
        break;
    }

    if (mEnergyScanning)
    {
        SuccessOrDie(EnergyScan(mScanChannel, mScanDuration));
    }

    --mRcpFailureCount;
    LogNote("RCP recovery is done");

exit:
    return;
#endif // OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
}

void RadioSpinel::HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    otError        error = OT_ERROR_NONE;
    spinel_ssize_t unpacked;

    if (aKey == SPINEL_PROP_STREAM_RAW)
    {
        SuccessOrExit(error = ParseRadioFrame(mRxRadioFrame, aBuffer, aLength, unpacked));
        RadioReceive();
    }
    else if (aKey == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t status = SPINEL_STATUS_OK;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, "i", &status);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

        if (status >= SPINEL_STATUS_RESET__BEGIN && status <= SPINEL_STATUS_RESET__END)
        {
            if (IsEnabled())
            {
                HandleRcpUnexpectedReset(status);
                ExitNow();
            }

            LogInfo("RCP reset: %s", spinel_status_to_cstr(status));
        }
        else if (status == SPINEL_STATUS_SWITCHOVER_DONE || status == SPINEL_STATUS_SWITCHOVER_FAILED)
        {
            if (mCallbacks.mSwitchoverDone != nullptr)
            {
                mCallbacks.mSwitchoverDone(mInstance, status == SPINEL_STATUS_SWITCHOVER_DONE);
            }
        }
        else
        {
            LogInfo("RCP last status: %s", spinel_status_to_cstr(status));
        }
    }
    else if (aKey == SPINEL_PROP_MAC_ENERGY_SCAN_RESULT)
    {
        uint8_t scanChannel;
        int8_t  maxRssi;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, "Cc", &scanChannel, &maxRssi);

        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
        mEnergyScanning = false;
#endif

        mCallbacks.mEnergyScanDone(mInstance, maxRssi);
    }
    else if (aKey == SPINEL_PROP_STREAM_DEBUG)
    {
        char         logStream[OPENTHREAD_CONFIG_NCP_SPINEL_LOG_MAX_SIZE + 1];
        unsigned int len = sizeof(logStream);

        unpacked = spinel_datatype_unpack_in_place(aBuffer, aLength, SPINEL_DATATYPE_DATA_S, logStream, &len);
        assert(len < sizeof(logStream));
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
        logStream[len] = '\0';
        LogDebg("RCP => %s", logStream);
    }
    else if ((aKey == SPINEL_PROP_STREAM_LOG) && sSupportsLogStream)
    {
        const char *logString;
        uint8_t     logLevel;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UTF8_S, &logString);
        VerifyOrExit(unpacked >= 0, error = OT_ERROR_PARSE);
        aBuffer += unpacked;
        aLength -= unpacked;

        unpacked = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UINT8_S, &logLevel);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

        switch (logLevel)
        {
        case SPINEL_NCP_LOG_LEVEL_EMERG:
        case SPINEL_NCP_LOG_LEVEL_ALERT:
        case SPINEL_NCP_LOG_LEVEL_CRIT:
            LogCrit("RCP => %s", logString);
            break;

        case SPINEL_NCP_LOG_LEVEL_ERR:
        case SPINEL_NCP_LOG_LEVEL_WARN:
            LogWarn("RCP => %s", logString);
            break;

        case SPINEL_NCP_LOG_LEVEL_NOTICE:
            LogNote("RCP => %s", logString);
            break;

        case SPINEL_NCP_LOG_LEVEL_INFO:
            LogInfo("RCP => %s", logString);
            break;

        case SPINEL_NCP_LOG_LEVEL_DEBUG:
        default:
            LogDebg("RCP => %s", logString);
            break;
        }
    }
#if OPENTHREAD_SPINEL_CONFIG_VENDOR_HOOK_ENABLE
    else if (aKey >= SPINEL_PROP_VENDOR__BEGIN && aKey < SPINEL_PROP_VENDOR__END)
    {
        error = VendorHandleValueIs(aKey);
    }
#endif

exit:
    LogIfFail("Failed to handle ValueIs", error);
}

void RadioSpinel::HandleTransmitDone(uint32_t          aCommand,
                                     spinel_prop_key_t aKey,
                                     const uint8_t    *aBuffer,
                                     uint16_t          aLength)
{
    otError         error         = OT_ERROR_NONE;
    spinel_status_t status        = SPINEL_STATUS_OK;
    bool            framePending  = false;
    bool            headerUpdated = false;
    spinel_ssize_t  unpacked;

    VerifyOrExit(aCommand == SPINEL_CMD_PROP_VALUE_IS && aKey == SPINEL_PROP_LAST_STATUS, error = OT_ERROR_FAILED);

    unpacked = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UINT_PACKED_S, &status);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

    aBuffer += unpacked;
    aLength -= static_cast<uint16_t>(unpacked);

    unpacked = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_BOOL_S, &framePending);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

    aBuffer += unpacked;
    aLength -= static_cast<uint16_t>(unpacked);

    unpacked = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_BOOL_S, &headerUpdated);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);

    aBuffer += unpacked;
    aLength -= static_cast<uint16_t>(unpacked);

    if (status == SPINEL_STATUS_OK)
    {
        SuccessOrExit(error = ParseRadioFrame(mAckRadioFrame, aBuffer, aLength, unpacked));
        aBuffer += unpacked;
        aLength -= static_cast<uint16_t>(unpacked);
    }
    else
    {
        error = ot::Spinel::SpinelBase::SpinelStatusToOtError(status);
    }

    static_cast<Mac::TxFrame *>(mTransmitFrame)->SetIsHeaderUpdated(headerUpdated);

    if ((sRadioCaps & OT_RADIO_CAPS_TRANSMIT_SEC) && headerUpdated &&
        static_cast<Mac::TxFrame *>(mTransmitFrame)->GetSecurityEnabled())
    {
        uint8_t  keyId;
        uint32_t frameCounter;

        // Replace transmit frame security key index and frame counter with the one filled by RCP
        unpacked = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_UINT32_S, &keyId,
                                          &frameCounter);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
        static_cast<Mac::TxFrame *>(mTransmitFrame)->SetKeyId(keyId);
        static_cast<Mac::TxFrame *>(mTransmitFrame)->SetFrameCounter(frameCounter);

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
        mMacFrameCounterSet = true;
        mMacFrameCounter    = frameCounter;
#endif
    }

exit:
    // A parse error indicates an RCP misbehavior, so recover the RCP immediately.
    mState = kStateTransmitDone;
    if (error != OT_ERROR_PARSE)
    {
        mTxError = error;
    }
    else
    {
        mTxError = kErrorAbort;
        HandleCpTimeout();
        RecoverFromCpFailure();
    }

    LogIfFail("Handle transmit done failed", error);
}

otError RadioSpinel::SetPromiscuous(bool aEnable)
{
    otError error;

    uint8_t mode = (aEnable ? SPINEL_MAC_PROMISCUOUS_MODE_NETWORK : SPINEL_MAC_PROMISCUOUS_MODE_OFF);
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_PROMISCUOUS_MODE, SPINEL_DATATYPE_UINT8_S, mode));
    mIsPromiscuous = aEnable;

exit:
    return error;
}

otError RadioSpinel::SetRxOnWhenIdle(bool aEnable)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mRxOnWhenIdle != aEnable);
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_RX_ON_WHEN_IDLE_MODE, SPINEL_DATATYPE_BOOL_S, aEnable));
    mRxOnWhenIdle = aEnable;

exit:
    return error;
}

otError RadioSpinel::SetShortAddress(uint16_t aAddress)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mShortAddress != aAddress);
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_15_4_SADDR, SPINEL_DATATYPE_UINT16_S, aAddress));
    mShortAddress = aAddress;

exit:
    return error;
}

otError RadioSpinel::GetIeeeEui64(uint8_t *aIeeeEui64)
{
    memcpy(aIeeeEui64, sIeeeEui64.m8, sizeof(sIeeeEui64.m8));

    return OT_ERROR_NONE;
}

otError RadioSpinel::SetExtendedAddress(const otExtAddress &aExtAddress)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_15_4_LADDR, SPINEL_DATATYPE_EUI64_S, aExtAddress.m8));
    mExtendedAddress = aExtAddress;

exit:
    return error;
}

otError RadioSpinel::SetPanId(uint16_t aPanId)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mPanId != aPanId);
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_15_4_PANID, SPINEL_DATATYPE_UINT16_S, aPanId));
    mPanId = aPanId;

exit:
    return error;
}

otError RadioSpinel::GetTransmitPower(int8_t &aPower)
{
    otError error = mSpinelBase.Get(SPINEL_PROP_PHY_TX_POWER, SPINEL_DATATYPE_INT8_S, &aPower);

    LogIfFail("Get transmit power failed", error);
    return error;
}

otError RadioSpinel::SetTransmitPower(int8_t aPower)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_PHY_TX_POWER, SPINEL_DATATYPE_INT8_S, aPower));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mTransmitPower    = aPower;
    mTransmitPowerSet = true;
#endif

exit:
    LogIfFail("Set transmit power failed", error);
    return error;
}

otError RadioSpinel::GetCcaEnergyDetectThreshold(int8_t &aThreshold)
{
    otError error = mSpinelBase.Get(SPINEL_PROP_PHY_CCA_THRESHOLD, SPINEL_DATATYPE_INT8_S, &aThreshold);

    LogIfFail("Get CCA ED threshold failed", error);
    return error;
}

otError RadioSpinel::SetCcaEnergyDetectThreshold(int8_t aThreshold)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_PHY_CCA_THRESHOLD, SPINEL_DATATYPE_INT8_S, aThreshold));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mCcaEnergyDetectThreshold    = aThreshold;
    mCcaEnergyDetectThresholdSet = true;
#endif

exit:
    LogIfFail("Set CCA ED threshold failed", error);
    return error;
}

otError RadioSpinel::GetFemLnaGain(int8_t &aGain)
{
    otError error = mSpinelBase.Get(SPINEL_PROP_PHY_FEM_LNA_GAIN, SPINEL_DATATYPE_INT8_S, &aGain);

    LogIfFail("Get FEM LNA gain failed", error);
    return error;
}

otError RadioSpinel::SetFemLnaGain(int8_t aGain)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_PHY_FEM_LNA_GAIN, SPINEL_DATATYPE_INT8_S, aGain));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mFemLnaGain    = aGain;
    mFemLnaGainSet = true;
#endif

exit:
    LogIfFail("Set FEM LNA gain failed", error);
    return error;
}

int8_t RadioSpinel::GetRssi(void)
{
    int8_t  rssi  = OT_RADIO_RSSI_INVALID;
    otError error = mSpinelBase.Get(SPINEL_PROP_PHY_RSSI, SPINEL_DATATYPE_INT8_S, &rssi);

    LogIfFail("Get RSSI failed", error);
    return rssi;
}

otRadioState RadioSpinel::GetState(void) const
{
    static const otRadioState sOtRadioStateMap[] = {
        OT_RADIO_STATE_DISABLED, OT_RADIO_STATE_SLEEP,    OT_RADIO_STATE_RECEIVE,
        OT_RADIO_STATE_TRANSMIT, OT_RADIO_STATE_TRANSMIT,
    };

    return sOtRadioStateMap[mState];
}

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError RadioSpinel::SetCoexEnabled(bool aEnabled)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_RADIO_COEX_ENABLE, SPINEL_DATATYPE_BOOL_S, aEnabled));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mCoexEnabled    = aEnabled;
    mCoexEnabledSet = true;
#endif

exit:
    return error;
}

bool RadioSpinel::IsCoexEnabled(void)
{
    bool    enabled;
    otError error = mSpinelBase.Get(SPINEL_PROP_RADIO_COEX_ENABLE, SPINEL_DATATYPE_BOOL_S, &enabled);

    LogIfFail("Get Coex State failed", error);
    return enabled;
}

otError RadioSpinel::GetCoexMetrics(otRadioCoexMetrics &aCoexMetrics)
{
    otError error;

    error = mSpinelBase.Get(SPINEL_PROP_RADIO_COEX_METRICS,
                            SPINEL_DATATYPE_STRUCT_S(                            // Tx Coex Metrics Structure
                                SPINEL_DATATYPE_UINT32_S                         // NumTxRequest
                                    SPINEL_DATATYPE_UINT32_S                     // NumTxGrantImmediate
                                        SPINEL_DATATYPE_UINT32_S                 // NumTxGrantWait
                                            SPINEL_DATATYPE_UINT32_S             // NumTxGrantWaitActivated
                                                SPINEL_DATATYPE_UINT32_S         // NumTxGrantWaitTimeout
                                                    SPINEL_DATATYPE_UINT32_S     // NumTxGrantDeactivatedDuringRequest
                                                        SPINEL_DATATYPE_UINT32_S // NumTxDelayedGrant
                                                            SPINEL_DATATYPE_UINT32_S // AvgTxRequestToGrantTime
                                ) SPINEL_DATATYPE_STRUCT_S(                          // Rx Coex Metrics Structure
                                SPINEL_DATATYPE_UINT32_S                             // NumRxRequest
                                    SPINEL_DATATYPE_UINT32_S                         // NumRxGrantImmediate
                                        SPINEL_DATATYPE_UINT32_S                     // NumRxGrantWait
                                            SPINEL_DATATYPE_UINT32_S                 // NumRxGrantWaitActivated
                                                SPINEL_DATATYPE_UINT32_S             // NumRxGrantWaitTimeout
                                                    SPINEL_DATATYPE_UINT32_S     // NumRxGrantDeactivatedDuringRequest
                                                        SPINEL_DATATYPE_UINT32_S // NumRxDelayedGrant
                                                            SPINEL_DATATYPE_UINT32_S     // AvgRxRequestToGrantTime
                                                                SPINEL_DATATYPE_UINT32_S // NumRxGrantNone
                                ) SPINEL_DATATYPE_BOOL_S                                 // Stopped
                                SPINEL_DATATYPE_UINT32_S,                                // NumGrantGlitch
                            &aCoexMetrics.mNumTxRequest, &aCoexMetrics.mNumTxGrantImmediate,
                            &aCoexMetrics.mNumTxGrantWait, &aCoexMetrics.mNumTxGrantWaitActivated,
                            &aCoexMetrics.mNumTxGrantWaitTimeout, &aCoexMetrics.mNumTxGrantDeactivatedDuringRequest,
                            &aCoexMetrics.mNumTxDelayedGrant, &aCoexMetrics.mAvgTxRequestToGrantTime,
                            &aCoexMetrics.mNumRxRequest, &aCoexMetrics.mNumRxGrantImmediate,
                            &aCoexMetrics.mNumRxGrantWait, &aCoexMetrics.mNumRxGrantWaitActivated,
                            &aCoexMetrics.mNumRxGrantWaitTimeout, &aCoexMetrics.mNumRxGrantDeactivatedDuringRequest,
                            &aCoexMetrics.mNumRxDelayedGrant, &aCoexMetrics.mAvgRxRequestToGrantTime,
                            &aCoexMetrics.mNumRxGrantNone, &aCoexMetrics.mStopped, &aCoexMetrics.mNumGrantGlitch);

    LogIfFail("Get Coex Metrics failed", error);
    return error;
}
#endif // OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE

otError RadioSpinel::EnableSrcMatch(bool aEnable)
{
    return mSpinelBase.Set(SPINEL_PROP_MAC_SRC_MATCH_ENABLED, SPINEL_DATATYPE_BOOL_S, aEnable);
}

otError RadioSpinel::AddSrcMatchShortEntry(uint16_t aShortAddress)
{
    otError error;

    SuccessOrExit(
        error = mSpinelBase.Insert(SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES, SPINEL_DATATYPE_UINT16_S, aShortAddress));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    assert(mSrcMatchShortEntryCount < OPENTHREAD_CONFIG_MLE_MAX_CHILDREN);

    for (int i = 0; i < mSrcMatchShortEntryCount; ++i)
    {
        if (mSrcMatchShortEntries[i] == aShortAddress)
        {
            ExitNow();
        }
    }
    mSrcMatchShortEntries[mSrcMatchShortEntryCount] = aShortAddress;
    ++mSrcMatchShortEntryCount;
#endif

exit:
    return error;
}

otError RadioSpinel::ClearSrcMatchShortEntry(uint16_t aShortAddress)
{
    otError error;

    SuccessOrExit(
        error = mSpinelBase.Remove(SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES, SPINEL_DATATYPE_UINT16_S, aShortAddress));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    for (int i = 0; i < mSrcMatchShortEntryCount; ++i)
    {
        if (mSrcMatchShortEntries[i] == aShortAddress)
        {
            mSrcMatchShortEntries[i] = mSrcMatchShortEntries[mSrcMatchShortEntryCount - 1];
            --mSrcMatchShortEntryCount;
            break;
        }
    }
#endif

exit:
    return error;
}

otError RadioSpinel::ClearSrcMatchShortEntries(void)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES, nullptr));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mSrcMatchShortEntryCount = 0;
#endif

exit:
    return error;
}

otError RadioSpinel::AddSrcMatchExtEntry(const otExtAddress &aExtAddress)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Insert(SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES, SPINEL_DATATYPE_EUI64_S,
                                             aExtAddress.m8));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    assert(mSrcMatchExtEntryCount < OPENTHREAD_CONFIG_MLE_MAX_CHILDREN);

    for (int i = 0; i < mSrcMatchExtEntryCount; ++i)
    {
        if (memcmp(aExtAddress.m8, mSrcMatchExtEntries[i].m8, OT_EXT_ADDRESS_SIZE) == 0)
        {
            ExitNow();
        }
    }
    mSrcMatchExtEntries[mSrcMatchExtEntryCount] = aExtAddress;
    ++mSrcMatchExtEntryCount;
#endif

exit:
    return error;
}

otError RadioSpinel::ClearSrcMatchExtEntry(const otExtAddress &aExtAddress)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Remove(SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES, SPINEL_DATATYPE_EUI64_S,
                                             aExtAddress.m8));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    for (int i = 0; i < mSrcMatchExtEntryCount; ++i)
    {
        if (memcmp(mSrcMatchExtEntries[i].m8, aExtAddress.m8, OT_EXT_ADDRESS_SIZE) == 0)
        {
            mSrcMatchExtEntries[i] = mSrcMatchExtEntries[mSrcMatchExtEntryCount - 1];
            --mSrcMatchExtEntryCount;
            break;
        }
    }
#endif

exit:
    return error;
}

otError RadioSpinel::ClearSrcMatchExtEntries(void)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES, nullptr));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mSrcMatchExtEntryCount = 0;
#endif

exit:
    return error;
}

otError RadioSpinel::EnergyScan(uint8_t aScanChannel, uint16_t aScanDuration)
{
    otError error;

    VerifyOrExit(sRadioCaps & OT_RADIO_CAPS_ENERGY_SCAN, error = OT_ERROR_NOT_CAPABLE);

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mScanChannel    = aScanChannel;
    mScanDuration   = aScanDuration;
    mEnergyScanning = true;
#endif

    SuccessOrExit(
        error = mSpinelBase.Set(SPINEL_PROP_MAC_SCAN_MASK, SPINEL_DATATYPE_DATA_S, &aScanChannel, sizeof(uint8_t)));
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_SCAN_PERIOD, SPINEL_DATATYPE_UINT16_S, aScanDuration));
    SuccessOrExit(error =
                      mSpinelBase.Set(SPINEL_PROP_MAC_SCAN_STATE, SPINEL_DATATYPE_UINT8_S, SPINEL_SCAN_STATE_ENERGY));

    mChannel = aScanChannel;

exit:
    return error;
}

otError RadioSpinel::Transmit(otRadioFrame &aFrame)
{
    otError error = OT_ERROR_INVALID_STATE;

    VerifyOrExit(mState == kStateReceive || (mState == kStateSleep && (sRadioCaps & OT_RADIO_CAPS_SLEEP_TO_TX)));

    mTransmitFrame = &aFrame;

    // `otPlatRadioTxStarted()` is triggered immediately for now, which may be earlier than real started time.
    mCallbacks.mTxStarted(mInstance, mTransmitFrame);

    error = mSpinelBase.Request(
        SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_STREAM_RAW,
        SPINEL_DATATYPE_DATA_WLEN_S                                      // Frame data
            SPINEL_DATATYPE_UINT8_S                                      // Channel
                SPINEL_DATATYPE_UINT8_S                                  // MaxCsmaBackoffs
                    SPINEL_DATATYPE_UINT8_S                              // MaxFrameRetries
                        SPINEL_DATATYPE_BOOL_S                           // CsmaCaEnabled
                            SPINEL_DATATYPE_BOOL_S                       // IsHeaderUpdated
                                SPINEL_DATATYPE_BOOL_S                   // IsARetx
                                    SPINEL_DATATYPE_BOOL_S               // IsSecurityProcessed
                                        SPINEL_DATATYPE_UINT32_S         // TxDelay
                                            SPINEL_DATATYPE_UINT32_S     // TxDelayBaseTime
                                                SPINEL_DATATYPE_UINT8_S, // RxChannelAfterTxDone
        mTransmitFrame->mPsdu, mTransmitFrame->mLength, mTransmitFrame->mChannel,
        mTransmitFrame->mInfo.mTxInfo.mMaxCsmaBackoffs, mTransmitFrame->mInfo.mTxInfo.mMaxFrameRetries,
        mTransmitFrame->mInfo.mTxInfo.mCsmaCaEnabled, mTransmitFrame->mInfo.mTxInfo.mIsHeaderUpdated,
        mTransmitFrame->mInfo.mTxInfo.mIsARetx, mTransmitFrame->mInfo.mTxInfo.mIsSecurityProcessed,
        mTransmitFrame->mInfo.mTxInfo.mTxDelay, mTransmitFrame->mInfo.mTxInfo.mTxDelayBaseTime,
        mTransmitFrame->mInfo.mTxInfo.mRxChannelAfterTxDone);

    if (error == OT_ERROR_NONE)
    {
        // Waiting for `TransmitDone` event.
        mState        = kStateTransmitting;
        mTxRadioEndUs = otPlatTimeGet() + kTxWaitUs;
        mChannel      = mTransmitFrame->mChannel;
    }

    LogWarn("!!! Transmit, error:%d", error);
exit:
    return error;
}

otError RadioSpinel::Receive(uint8_t aChannel)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mState != kStateDisabled, error = OT_ERROR_INVALID_STATE);

    if (mChannel != aChannel)
    {
        error = mSpinelBase.Set(SPINEL_PROP_PHY_CHAN, SPINEL_DATATYPE_UINT8_S, aChannel);
        SuccessOrExit(error);
        mChannel = aChannel;
    }

    if (mState == kStateSleep)
    {
        error = mSpinelBase.Set(SPINEL_PROP_MAC_RAW_STREAM_ENABLED, SPINEL_DATATYPE_BOOL_S, true);
        SuccessOrExit(error);
    }

    mSpinelBase.ResetStreamRawTid();

    mState = kStateReceive;

exit:
    return error;
}

otError RadioSpinel::Sleep(void)
{
    otError error = OT_ERROR_NONE;

    switch (mState)
    {
    case kStateReceive:
        error = mSpinelBase.Set(SPINEL_PROP_MAC_RAW_STREAM_ENABLED, SPINEL_DATATYPE_BOOL_S, false);
        SuccessOrExit(error);

        mState = kStateSleep;
        break;

    case kStateSleep:
        break;

    default:
        error = OT_ERROR_INVALID_STATE;
        break;
    }

exit:
    return error;
}

otError RadioSpinel::Enable(otInstance *aInstance)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(!IsEnabled());

    mInstance = aInstance;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, true));
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_15_4_PANID, SPINEL_DATATYPE_UINT16_S, mPanId));
    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_MAC_15_4_SADDR, SPINEL_DATATYPE_UINT16_S, mShortAddress));
    SuccessOrExit(error = mSpinelBase.Get(SPINEL_PROP_PHY_RX_SENSITIVITY, SPINEL_DATATYPE_INT8_S, &mRxSensitivity));

    mState = kStateSleep;

exit:
    if (error != OT_ERROR_NONE)
    {
        LogWarn("RadioSpinel enable: %s", otThreadErrorToString(error));
        error = OT_ERROR_FAILED;
    }

    return error;
}

otError RadioSpinel::Disable(void)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(IsEnabled());
    VerifyOrExit(mState == kStateSleep, error = OT_ERROR_INVALID_STATE);

    SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, false));
    mState    = kStateDisabled;
    mInstance = nullptr;

exit:
    return error;
}

#if OPENTHREAD_CONFIG_DIAG_ENABLE
otError RadioSpinel::PlatDiagProcess(const char *aString, char *aOutput, size_t aOutputMaxLen)
{
    otError error;

    mDiagOutput       = aOutput;
    mDiagOutputMaxLen = aOutputMaxLen;

    error = mSpinelBase.Set(SPINEL_PROP_NEST_STREAM_MFG, SPINEL_DATATYPE_UTF8_S, aString);

    mDiagOutput       = nullptr;
    mDiagOutputMaxLen = 0;

    return error;
}
#endif

uint32_t RadioSpinel::GetRadioChannelMask(bool aPreferred)
{
    uint8_t        maskBuffer[kChannelMaskBufferSize];
    otError        error       = OT_ERROR_NONE;
    uint32_t       channelMask = 0;
    const uint8_t *maskData    = maskBuffer;
    spinel_size_t  maskLength  = sizeof(maskBuffer);

    SuccessOrDie(mSpinelBase.Get(aPreferred ? SPINEL_PROP_PHY_CHAN_PREFERRED : SPINEL_PROP_PHY_CHAN_SUPPORTED,
                                 SPINEL_DATATYPE_DATA_S, maskBuffer, &maskLength));

    while (maskLength > 0)
    {
        uint8_t        channel;
        spinel_ssize_t unpacked;

        unpacked = spinel_datatype_unpack(maskData, maskLength, SPINEL_DATATYPE_UINT8_S, &channel);
        VerifyOrExit(unpacked > 0, error = OT_ERROR_FAILED);
        VerifyOrExit(channel < kChannelMaskBufferSize, error = OT_ERROR_PARSE);
        channelMask |= (1UL << channel);

        maskData += unpacked;
        maskLength -= static_cast<spinel_size_t>(unpacked);
    }

    channelMask &= mMaxPowerTable.GetSupportedChannelMask();

exit:
    UpdateParseErrorCount(error);
    LogIfFail("Get radio channel mask failed", error);
    return channelMask;
}

#if OPENTHREAD_CONFIG_PLATFORM_KEY_REFERENCES_ENABLE

otError RadioSpinel::ReadMacKey(const otMacKeyMaterial &aKeyMaterial, otMacKey &aKey)
{
    size_t  keySize;
    otError error = otPlatCryptoExportKey(aKeyMaterial.mKeyMaterial.mKeyRef, aKey.m8, sizeof(aKey), &keySize);

    SuccessOrExit(error);
    VerifyOrExit(keySize == sizeof(otMacKey), error = OT_ERROR_FAILED);

exit:
    return error;
}

otError RadioSpinel::SetMacKey(uint8_t                 aKeyIdMode,
                               uint8_t                 aKeyId,
                               const otMacKeyMaterial *aPrevKey,
                               const otMacKeyMaterial *aCurrKey,
                               const otMacKeyMaterial *aNextKey)
{
    otError  error;
    otMacKey prevKey;
    otMacKey currKey;
    otMacKey nextKey;

    SuccessOrExit(error = ReadMacKey(*aPrevKey, prevKey));
    SuccessOrExit(error = ReadMacKey(*aCurrKey, currKey));
    SuccessOrExit(error = ReadMacKey(*aNextKey, nextKey));
    error = SetMacKey(aKeyIdMode, aKeyId, prevKey, currKey, nextKey);

exit:
    return error;
}

#else

otError RadioSpinel::SetMacKey(uint8_t aKeyIdMode,
                               uint8_t aKeyId,
                               const otMacKeyMaterial *aPrevKey,
                               const otMacKeyMaterial *aCurrKey,
                               const otMacKeyMaterial *aNextKey)
{
    return SetMacKey(aKeyIdMode, aKeyId, aPrevKey->mKeyMaterial.mKey, aCurrKey->mKeyMaterial.mKey,
                     aNextKey->mKeyMaterial.mKey);
}

#endif // OPENTHREAD_CONFIG_PLATFORM_KEY_REFERENCES_ENABLE

otError RadioSpinel::SetMacKey(uint8_t         aKeyIdMode,
                               uint8_t         aKeyId,
                               const otMacKey &aPrevKey,
                               const otMacKey &aCurrKey,
                               const otMacKey &aNextKey)
{
    otError error;

    SuccessOrExit(error = mSpinelBase.Set(SPINEL_PROP_RCP_MAC_KEY,
                                          SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_DATA_WLEN_S
                                              SPINEL_DATATYPE_DATA_WLEN_S SPINEL_DATATYPE_DATA_WLEN_S,
                                          aKeyIdMode, aKeyId, aPrevKey.m8, sizeof(aPrevKey), aCurrKey.m8,
                                          sizeof(aCurrKey), aNextKey.m8, sizeof(aNextKey)));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mKeyIdMode = aKeyIdMode;
    mKeyId     = aKeyId;

    mPrevKey = aPrevKey;
    mCurrKey = aCurrKey;
    mNextKey = aNextKey;

    mMacKeySet = true;
#endif

exit:
    return error;
}

otError RadioSpinel::SetMacFrameCounter(uint32_t aMacFrameCounter, bool aSetIfLarger)
{
    otError error;

    SuccessOrExit(error =
                      mSpinelBase.Set(SPINEL_PROP_RCP_MAC_FRAME_COUNTER,
                                      SPINEL_DATATYPE_UINT32_S SPINEL_DATATYPE_BOOL_S, aMacFrameCounter, aSetIfLarger));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mMacFrameCounterSet = true;
    mMacFrameCounter    = aMacFrameCounter;
#endif

exit:
    return error;
}

otError RadioSpinel::SetRadioRegion(uint16_t aRegionCode)
{
    otError error;

    error = mSpinelBase.Set(SPINEL_PROP_PHY_REGION_CODE, SPINEL_DATATYPE_UINT16_S, aRegionCode);

    if (error == OT_ERROR_NONE)
    {
        LogNote("Set region code \"%c%c\" successfully", static_cast<char>(aRegionCode >> 8),
                static_cast<char>(aRegionCode));
    }
    else
    {
        LogWarn("Failed to set region code \"%c%c\": %s", static_cast<char>(aRegionCode >> 8),
                static_cast<char>(aRegionCode), otThreadErrorToString(error));
    }

    return error;
}

otError RadioSpinel::GetRadioRegion(uint16_t *aRegionCode)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aRegionCode != nullptr, error = OT_ERROR_INVALID_ARGS);
    error = mSpinelBase.Get(SPINEL_PROP_PHY_REGION_CODE, SPINEL_DATATYPE_UINT16_S, aRegionCode);

exit:
    return error;
}

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
otError RadioSpinel::ConfigureEnhAckProbing(otLinkMetrics         aLinkMetrics,
                                            const otShortAddress &aShortAddress,
                                            const otExtAddress   &aExtAddress)
{
    otError error = OT_ERROR_NONE;
    uint8_t flags = 0;

    if (aLinkMetrics.mPduCount)
    {
        flags |= SPINEL_THREAD_LINK_METRIC_PDU_COUNT;
    }

    if (aLinkMetrics.mLqi)
    {
        flags |= SPINEL_THREAD_LINK_METRIC_LQI;
    }

    if (aLinkMetrics.mLinkMargin)
    {
        flags |= SPINEL_THREAD_LINK_METRIC_LINK_MARGIN;
    }

    if (aLinkMetrics.mRssi)
    {
        flags |= SPINEL_THREAD_LINK_METRIC_RSSI;
    }

    error = mSpinelBase.Set(SPINEL_PROP_RCP_ENH_ACK_PROBING,
                            SPINEL_DATATYPE_UINT16_S SPINEL_DATATYPE_EUI64_S SPINEL_DATATYPE_UINT8_S, aShortAddress,
                            aExtAddress.m8, flags);

    return error;
}
#endif // OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE || OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
uint8_t RadioSpinel::GetCslAccuracy(void)
{
    uint8_t accuracy = UINT8_MAX;
    otError error    = mSpinelBase.Get(SPINEL_PROP_RCP_CSL_ACCURACY, SPINEL_DATATYPE_UINT8_S, &accuracy);

    LogIfFail("Get CSL Accuracy failed", error);
    return accuracy;
}
#endif

#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
uint8_t RadioSpinel::GetCslUncertainty(void)
{
    uint8_t uncertainty = UINT8_MAX;
    otError error       = mSpinelBase.Get(SPINEL_PROP_RCP_CSL_UNCERTAINTY, SPINEL_DATATYPE_UINT8_S, &uncertainty);

    LogIfFail("Get CSL Uncertainty failed", error);
    return uncertainty;
}
#endif

#if OPENTHREAD_CONFIG_PLATFORM_POWER_CALIBRATION_ENABLE
otError RadioSpinel::AddCalibratedPower(uint8_t        aChannel,
                                        int16_t        aActualPower,
                                        const uint8_t *aRawPowerSetting,
                                        uint16_t       aRawPowerSettingLength)
{
    otError error;

    assert(aRawPowerSetting != nullptr);
    SuccessOrExit(error =
                      mSpinelBase.Insert(SPINEL_PROP_PHY_CALIBRATED_POWER,
                                         SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_INT16_S SPINEL_DATATYPE_DATA_WLEN_S,
                                         aChannel, aActualPower, aRawPowerSetting, aRawPowerSettingLength));

exit:
    return error;
}

otError RadioSpinel::ClearCalibratedPowers(void) { return mSpinelBase.Set(SPINEL_PROP_PHY_CALIBRATED_POWER, nullptr); }

otError RadioSpinel::SetChannelTargetPower(uint8_t aChannel, int16_t aTargetPower)
{
    otError error = OT_ERROR_NONE;
    VerifyOrExit(aChannel >= Radio::kChannelMin && aChannel <= Radio::kChannelMax, error = OT_ERROR_INVALID_ARGS);
    error = mSpinelBase.Set(SPINEL_PROP_PHY_CHAN_TARGET_POWER, SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_INT16_S,
                            aChannel, aTargetPower);

exit:
    return error;
}
#endif // OPENTHREAD_CONFIG_PLATFORM_POWER_CALIBRATION_ENABLE

uint64_t RadioSpinel::GetNow(void) { return (mIsTimeSynced) ? (otPlatTimeGet() + mRadioTimeOffset) : UINT64_MAX; }

otError RadioSpinel::SetChannelMaxTransmitPower(uint8_t aChannel, int8_t aMaxPower)
{
    otError error = OT_ERROR_NONE;
    VerifyOrExit(aChannel >= Radio::kChannelMin && aChannel <= Radio::kChannelMax, error = OT_ERROR_INVALID_ARGS);
    mMaxPowerTable.SetTransmitPower(aChannel, aMaxPower);
    error = mSpinelBase.Set(SPINEL_PROP_PHY_CHAN_MAX_POWER, SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_INT8_S, aChannel,
                            aMaxPower);

exit:
    return error;
}

bool RadioSpinel::IsRcp(bool &aSupportsRcpApiVersion, bool &aSupportsRcpMinHostApiVersion)
{
    uint8_t        capsBuffer[kCapsBufferSize];
    const uint8_t *capsData         = capsBuffer;
    spinel_size_t  capsLength       = sizeof(capsBuffer);
    bool           supportsRawRadio = false;
    bool           isRcp            = false;

    aSupportsRcpApiVersion        = false;
    aSupportsRcpMinHostApiVersion = false;

    SuccessOrDie(mSpinelBase.Get(SPINEL_PROP_CAPS, SPINEL_DATATYPE_DATA_S, capsBuffer, &capsLength));

    while (capsLength > 0)
    {
        unsigned int   capability;
        spinel_ssize_t unpacked;

        unpacked = spinel_datatype_unpack(capsData, capsLength, SPINEL_DATATYPE_UINT_PACKED_S, &capability);
        VerifyOrDie(unpacked > 0, OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);

        if (capability == SPINEL_CAP_MAC_RAW)
        {
            supportsRawRadio = true;
        }

        if (capability == SPINEL_CAP_CONFIG_RADIO)
        {
            isRcp = true;
        }

        if (capability == SPINEL_CAP_OPENTHREAD_LOG_METADATA)
        {
            sSupportsLogStream = true;
        }

        if (capability == SPINEL_CAP_RCP_API_VERSION)
        {
            aSupportsRcpApiVersion = true;
        }

        if (capability == SPINEL_CAP_RCP_RESET_TO_BOOTLOADER)
        {
            sSupportsResetToBootloader = true;
        }

        if (capability == SPINEL_PROP_RCP_MIN_HOST_API_VERSION)
        {
            aSupportsRcpMinHostApiVersion = true;
        }

        capsData += unpacked;
        capsLength -= static_cast<spinel_size_t>(unpacked);
    }

    if (!supportsRawRadio && isRcp)
    {
        LogCrit("RCP capability list does not include support for radio/raw mode");
        DieNow(OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);
    }

    return isRcp;
}

otError RadioSpinel::CheckRadioCapabilities(void)
{
    const otRadioCaps kRequiredRadioCaps =
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
        OT_RADIO_CAPS_TRANSMIT_SEC | OT_RADIO_CAPS_TRANSMIT_TIMING |
#endif
        OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_TRANSMIT_RETRIES | OT_RADIO_CAPS_CSMA_BACKOFF;

    otError      error = OT_ERROR_NONE;
    unsigned int radioCaps;

    SuccessOrExit(error = mSpinelBase.Get(SPINEL_PROP_RADIO_CAPS, SPINEL_DATATYPE_UINT_PACKED_S, &radioCaps));
    sRadioCaps = static_cast<otRadioCaps>(radioCaps);

    if ((sRadioCaps & kRequiredRadioCaps) != kRequiredRadioCaps)
    {
        otRadioCaps missingCaps = (sRadioCaps & kRequiredRadioCaps) ^ kRequiredRadioCaps;

        // missingCaps may be an unused variable when LogCrit is blank
        // avoid compiler warning in that case
        OT_UNUSED_VARIABLE(missingCaps);

        LogCrit("RCP is missing required capabilities: %s%s%s%s%s",
                (missingCaps & OT_RADIO_CAPS_ACK_TIMEOUT) ? "ack-timeout " : "",
                (missingCaps & OT_RADIO_CAPS_TRANSMIT_RETRIES) ? "tx-retries " : "",
                (missingCaps & OT_RADIO_CAPS_CSMA_BACKOFF) ? "CSMA-backoff " : "",
                (missingCaps & OT_RADIO_CAPS_TRANSMIT_SEC) ? "tx-security " : "",
                (missingCaps & OT_RADIO_CAPS_TRANSMIT_TIMING) ? "tx-timing " : "");

        DieNow(OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);
    }

exit:
    return error;
}

otError RadioSpinel::CheckRcpApiVersion(bool aSupportsRcpApiVersion, bool aSupportsRcpMinHostApiVersion)
{
    otError error = OT_ERROR_NONE;

    static_assert(SPINEL_MIN_HOST_SUPPORTED_RCP_API_VERSION <= SPINEL_RCP_API_VERSION,
                  "MIN_HOST_SUPPORTED_RCP_API_VERSION must be smaller than or equal to RCP_API_VERSION");

    if (aSupportsRcpApiVersion)
    {
        // Make sure RCP is not too old and its version is within the
        // range host supports.

        unsigned int rcpApiVersion;

        SuccessOrExit(error =
                          mSpinelBase.Get(SPINEL_PROP_RCP_API_VERSION, SPINEL_DATATYPE_UINT_PACKED_S, &rcpApiVersion));

        if (rcpApiVersion < SPINEL_MIN_HOST_SUPPORTED_RCP_API_VERSION)
        {
            LogCrit("RCP and host are using incompatible API versions");
            LogCrit("RCP API Version %u is older than min required by host %u", rcpApiVersion,
                    SPINEL_MIN_HOST_SUPPORTED_RCP_API_VERSION);
            DieNow(OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);
        }
    }

    if (aSupportsRcpMinHostApiVersion)
    {
        // Check with RCP about min host API version it can work with,
        // and make sure on host side our version is within the supported
        // range.

        unsigned int minHostRcpApiVersion;

        SuccessOrExit(error = mSpinelBase.Get(SPINEL_PROP_RCP_MIN_HOST_API_VERSION, SPINEL_DATATYPE_UINT_PACKED_S,
                                              &minHostRcpApiVersion));

        if (SPINEL_RCP_API_VERSION < minHostRcpApiVersion)
        {
            LogCrit("RCP and host are using incompatible API versions");
            LogCrit("RCP requires min host API version %u but host is older and at version %u", minHostRcpApiVersion,
                    SPINEL_RCP_API_VERSION);
            DieNow(OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);
        }
    }

exit:
    return error;
}

void RadioSpinel::ProcessRadioStateMachine(void)
{
    if (mState == kStateTransmitDone)
    {
        mState        = kStateReceive;
        mTxRadioEndUs = UINT64_MAX;

        TransmitDone(mTransmitFrame, (mAckRadioFrame.mLength != 0) ? &mAckRadioFrame : nullptr, mTxError);
    }
    else if (mState == kStateTransmitting && otPlatTimeGet() >= mTxRadioEndUs)
    {
        // Frame has been successfully passed to radio, but no `TransmitDone` event received within kTxWaitUs.
        LogWarn("radio tx timeout");
        HandleCpTimeout();
    }
}

void RadioSpinel::Process(const void *aContext)
{
    OT_UNUSED_VARIABLE(aContext);

    ProcessRadioStateMachine();
    RecoverFromCpFailure();
    CalcRcpTimeOffset();
}

void RadioSpinel::RadioReceive(void)
{
    if (!mIsPromiscuous)
    {
        switch (mState)
        {
        case kStateDisabled:
        case kStateSleep:
            ExitNow();

        case kStateReceive:
        case kStateTransmitting:
        case kStateTransmitDone:
            break;
        }
    }

#if OPENTHREAD_CONFIG_DIAG_ENABLE
    if (otPlatDiagModeGet())
    {
        mCallbacks.mDiagReceiveDone(mInstance, &mRxRadioFrame, OT_ERROR_NONE);
    }
    else
#endif
    {
        mCallbacks.mReceiveDone(mInstance, &mRxRadioFrame, OT_ERROR_NONE);
    }
exit:
    return;
}

void RadioSpinel::TransmitDone(otRadioFrame *aFrame, otRadioFrame *aAckFrame, otError aError)
{
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    if (otPlatDiagModeGet())
    {
        mCallbacks.mDiagTransmitDone(mInstance, aFrame, aError);
    }
    else
#endif
    {
        mCallbacks.mTransmitDone(mInstance, aFrame, aAckFrame, aError);
    }
}

void RadioSpinel::CalcRcpTimeOffset(void)
{
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    otError        error = OT_ERROR_NONE;
    uint64_t       localTxTimestamp;
    uint64_t       localRxTimestamp;
    uint64_t       remoteTimestamp = 0;
    uint8_t        buffer[sizeof(remoteTimestamp)];
    spinel_ssize_t packed;

    /*
     * Use a modified Network Time Protocol(NTP) to calculate the time offset
     * Assume the time offset is D so that local can calculate remote time with,
     *         T' = T + D
     * Where T is the local time and T' is the remote time.
     * The time offset is calculated using timestamp measured at local and remote.
     *
     *              T0  P    P T2
     *  local time --+----+----+--->
     *                \   |   ^
     *              get\  |  /is
     *                  v | /
     * remote time -------+--------->
     *                    T1'
     *
     * Based on the assumptions,
     * 1. If the propagation time(P) from local to remote and from remote to local are same.
     * 2. Both the host and RCP can accurately measure the time they send or receive a message.
     * The degree to which these assumptions hold true determines the accuracy of the offset.
     * Then,
     *         T1' = T0 + P + D and T1' = T2 - P + D
     * Time offset can be calculated with,
     *         D = T1' - ((T0 + T2)/ 2)
     */

    VerifyOrExit(!mIsTimeSynced || (otPlatTimeGet() >= GetNextRadioTimeRecalcStart()));

    LogDebg("Trying to get RCP time offset");

    packed = spinel_datatype_pack(buffer, sizeof(buffer), SPINEL_DATATYPE_UINT64_S, remoteTimestamp);
    VerifyOrExit(packed > 0 && static_cast<size_t>(packed) <= sizeof(buffer), error = OT_ERROR_NO_BUFS);

    localTxTimestamp = otPlatTimeGet();

    // Dummy timestamp payload to make request length same as response
    error = mSpinelBase.GetWithParam(SPINEL_PROP_RCP_TIMESTAMP, buffer, static_cast<spinel_size_t>(packed),
                                     SPINEL_DATATYPE_UINT64_S, &remoteTimestamp);

    localRxTimestamp = otPlatTimeGet();

    VerifyOrExit(error == OT_ERROR_NONE, mRadioTimeRecalcStart = localRxTimestamp);

    mRadioTimeOffset      = (remoteTimestamp - ((localRxTimestamp / 2) + (localTxTimestamp / 2)));
    mIsTimeSynced         = true;
    mRadioTimeRecalcStart = localRxTimestamp + OPENTHREAD_SPINEL_CONFIG_RCP_TIME_SYNC_INTERVAL;

exit:
    LogIfFail("Error calculating RCP time offset: %s", error);
#endif // OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
}

void RadioSpinel::HandleRcpUnexpectedReset(spinel_status_t aStatus)
{
    OT_UNUSED_VARIABLE(aStatus);

    mRadioSpinelMetrics.mRcpUnexpectedResetCount++;
    LogCrit("Unexpected RCP reset: %s", spinel_status_to_cstr(aStatus));

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    mRcpFailure = kRcpFailureUnexpectedReset;
#elif OPENTHREAD_SPINEL_CONFIG_ABORT_ON_UNEXPECTED_RCP_RESET_ENABLE
    abort();
#else
    DieNow(OT_EXIT_RADIO_SPINEL_RESET);
#endif
}

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
void RadioSpinel::RestoreProperties(void)
{
    SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_MAC_15_4_PANID, SPINEL_DATATYPE_UINT16_S, mPanId));
    SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_MAC_15_4_SADDR, SPINEL_DATATYPE_UINT16_S, mShortAddress));
    SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_MAC_15_4_LADDR, SPINEL_DATATYPE_EUI64_S, mExtendedAddress.m8));
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
    // In case multiple PANs are running, don't force RCP to change channel.
    IgnoreError(mSpinelBase.Set(SPINEL_PROP_PHY_CHAN, SPINEL_DATATYPE_UINT8_S, mChannel));
#else
    SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_PHY_CHAN, SPINEL_DATATYPE_UINT8_S, mChannel));
#endif

    if (mMacKeySet)
    {
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_RCP_MAC_KEY,
                                     SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_DATA_WLEN_S
                                         SPINEL_DATATYPE_DATA_WLEN_S SPINEL_DATATYPE_DATA_WLEN_S,
                                     mKeyIdMode, mKeyId, mPrevKey.m8, sizeof(otMacKey), mCurrKey.m8, sizeof(otMacKey),
                                     mNextKey.m8, sizeof(otMacKey)));
    }

    if (mMacFrameCounterSet)
    {
        // There is a chance that radio/RCP has used some counters after `mMacFrameCounter` (for enh ack) and they
        // are in queue to be sent to host (not yet processed by host RadioSpinel). Here we add some guard jump
        // when we restore the frame counter.
        // Consider the worst case: the radio/RCP continuously receives the shortest data frame and replies with the
        // shortest enhanced ACK. The radio/RCP consumes at most 992 frame counters during the timeout time.
        // The frame counter guard is set to 1000 which should ensure that the restored frame counter is unused.
        //
        // DataFrame: 6(PhyHeader) + 2(Fcf) + 1(Seq) + 6(AddrInfo) + 6(SecHeader) + 1(Payload) + 4(Mic) + 2(Fcs) = 28
        // AckFrame : 6(PhyHeader) + 2(Fcf) + 1(Seq) + 6(AddrInfo) + 6(SecHeader) + 2(Ie) + 4(Mic) + 2(Fcs) = 29
        // CounterGuard: 2000ms(Timeout) / [(28bytes(Data) + 29bytes(Ack)) * 32us/byte + 192us(Ifs)] = 992
        static constexpr uint16_t kFrameCounterGuard = 1000;

        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_RCP_MAC_FRAME_COUNTER, SPINEL_DATATYPE_UINT32_S,
                                     mMacFrameCounter + kFrameCounterGuard));
    }

    for (int i = 0; i < mSrcMatchShortEntryCount; ++i)
    {
        SuccessOrDie(mSpinelBase.Insert(SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES, SPINEL_DATATYPE_UINT16_S,
                                        mSrcMatchShortEntries[i]));
    }

    for (int i = 0; i < mSrcMatchExtEntryCount; ++i)
    {
        SuccessOrDie(mSpinelBase.Insert(SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES, SPINEL_DATATYPE_EUI64_S,
                                        mSrcMatchExtEntries[i].m8));
    }

    if (mCcaEnergyDetectThresholdSet)
    {
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_PHY_CCA_THRESHOLD, SPINEL_DATATYPE_INT8_S, mCcaEnergyDetectThreshold));
    }

    if (mTransmitPowerSet)
    {
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_PHY_TX_POWER, SPINEL_DATATYPE_INT8_S, mTransmitPower));
    }

    if (mCoexEnabledSet)
    {
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_RADIO_COEX_ENABLE, SPINEL_DATATYPE_BOOL_S, mCoexEnabled));
    }

    if (mFemLnaGainSet)
    {
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_PHY_FEM_LNA_GAIN, SPINEL_DATATYPE_INT8_S, mFemLnaGain));
    }

#if OPENTHREAD_POSIX_CONFIG_MAX_POWER_TABLE_ENABLE
    for (uint8_t channel = Radio::kChannelMin; channel <= Radio::kChannelMax; channel++)
    {
        int8_t power = mMaxPowerTable.GetTransmitPower(channel);

        if (power != OT_RADIO_POWER_INVALID)
        {
            // Some old RCPs doesn't support max transmit power
            otError error = SetChannelMaxTransmitPower(channel, power);

            if (error != OT_ERROR_NONE && error != OT_ERROR_NOT_FOUND)
            {
                DieNow(OT_EXIT_FAILURE);
            }
        }
    }
#endif // OPENTHREAD_POSIX_CONFIG_MAX_POWER_TABLE_ENABLE

    if ((sRadioCaps & OT_RADIO_CAPS_RX_ON_WHEN_IDLE) != 0)
    {
        SuccessOrDie(mSpinelBase.Set(SPINEL_PROP_MAC_RX_ON_WHEN_IDLE_MODE, SPINEL_DATATYPE_BOOL_S, mRxOnWhenIdle));
    }

    CalcRcpTimeOffset();
}
#endif

} // namespace Spinel
} // namespace ot
