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

#include "spinel_base.hpp"

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>

#include <openthread/logging.h>
#include <openthread/platform/time.h>

#include "common/code_utils.hpp"
#include "common/num_utils.hpp"
#include "lib/platform/exit_code.h"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_interface.hpp"

namespace ot {
namespace Spinel {

SpinelBase::SpinelBase(void)
    : mInstance(nullptr)
    , mSpinelInterface(nullptr)
    , mCmdTidsInUse(0)
    , mCmdNextTid(1)
    , mWaitingTid(0)
    , mWaitingKey(SPINEL_PROP_LAST_STATUS)
    , mPropertyFormat(nullptr)
    , mExpectedCommand(0)
    , mError(OT_ERROR_NONE)
    , mIsReady(false)
{
}

void SpinelBase::Init(SpinelInterface &aSpinelInterface, bool aReset, bool aSkipRcpCompatibilityCheck)
{
    otError error = OT_ERROR_NONE;
    (void)aReset;
    (void)aSkipRcpCompatibilityCheck;

    mSpinelInterface = &aSpinelInterface;
    SuccessOrDie(mSpinelInterface->Init(HandleReceivedFrame, this, mRxFrameBuffer));

    // TODO: General Reset
    //ResetRcp(aResetRadio);
    SuccessOrExit(error = CheckSpinelVersion());
    SuccessOrExit(error = Get(SPINEL_PROP_NCP_VERSION, SPINEL_DATATYPE_UTF8_S, mVersion, sizeof(mVersion)));

    // TODO: Check Capability and decide the mode

exit:
    SuccessOrDie(error);
}

otError SpinelBase::Get(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    otError error;

    assert(mWaitingTid == 0);

    {
        va_start(mPropertyArgs, aFormat);
        error = RequestWithPropertyFormatV(aFormat, SPINEL_CMD_PROP_VALUE_GET, aKey, nullptr, mPropertyArgs);
        va_end(mPropertyArgs);
    }

    return error;
}

otError SpinelBase::GetWithParam(spinel_prop_key_t aKey,
                                  const uint8_t    *aParam,
                                  spinel_size_t     aParamSize,
                                  const char       *aFormat,
                                  ...)
{
    otError error;

    assert(mWaitingTid == 0);

    {
        va_start(mPropertyArgs, aFormat);
        error = RequestWithPropertyFormat(aFormat, SPINEL_CMD_PROP_VALUE_GET, aKey, SPINEL_DATATYPE_DATA_S, aParam,
                                          aParamSize);
        va_end(mPropertyArgs);
    }

    return error;
}

otError SpinelBase::Set(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    otError error;

    assert(mWaitingTid == 0);

    {
        va_start(mPropertyArgs, aFormat);
        error = RequestWithExpectedCommandV(SPINEL_CMD_PROP_VALUE_IS, SPINEL_CMD_PROP_VALUE_SET, aKey, aFormat,
                                            mPropertyArgs);
        va_end(mPropertyArgs);
    }

    return error;
}

otError SpinelBase::Insert(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    otError error;

    assert(mWaitingTid == 0);

    {
        va_start(mPropertyArgs, aFormat);
        error = RequestWithExpectedCommandV(SPINEL_CMD_PROP_VALUE_INSERTED, SPINEL_CMD_PROP_VALUE_INSERT, aKey, aFormat,
                                            mPropertyArgs);
        va_end(mPropertyArgs);
    }

    return error;
}

otError SpinelBase::Remove(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    otError error;

    assert(mWaitingTid == 0);

    {
        va_start(mPropertyArgs, aFormat);
        error = RequestWithExpectedCommandV(SPINEL_CMD_PROP_VALUE_REMOVED, SPINEL_CMD_PROP_VALUE_REMOVE, aKey, aFormat,
                                            mPropertyArgs);
        va_end(mPropertyArgs);
    }

    return error;
}

void SpinelBase::Process(const void *aContext)
{
    if (mRxFrameBuffer.HasSavedFrame())
    {
        ProcessFrameQueue();
    }

    mSpinelInterface->Process(aContext);

    if (mRxFrameBuffer.HasSavedFrame())
    {
        ProcessFrameQueue();
    }
}

otError SpinelBase::SpinelStatusToOtError(spinel_status_t aStatus)
{
    otError ret;

    switch (aStatus)
    {
    case SPINEL_STATUS_OK:
        ret = OT_ERROR_NONE;
        break;

    case SPINEL_STATUS_FAILURE:
        ret = OT_ERROR_FAILED;
        break;

    case SPINEL_STATUS_DROPPED:
        ret = OT_ERROR_DROP;
        break;

    case SPINEL_STATUS_NOMEM:
        ret = OT_ERROR_NO_BUFS;
        break;

    case SPINEL_STATUS_BUSY:
        ret = OT_ERROR_BUSY;
        break;

    case SPINEL_STATUS_PARSE_ERROR:
        ret = OT_ERROR_PARSE;
        break;

    case SPINEL_STATUS_INVALID_ARGUMENT:
        ret = OT_ERROR_INVALID_ARGS;
        break;

    case SPINEL_STATUS_UNIMPLEMENTED:
        ret = OT_ERROR_NOT_IMPLEMENTED;
        break;

    case SPINEL_STATUS_INVALID_STATE:
        ret = OT_ERROR_INVALID_STATE;
        break;

    case SPINEL_STATUS_NO_ACK:
        ret = OT_ERROR_NO_ACK;
        break;

    case SPINEL_STATUS_CCA_FAILURE:
        ret = OT_ERROR_CHANNEL_ACCESS_FAILURE;
        break;

    case SPINEL_STATUS_ALREADY:
        ret = OT_ERROR_ALREADY;
        break;

    case SPINEL_STATUS_PROP_NOT_FOUND:
        ret = OT_ERROR_NOT_IMPLEMENTED;
        break;

    case SPINEL_STATUS_ITEM_NOT_FOUND:
        ret = OT_ERROR_NOT_FOUND;
        break;

    default:
        if (aStatus >= SPINEL_STATUS_STACK_NATIVE__BEGIN && aStatus <= SPINEL_STATUS_STACK_NATIVE__END)
        {
            ret = static_cast<otError>(aStatus - SPINEL_STATUS_STACK_NATIVE__BEGIN);
        }
        else
        {
            ret = OT_ERROR_FAILED;
        }
        break;
    }

    return ret;
}

void SpinelBase::HandleReceivedFrame(void *aContext) { static_cast<SpinelBase *>(aContext)->HandleReceivedFrame(); }

void SpinelBase::HandleReceivedFrame(void)
{
    otError        error = OT_ERROR_NONE;
    uint8_t        header;
    spinel_ssize_t unpacked;

    unpacked = spinel_datatype_unpack(mRxFrameBuffer.GetFrame(), mRxFrameBuffer.GetLength(), "C", &header);

    VerifyOrExit(unpacked > 0 && (header & SPINEL_HEADER_FLAG) == SPINEL_HEADER_FLAG &&
                     SPINEL_HEADER_GET_IID(header) == 0,
                 error = OT_ERROR_PARSE);

    if (SPINEL_HEADER_GET_TID(header) == 0)
    {
        HandleNotification(mRxFrameBuffer);
    }
    else
    {
        HandleResponse(mRxFrameBuffer.GetFrame(), mRxFrameBuffer.GetLength());
        mRxFrameBuffer.DiscardFrame();
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        mRxFrameBuffer.DiscardFrame();
        LogWarn("Error handling hdlc frame: %s", otThreadErrorToString(error));
    }
}

otError SpinelBase::CheckSpinelVersion(void)
{
    otError      error = OT_ERROR_NONE;
    unsigned int versionMajor;
    unsigned int versionMinor;

    SuccessOrExit(error =
                      Get(SPINEL_PROP_PROTOCOL_VERSION, (SPINEL_DATATYPE_UINT_PACKED_S SPINEL_DATATYPE_UINT_PACKED_S),
                          &versionMajor, &versionMinor));

    if ((versionMajor != SPINEL_PROTOCOL_VERSION_THREAD_MAJOR) ||
        (versionMinor != SPINEL_PROTOCOL_VERSION_THREAD_MINOR))
    {
        LogCrit("Spinel version mismatch - Posix:%d.%d, Co-Processor:%d.%d", SPINEL_PROTOCOL_VERSION_THREAD_MAJOR,
                SPINEL_PROTOCOL_VERSION_THREAD_MINOR, versionMajor, versionMinor);
        DieNow(OT_EXIT_RADIO_SPINEL_INCOMPATIBLE);
    }

exit:
    return error;
}

otError SpinelBase::WaitResponse(bool aHandleRcpTimeout)
{
    uint64_t end = otPlatTimeGet() + kMaxWaitTime * kUsPerMs;

    LogDebg("Wait response: tid=%u key=%lu", mWaitingTid, ToUlong(mWaitingKey));

    do
    {
        uint64_t now;

        now = otPlatTimeGet();
        if ((end <= now) || (mSpinelInterface->WaitForFrame(end - now) != OT_ERROR_NONE))
        {
            LogWarn("Wait for response timeout");
            if (aHandleRcpTimeout)
            {
                HandleTimeout();
            }
            ExitNow(mError = OT_ERROR_RESPONSE_TIMEOUT);
        }
    } while (mWaitingTid || !mIsReady);

    LogIfFail("Error waiting response", mError);
    // This indicates end of waiting response.
    mWaitingKey = SPINEL_PROP_LAST_STATUS;

exit:
    return mError;
}

otError SpinelBase::SendCommand(uint32_t          aCommand,
                                 spinel_prop_key_t aKey,
                                 spinel_tid_t      tid,
                                 const char       *aFormat,
                                 va_list           args)
{
    otError        error = OT_ERROR_NONE;
    uint8_t        buffer[kMaxSpinelFrame];
    spinel_ssize_t packed;
    uint16_t       offset;

    // Pack the header, command and key
    packed = spinel_datatype_pack(buffer, sizeof(buffer), "Cii", SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0 | tid,
                                  aCommand, aKey);

    VerifyOrExit(packed > 0 && static_cast<size_t>(packed) <= sizeof(buffer), error = OT_ERROR_NO_BUFS);

    offset = static_cast<uint16_t>(packed);

    // Pack the data (if any)
    if (aFormat)
    {
        packed = spinel_datatype_vpack(buffer + offset, sizeof(buffer) - offset, aFormat, args);
        VerifyOrExit(packed > 0 && static_cast<size_t>(packed + offset) <= sizeof(buffer), error = OT_ERROR_NO_BUFS);

        offset += static_cast<uint16_t>(packed);
    }

    SuccessOrExit(error = mSpinelInterface->SendFrame(buffer, offset));

exit:
    return error;
}

void SpinelBase::HandleNotification(SpinelInterface::RxFrameBuffer &aFrameBuffer)
{
    spinel_prop_key_t key;
    spinel_size_t     len = 0;
    spinel_ssize_t    unpacked;
    uint8_t          *data = nullptr;
    uint32_t          cmd;
    uint8_t           header;
    otError           error           = OT_ERROR_NONE;
    bool              shouldSaveFrame = false;

    unpacked = spinel_datatype_unpack(aFrameBuffer.GetFrame(), aFrameBuffer.GetLength(), "CiiD", &header, &cmd, &key,
                                      &data, &len);
    VerifyOrExit(unpacked > 0, error = OT_ERROR_PARSE);
    VerifyOrExit(SPINEL_HEADER_GET_TID(header) == 0, error = OT_ERROR_PARSE);

    switch (cmd)
    {
    case SPINEL_CMD_PROP_VALUE_IS:
        // Some spinel properties cannot be handled during `WaitResponse()`, we must cache these events.
        // `mWaitingTid` is released immediately after received the response. And `mWaitingKey` is be set
        // to `SPINEL_PROP_LAST_STATUS` at the end of `WaitResponse()`.

        if (!IsSafeToHandleNow(key))
        {
            ExitNow(shouldSaveFrame = true);
        }

        HandleValueIs(key, data, static_cast<uint16_t>(len));
        break;

    case SPINEL_CMD_PROP_VALUE_INSERTED:
        HandleValueInserted(key, data, static_cast<uint16_t>(len));
        break;
    case SPINEL_CMD_PROP_VALUE_REMOVED:
        LogInfo("Ignored command %lu", ToUlong(cmd));
        break;

    default:
        ExitNow(error = OT_ERROR_PARSE);
    }

exit:
    if (!shouldSaveFrame || aFrameBuffer.SaveFrame() != OT_ERROR_NONE)
    {
        aFrameBuffer.DiscardFrame();

        if (shouldSaveFrame)
        {
            LogCrit("RX Spinel buffer full, dropped incoming frame");
        }
    }

    LogIfFail("Error processing notification", error);
}

void SpinelBase::HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    (void)aKey;
    (void)aBuffer;
    (void)aLength;
}

void SpinelBase::HandleValueInserted(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    (void)aKey;
    (void)aBuffer;
    (void)aLength;
}

void SpinelBase::HandleSavedNotification(const uint8_t *aFrame, uint16_t aLength)
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

void SpinelBase::HandleResponse(const uint8_t *aBuffer, uint16_t aLength)
{
    spinel_prop_key_t key;
    uint8_t          *data   = nullptr;
    spinel_size_t     len    = 0;
    uint8_t           header = 0;
    uint32_t          cmd    = 0;
    spinel_ssize_t    rval   = 0;
    otError           error  = OT_ERROR_NONE;

    rval = spinel_datatype_unpack(aBuffer, aLength, "CiiD", &header, &cmd, &key, &data, &len);
    VerifyOrExit(rval > 0 && cmd >= SPINEL_CMD_PROP_VALUE_IS && cmd <= SPINEL_CMD_PROP_VALUE_REMOVED,
                 error = OT_ERROR_PARSE);

    if (mWaitingTid == SPINEL_HEADER_GET_TID(header))
    {
        HandleWaitingResponse(cmd, key, data, static_cast<uint16_t>(len));
        FreeTid(mWaitingTid);
        mWaitingTid = 0;
    }
    else
    {
        LogWarn("Unexpected Spinel transaction message: %u", SPINEL_HEADER_GET_TID(header));
        error = OT_ERROR_DROP;
    }
exit:
    LogIfFail("Error processing response", error);
}

void SpinelBase::HandleWaitingResponse(uint32_t          aCommand,
                                        spinel_prop_key_t aKey,
                                        const uint8_t    *aBuffer,
                                        uint16_t          aLength)
{
    if (aKey == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t status;
        spinel_ssize_t  unpacked = spinel_datatype_unpack(aBuffer, aLength, "i", &status);

        VerifyOrExit(unpacked > 0, mError = OT_ERROR_PARSE);
        mError = SpinelStatusToOtError(status);
    }
    else if (aKey == mWaitingKey)
    {
        if (mPropertyFormat)
        {
            {
                spinel_ssize_t unpacked =
                    spinel_datatype_vunpack_in_place(aBuffer, aLength, mPropertyFormat, mPropertyArgs);
                VerifyOrExit(unpacked > 0, mError = OT_ERROR_PARSE);
                mError = OT_ERROR_NONE;
            }
        }
        else
        {
            if (aCommand == mExpectedCommand)
            {
                mError = OT_ERROR_NONE;
            }
            else
            {
                mError = OT_ERROR_DROP;
            }
        }
    }
    else
    {
        mError = OT_ERROR_DROP;
    }

exit:
    LogIfFail("Error processing result", mError);
}

void SpinelBase::ProcessFrameQueue(void)
{
    uint8_t *frame = nullptr;
    uint16_t length;

    while (mRxFrameBuffer.GetNextSavedFrame(frame, length) == OT_ERROR_NONE)
    {
        HandleSavedNotification(frame, length);
    }

    mRxFrameBuffer.ClearSavedFrames();
}

spinel_tid_t SpinelBase::GetNextTid(void)
{
    spinel_tid_t tid = mCmdNextTid;

    while (((1 << tid) & mCmdTidsInUse) != 0)
    {
        tid = SPINEL_GET_NEXT_TID(tid);

        if (tid == mCmdNextTid)
        {
            // We looped back to `mCmdNextTid` indicating that all
            // TIDs are in-use.

            ExitNow(tid = 0);
        }
    }

    mCmdTidsInUse |= (1 << tid);
    mCmdNextTid = SPINEL_GET_NEXT_TID(tid);

exit:
    return tid;
}

otError SpinelBase::RequestV(uint32_t command, spinel_prop_key_t aKey, const char *aFormat, va_list aArgs)
{
    otError      error = OT_ERROR_NONE;
    spinel_tid_t tid   = GetNextTid();

    VerifyOrExit(tid > 0, error = OT_ERROR_BUSY);

    error = SendCommand(command, aKey, tid, aFormat, aArgs);
    SuccessOrExit(error);

    if (aKey == SPINEL_PROP_STREAM_RAW)
    {
        // not allowed to send another frame before the last frame is done.
        assert(mTxRadioTid == 0);
        VerifyOrExit(mTxRadioTid == 0, error = OT_ERROR_BUSY);
        mTxRadioTid = tid;
    }
    else
    {
        mWaitingKey = aKey;
        mWaitingTid = tid;
        error       = WaitResponse();
    }

exit:
    return error;
}

otError SpinelBase::Request(uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, ...)
{
    va_list args;
    va_start(args, aFormat);
    otError status = RequestV(aCommand, aKey, aFormat, args);
    va_end(args);
    return status;
}

otError SpinelBase::RequestWithPropertyFormat(const char       *aPropertyFormat,
                                               uint32_t          aCommand,
                                               spinel_prop_key_t aKey,
                                               const char       *aFormat,
                                               ...)
{
    otError error;
    va_list args;

    va_start(args, aFormat);
    error = RequestWithPropertyFormatV(aPropertyFormat, aCommand, aKey, aFormat, args);
    va_end(args);

    return error;
}

otError SpinelBase::RequestWithPropertyFormatV(const char       *aPropertyFormat,
                                                uint32_t          aCommand,
                                                spinel_prop_key_t aKey,
                                                const char       *aFormat,
                                                va_list           aArgs)
{
    otError error;

    mPropertyFormat = aPropertyFormat;
    error           = RequestV(aCommand, aKey, aFormat, aArgs);
    mPropertyFormat = nullptr;

    return error;
}

otError SpinelBase::RequestWithExpectedCommandV(uint32_t          aExpectedCommand,
                                                 uint32_t          aCommand,
                                                 spinel_prop_key_t aKey,
                                                 const char       *aFormat,
                                                 va_list           aArgs)
{
    otError error;

    mExpectedCommand = aExpectedCommand;
    error            = RequestV(aCommand, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

void SpinelBase::HandleTimeout(void)
{
    DieNow(OT_EXIT_RADIO_SPINEL_NO_RESPONSE);
}

void SpinelBase::LogIfFail(const char *aText, otError aError)
{
    OT_UNUSED_VARIABLE(aText);

    if (aError != OT_ERROR_NONE && aError != OT_ERROR_NO_ACK)
    {
        LogWarn("%s: %s", aText, otThreadErrorToString(aError));
    }
}

static const char kModuleName[] = "SpinelBase";

void SpinelBase::LogCrit(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    otLogPlatArgs(OT_LOG_LEVEL_CRIT, kModuleName, aFormat, args);
    va_end(args);
}

void SpinelBase::LogWarn(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    otLogPlatArgs(OT_LOG_LEVEL_WARN, kModuleName, aFormat, args);
    va_end(args);
}

void SpinelBase::LogNote(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    otLogPlatArgs(OT_LOG_LEVEL_NOTE, kModuleName, aFormat, args);
    va_end(args);
}

void SpinelBase::LogInfo(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    otLogPlatArgs(OT_LOG_LEVEL_INFO, kModuleName, aFormat, args);
    va_end(args);
}

void SpinelBase::LogDebg(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    otLogPlatArgs(OT_LOG_LEVEL_DEBG, kModuleName, aFormat, args);
    va_end(args);
}

} // namespace Spinel
} // namespace ot
