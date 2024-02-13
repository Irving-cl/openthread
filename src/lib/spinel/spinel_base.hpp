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

#ifndef SPINEL_BASE_HPP_
#define SPINEL_BASE_HPP_

#include <openthread/instance.h>

#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_interface.hpp"

namespace ot {
namespace Spinel {

class SpinelBase
{
public:

    SpinelBase(void);

    void Init(SpinelInterface &aSpinelInterface, bool aReset, bool aSkipRcpCompatibilityCheck);

    otError Get(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError GetWithParam(spinel_prop_key_t aKey,
                         const uint8_t    *aParam,
                         spinel_size_t     aParamSize,
                         const char       *aFormat,
                         ...);

    otError Set(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError Insert(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError Remove(spinel_prop_key_t aKey, const char *aFormat, ...);

    void Process(const void *aContext);

    static otError SpinelStatusToOtError(spinel_status_t aStatus);

protected:
    enum
    {
        kMaxSpinelFrame        = SPINEL_FRAME_MAX_SIZE,
        kMaxWaitTime           = 2000, ///< Max time to wait for response in milliseconds.
        kVersionStringSize     = 128,  ///< Max size of version string.
        kCapsBufferSize        = 100,  ///< Max buffer size used to store `SPINEL_PROP_CAPS` value.
        kChannelMaskBufferSize = 32,   ///< Max buffer size used to store `SPINEL_PROP_PHY_CHAN_SUPPORTED` value.
    };

    enum
    {
        kUsPerMs  = 1000,    ///< Microseconds per millisecond.
    };

    static void HandleReceivedFrame(void *aContext);
    void HandleReceivedFrame(void);

    otError CheckSpinelVersion(void);

    void ProcessFrameQueue(void);

    spinel_tid_t GetNextTid(void);
    void         FreeTid(spinel_tid_t tid) { mCmdTidsInUse &= ~(1 << tid); }

    otError RequestV(uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, va_list aArgs);
    otError Request(uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, ...);
    otError RequestWithPropertyFormat(const char       *aPropertyFormat,
                                      uint32_t          aCommand,
                                      spinel_prop_key_t aKey,
                                      const char       *aFormat,
                                      ...);
    otError RequestWithPropertyFormatV(const char       *aPropertyFormat,
                                       uint32_t          aCommand,
                                       spinel_prop_key_t aKey,
                                       const char       *aFormat,
                                       va_list           aArgs);
    otError RequestWithExpectedCommandV(uint32_t          aExpectedCommand,
                                        uint32_t          aCommand,
                                        spinel_prop_key_t aKey,
                                        const char       *aFormat,
                                        va_list           aArgs);
    otError WaitResponse(bool aHandleRcpTimeout = true);
    otError SendCommand(uint32_t          aCommand,
                        spinel_prop_key_t aKey,
                        spinel_tid_t      aTid,
                        const char       *aFormat,
                        va_list           aArgs);

    bool IsSafeToHandleNow(spinel_prop_key_t aKey) const
    {
        (void)aKey;
        return true;
    }

    void HandleNotification(SpinelInterface::RxFrameBuffer &aFrameBuffer);
    void HandleSavedNotification(const uint8_t *aFrame, uint16_t aLength);
    void HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void HandleValueInserted(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    void HandleResponse(const uint8_t *aBuffer, uint16_t aLength);
    void HandleTransmitDone(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void HandleWaitingResponse(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    void HandleTimeout(void);

    static void LogIfFail(const char *aText, otError aError);

    static void LogCrit(const char *aFormat, ...) OT_TOOL_PRINTF_STYLE_FORMAT_ARG_CHECK(1, 2);
    static void LogWarn(const char *aFormat, ...) OT_TOOL_PRINTF_STYLE_FORMAT_ARG_CHECK(1, 2);
    static void LogNote(const char *aFormat, ...) OT_TOOL_PRINTF_STYLE_FORMAT_ARG_CHECK(1, 2);
    static void LogInfo(const char *aFormat, ...) OT_TOOL_PRINTF_STYLE_FORMAT_ARG_CHECK(1, 2);
    static void LogDebg(const char *aFormat, ...) OT_TOOL_PRINTF_STYLE_FORMAT_ARG_CHECK(1, 2);

    uint32_t Snprintf(char *aDest, uint32_t aSize, const char *aFormat, ...);
    void     LogSpinelFrame(const uint8_t *aFrame, uint16_t aLength, bool aTx);

private:
    otInstance *mInstance;

    SpinelInterface::RxFrameBuffer mRxFrameBuffer;
    SpinelInterface               *mSpinelInterface;

    uint16_t          mCmdTidsInUse;    ///< Used transaction ids.
    spinel_tid_t      mCmdNextTid;      ///< Next available transaction id.
    spinel_tid_t      mTxRadioTid;      ///< The transaction id used to send a radio frame.
    spinel_tid_t      mWaitingTid;      ///< The transaction id of current transaction.
    spinel_prop_key_t mWaitingKey;      ///< The property key of current transaction.
    const char       *mPropertyFormat;  ///< The spinel property format of current transaction.
    va_list           mPropertyArgs;    ///< The arguments pack or unpack spinel property of current transaction.
    uint32_t          mExpectedCommand; ///< Expected response command of current transaction.
    otError           mError;           ///< The result of current transaction.


    bool              mIsReady : 1;     ///< NCP ready.

    char              mVersion[kVersionStringSize];
};

} // namespace Spinel
} // namespace ot

#endif // SPINEL_BASE_HPP_
