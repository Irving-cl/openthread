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

/**
 * Maximum number of Spinel Interface IDs.
 *
 */
#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE
static constexpr uint8_t kSpinelHeaderMaxNumIid = 4;
#else
static constexpr uint8_t kSpinelHeaderMaxNumIid = 1;
#endif

class SpinelCallbacks
{
public:
    virtual otError HandleCmdFromNotification(uint32_t          aCmd,
                                              spinel_prop_key_t aKey,
                                              const uint8_t    *aData,
                                              uint16_t          aLength,
                                              bool             &aShouldSaveFrame)                 = 0;
    virtual otError HandleCmdFromSavedNotification(uint32_t          aCmd,
                                                   spinel_prop_key_t aKey,
                                                   const uint8_t    *aData,
                                                   uint16_t          aLength)                  = 0;
    virtual void    HandleStreamRawResponse(uint32_t          aCmd,
                                            spinel_prop_key_t aKey,
                                            const uint8_t    *aBuffer,
                                            uint16_t          aLength)                         = 0;
    virtual otError HandleStreamMfgResponse(const uint8_t *aBuffer, uint16_t aLength) = 0;
    virtual void    HandleCpTimeout(void)                                             = 0;
    virtual void    RecoverFromCpFailure(void)                                        = 0;
};

class SpinelBase : public SpinelCallbacks
{
public:
    SpinelBase(void);

    void Init(SpinelInterface &aSpinelInterface, bool aSwReset, const spinel_iid_t *aIidList, uint8_t aIidListLength);

    void Deinit(void);

    void Recover(bool aSkipReset);

    void ResetCp(bool aSwReset);

    void SetSpinelCallbacks(SpinelCallbacks *aSpinelCallbacks) { mSpinelCallbacks = aSpinelCallbacks; }

    otError Get(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError GetWithParam(spinel_prop_key_t aKey,
                         const uint8_t    *aParam,
                         spinel_size_t     aParamSize,
                         const char       *aFormat,
                         ...);

    otError Set(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError Insert(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError Remove(spinel_prop_key_t aKey, const char *aFormat, ...);

    otError SendReset(uint8_t aResetType);

    void Process(const void *aContext);

    static otError SpinelStatusToOtError(spinel_status_t aStatus);

    bool HasPendingFrame(void) const { return mRxFrameBuffer.HasSavedFrame(); }

    const char *GetVersion(void) const { return sVersion; }

    otError Request(uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, ...);

    void FreeTid(spinel_tid_t tid) { mCmdTidsInUse &= ~(1 << tid); }

    void ResetStreamRawTid(void);

    /**
     * Checks whether given interface ID is part of list of IIDs to be allowed.
     *
     * @param[in] aIid    Spinel Interface ID.
     *
     * @retval  TRUE    Given IID present in allow list.
     * @retval  FALSE   Otherwise.
     *
     */
    inline bool IsFrameForUs(spinel_iid_t aIid);

    /**
     * Get currently active interface.
     *
     * @param[out] aIid IID of the interface that owns the radio.
     *
     * @retval  OT_ERROR_NONE               Successfully got the property.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NOT_IMPLEMENTED    Failed due to lack of the support in radio
     * @retval  OT_ERROR_INVALID_COMMAND    Platform supports all interfaces simultaneously.
     *                                      (i.e. no active/inactive interface concept in the platform level)
     *
     */
    otError GetMultipanActiveInterface(spinel_iid_t *aIid);

    /**
     * Sets specified radio interface active
     *
     * This function allows selecting currently active radio interface on platforms that do not support parallel
     * communication on multiple interfaces. I.e. if more than one interface is in receive state calling
     * SetMultipanActiveInterface guarantees that specified interface will not be losing frames. This function
     * returns if the request was received properly. After interface switching is complete SwitchoverDone callback is
     * Invoked. Switching interfaces may take longer if aCompletePending is set true.
     *
     * @param[in] aIid              IID of the interface to set active.
     * @param[in] aCompletePending  Set true if pending radio operation should complete first(Soft switch) or false if
     * ongoing operations should be interrupted (Force switch).
     *
     * @retval  OT_ERROR_NONE               Successfully requested interface switch.
     * @retval  OT_ERROR_BUSY               Failed due to another operation on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NOT_IMPLEMENTED    Failed due to lack of support in radio for the given interface id or
     * @retval  OT_ERROR_INVALID_COMMAND    Platform supports all interfaces simultaneously
     *                                      (i.e. no active/inactive interface concept in the platform level)
     * @retval  OT_ERROR_ALREADY            Given interface is already active.
     *
     */
    otError SetMultipanActiveInterface(spinel_iid_t aIid, bool aCompletePending);

private:
    enum
    {
        kMaxSpinelFrame    = SPINEL_FRAME_MAX_SIZE,
        kMaxWaitTime       = 2000, ///< Max time to wait for response in milliseconds.
        kVersionStringSize = 128,  ///< Max size of version string.
        kCapsBufferSize    = 100,  ///< Max buffer size used to store `SPINEL_PROP_CAPS` value.
    };

    enum
    {
        kUsPerMs = 1000, ///< Microseconds per millisecond.
    };

    static void HandleReceivedFrame(void *aContext);
    void        HandleReceivedFrame(void);

    otError CheckSpinelVersion(void);

    void ProcessFrameQueue(void);

    spinel_tid_t GetNextTid(void);

    otError RequestV(uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, va_list aArgs);
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
    otError WaitResponse(bool aHandleTimeout = true);
    otError SendCommand(uint32_t          aCommand,
                        spinel_prop_key_t aKey,
                        spinel_tid_t      aTid,
                        const char       *aFormat,
                        va_list           aArgs);

    void HandleNotification(SpinelInterface::RxFrameBuffer &aFrameBuffer);
    void HandleSavedNotification(const uint8_t *aFrame, uint16_t aLength);

    otError HandleCmdFromNotification(uint32_t          aCmd,
                                      spinel_prop_key_t aKey,
                                      const uint8_t    *aData,
                                      uint16_t          aLength,
                                      bool             &aShouldSaveFrame);
    otError HandleCmdFromSavedNotification(uint32_t          aCmd,
                                           spinel_prop_key_t aKey,
                                           const uint8_t    *aData,
                                           uint16_t          aLength);
    void    HandleStreamRawResponse(uint32_t aCmd, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    otError HandleStreamMfgResponse(const uint8_t *aBuffer, uint16_t aLength);
    void    HandleCpTimeout(void);
    void    RecoverFromCpFailure(void);

    void HandleCommand(uint32_t aCommand);
    void HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    void HandleResponse(const uint8_t *aBuffer, uint16_t aLength);
    void HandleTransmitDone(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void HandleWaitingResponse(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    void HandleTimeout(void);

    otInstance *mInstance;

    SpinelInterface::RxFrameBuffer mRxFrameBuffer;
    SpinelInterface               *mSpinelInterface;

    uint16_t          mCmdTidsInUse;    ///< Used transaction ids.
    spinel_tid_t      mCmdNextTid;      ///< Next available transaction id.
    spinel_tid_t      mStreamRawTid;    ///< The transaction id used for STREAM_RAW frames.
    spinel_tid_t      mWaitingTid;      ///< The transaction id of current transaction.
    spinel_prop_key_t mWaitingKey;      ///< The property key of current transaction.
    const char       *mPropertyFormat;  ///< The spinel property format of current transaction.
    va_list           mPropertyArgs;    ///< The arguments pack or unpack spinel property of current transaction.
    uint32_t          mExpectedCommand; ///< Expected response command of current transaction.
    otError           mError;           ///< The result of current transaction.

    spinel_iid_t mIid;                             ///< the spinel interface id used by this process.
    spinel_iid_t mIidList[kSpinelHeaderMaxNumIid]; ///< Array of interface ids to accept the incoming spinel frames.

    static bool sIsReady;                   ///< NCP ready.
    static bool sSupportsResetToBootloader; ///< RCP supports resetting into bootloader mode.
                                            ///
    static char sVersion[kVersionStringSize];

    SpinelCallbacks *mSpinelCallbacks;
};

} // namespace Spinel
} // namespace ot

#endif // SPINEL_BASE_HPP_
