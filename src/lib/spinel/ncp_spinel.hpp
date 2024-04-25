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

/**
 * @file
 *   This file includes definitions for the spinel based Thread controller.
 */

#ifndef NCP_SPINEL_HPP_
#define NCP_SPINEL_HPP_

#include "openthread-spinel-config.h"

#include <openthread/dataset.h>
#include <openthread/link.h>
#include <openthread/thread.h>

#include "lib/spinel/logger.hpp"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_driver.hpp"

namespace ot {
namespace Spinel {

/**
 * The class for controlling the Thread stack on the neetwork NCP co-processor (NCP).
 *
 */
class NcpSpinel : private Logger
{
public:
    /**
     * This callback is called to return the result of an active scan.
     *
     * @param[in]  aError    The error code of the active scan.
     * @param[in]  aResult   A valid pointer to the first beacon information or NULL when there are no result. The
     *                       next beacon information is located at the next position of the pointer if exists.
     * @param[in]  aCount    The count of the beacon information.
     * @param[in]  aContext  A pointer to application-specific context.
     *
     */
    typedef void (*ActiveScanCallback)(otError aError, otActiveScanResult *aResult, uint8_t aCount, void *aContext);

    /**
     * This callback is called to return the result of async operations like `SetThreadEnabled`, `Join`, `Leave`.
     *
     * @param[in]  aError    The error code of the async operation.
     * @param[in]  aContext  A pointer to application-specific context.
     *
     */
    typedef void (*OperationResultReceiver)(otError aError, void *aContext);

    /**
     * Constructor.
     *
     */
    NcpSpinel(void);

    /**
     * Destructor.
     *
     */
    ~NcpSpinel(void);

    /**
     * Do the initialization.
     *
     * @param[in]  aSoftwareReset  TRUE to try SW reset first, FALSE to directly try HW reset.
     * @param[in]  aSpinelDriver   Pointer to the SpinelDriver instance that this object depends.
     *
     */
    void Init(SpinelDriver *aSpinelDriver);

    /**
     * Do the de-initialization.
     *
     */
    void Deinit(void);

    /**
     * Perform an active scan.
     *
     * @param[in]  aScanChannels     A bit vector indicating which channels to scan (e.g. OT_CHANNEL_11_MASK).
     * @param[in]  aScanDuration     The time in milliseconds to spend scanning each channel.
     * @param[in]  aCallback         A pointer to a function called on receiving a beacon or scan completes.
     * @param[in]  aContext          A pointer to application-specific context.
     *
     */
    void ActiveScan(uint32_t aScanChannels, uint16_t aScanDuration, ActiveScanCallback aCallback, void *aContext);

    /**
     * Enables/disables Thread.
     *
     * When disables Thread, it will first detach from the network without erasing the
     * active dataset, and then disable Thread radios.
     *
     * If called with same Thread enabled state as current state, the method succeeds with
     * no-op.
     *
     * @param[in]  aEnabled   TRUE to enable the Thread network, FALSE to disable.
     * @param[in]  aReceiver  A pointer to a function called to receive the result of the operation.
     * @param[in]  aContext   A pointer to application-specific context.
     *
     */
    void SetThreadEnabled(bool aEnabled, OperationResultReceiver aReceiver, void *aContext);

    /**
     * Joins this device to the network specified by @p aActiveOpDatasetTlvs.
     *
     * @param[in]  aActiveOpDatasetTlvs  A reference to the active dataset of the network to join.
     * @param[in]  aReceiver  A pointer to a function called to receive the result of the operation.
     * @param[in]  aContext   A pointer to application-specific context.
     *
     */
    void Join(otOperationalDatasetTlvs &aActiveOpDatasetTlvs, OperationResultReceiver aReceiver, void *aContext);

    /**
     * Leaves from the current network.
     *
     * 1. It returns success immediately if this device has already left or disabled
     * 2. Else if there is already an onging join request, no action will be taken but
     *    the callback will be invoked after the previous request is completed
     * 3. Otherwise, OTBR sends Address Release Notification (i.e. ADDR_REL.ntf) to gracefully
     *    detach from the current network and it takes 1 second to finish
     * 4. The Operational Dataset will be removed from persistent storage
     *
     * @param[in]  aReceiver  A pointer to a function called to receive the result of the operation.
     * @param[in]  aContext   A pointer to application-specific context.
     *
     */
    void Leave(OperationResultReceiver aReceiver, void *aContext);

private:

    static void HandleReceivedFrame(const uint8_t *aFrame,
                                    uint16_t       aLength,
                                    uint8_t        aHeader,
                                    bool          &aSave,
                                    void          *aContext);
    void        HandleReceivedFrame(const uint8_t *aFrame, uint16_t aLength, uint8_t aHeader, bool &aShouldSaveFrame);
    static void HandleSavedFrame(const uint8_t *aFrame, uint16_t aLength, void *aContext);

    void HandleNotification(const uint8_t *aFrame, uint16_t aLength);
    void HandleResponse(const uint8_t *aFrame, uint16_t aLength);
    void HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    SpinelDriver *mSpinelDriver;
    spinel_prop_key_t mWaitingKey;      ///< The property key of current transaction.

    otDeviceRole mDeviceRole;
};

} // namespace Spinel
} // namespace ot

#endif // NCP_SPINEL_HPP_
