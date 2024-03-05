/*
 *  Copyright (c) 2018, The OpenThread Authors.
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
 *   This file includes definitions for the spinel based radio transceiver.
 */

#ifndef RADIO_SPINEL_HPP_
#define RADIO_SPINEL_HPP_

#include <openthread/platform/radio.h>

#include "openthread-spinel-config.h"
#include "core/radio/max_power_table.hpp"
#include "lib/spinel/radio_spinel_metrics.h"
#include "lib/spinel/spinel.h"
#include "lib/spinel/spinel_base.hpp"
#include "lib/spinel/spinel_interface.hpp"
#include "ncp/ncp_config.h"

namespace ot {
namespace Spinel {

struct RadioSpinelCallbacks
{
    /**
     * This callback notifies user of `RadioSpinel` of a received frame.
     *
     * @param[in]  aInstance  The OpenThread instance structure.
     * @param[in]  aFrame     A pointer to the received frame or nullptr if the receive operation failed.
     * @param[in]  aError     kErrorNone when successfully received a frame,
     *                        kErrorAbort when reception was aborted and a frame was not received,
     *                        kErrorNoBufs when a frame could not be received due to lack of rx buffer space.
     *
     */
    void (*mReceiveDone)(otInstance *aInstance, otRadioFrame *aFrame, Error aError);

    /**
     * The callback notifies user of `RadioSpinel` that the transmit operation has completed, providing, if
     * applicable, the received ACK frame.
     *
     * @param[in]  aInstance  The OpenThread instance structure.
     * @param[in]  aFrame     The transmitted frame.
     * @param[in]  aAckFrame  A pointer to the ACK frame, nullptr if no ACK was received.
     * @param[in]  aError     kErrorNone when the frame was transmitted,
     *                        kErrorNoAck when the frame was transmitted but no ACK was received,
     *                        kErrorChannelAccessFailure tx failed due to activity on the channel,
     *                        kErrorAbort when transmission was aborted for other reasons.
     *
     */
    void (*mTransmitDone)(otInstance *aInstance, otRadioFrame *aFrame, otRadioFrame *aAckFrame, Error aError);

    /**
     * This callback notifies user of `RadioSpinel` that energy scan is complete.
     *
     * @param[in]  aInstance  The OpenThread instance structure.
     * @param[in]  aMaxRssi   Maximum RSSI seen on the channel, or `SubMac::kInvalidRssiValue` if failed.
     *
     */
    void (*mEnergyScanDone)(otInstance *aInstance, int8_t aMaxRssi);

    /**
     * This callback notifies user of `RadioSpinel` that the transmission has started.
     *
     * @param[in]  aInstance  A pointer to the OpenThread instance structure.
     * @param[in]  aFrame     A pointer to the frame that is being transmitted.
     *
     */
    void (*mTxStarted)(otInstance *aInstance, otRadioFrame *aFrame);

    /**
     * This callback notifies user of `RadioSpinel` that the radio interface switchover has completed.
     *
     * @param[in]  aInstance  A pointer to the OpenThread instance structure.
     * @param[in]  aSuccess   A value indicating if the switchover was successful or not.
     *
     */
    void (*mSwitchoverDone)(otInstance *aInstance, bool aSuccess);

#if OPENTHREAD_CONFIG_DIAG_ENABLE
    /**
     * This callback notifies diagnostics module using `RadioSpinel` of a received frame.
     *
     * This callback is used when diagnostics is enabled.
     *
     * @param[in]  aInstance  The OpenThread instance structure.
     * @param[in]  aFrame     A pointer to the received frame or NULL if the receive operation failed.
     * @param[in]  aError     OT_ERROR_NONE when successfully received a frame,
     *                        OT_ERROR_ABORT when reception was aborted and a frame was not received,
     *                        OT_ERROR_NO_BUFS when a frame could not be received due to lack of rx buffer space.
     *
     */
    void (*mDiagReceiveDone)(otInstance *aInstance, otRadioFrame *aFrame, Error aError);

    /**
     * This callback notifies diagnostics module using `RadioSpinel` that the transmission has completed.
     *
     * This callback is used when diagnostics is enabled.
     *
     * @param[in]  aInstance  The OpenThread instance structure.
     * @param[in]  aFrame     A pointer to the frame that was transmitted.
     * @param[in]  aError     OT_ERROR_NONE when the frame was transmitted,
     *                        OT_ERROR_CHANNEL_ACCESS_FAILURE tx could not take place due to activity on the
     * channel, OT_ERROR_ABORT when transmission was aborted for other reasons.
     *
     */
    void (*mDiagTransmitDone)(otInstance *aInstance, otRadioFrame *aFrame, Error aError);
#endif // OPENTHREAD_CONFIG_DIAG_ENABLE
};

/**
 * The class for providing a OpenThread radio interface by talking with a radio-only
 * co-processor(RCP).
 *
 */
class RadioSpinel : public SpinelCallbacks
{
public:
    /**
     * Initializes the spinel based OpenThread transceiver.
     *
     */
    RadioSpinel(SpinelBase &aSpinelBase);

    /**
     * Deinitializes the spinel based OpenThread transceiver.
     *
     */
    ~RadioSpinel(void) { Deinit(); }

    /**
     * Initialize this radio transceiver.
     *
     */
    void Init(bool aSkipRcpCompatibilityCheck);

    /**
     * This method sets the notification callbacks.
     *
     * @param[in]  aCallbacks  A pointer to structure with notification callbacks.
     *
     */
    void SetCallbacks(const struct RadioSpinelCallbacks &aCallbacks);

    /**
     * Deinitialize this radio transceiver.
     *
     */
    void Deinit(void);

    /**
     * Gets the status of promiscuous mode.
     *
     * @retval true   Promiscuous mode is enabled.
     * @retval false  Promiscuous mode is disabled.
     *
     */
    bool IsPromiscuous(void) const { return mIsPromiscuous; }

    /**
     * Sets the status of promiscuous mode.
     *
     * @param[in]   aEnable     Whether to enable or disable promiscuous mode.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetPromiscuous(bool aEnable);

    /**
     * Sets the status of RxOnWhenIdle mode.
     *
     * @param[in]   aEnable     Whether to enable or disable RxOnWhenIdle mode.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetRxOnWhenIdle(bool aEnable);

    /**
     * Sets the Short Address for address filtering.
     *
     * @param[in] aShortAddress  The IEEE 802.15.4 Short Address.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetShortAddress(uint16_t aAddress);

    /**
     * Gets the factory-assigned IEEE EUI-64 for this transceiver.
     *
     * @param[in]  aInstance   The OpenThread instance structure.
     * @param[out] aIeeeEui64  A pointer to the factory-assigned IEEE EUI-64.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError GetIeeeEui64(uint8_t *aIeeeEui64);

    /**
     * Sets the Extended Address for address filtering.
     *
     * @param[in] aExtAddress  A pointer to the IEEE 802.15.4 Extended Address stored in little-endian byte order.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetExtendedAddress(const otExtAddress &aExtAddress);

    /**
     * Sets the PAN ID for address filtering.
     *
     * @param[in]   aPanId  The IEEE 802.15.4 PAN ID.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetPanId(uint16_t aPanId);

    /**
     * Gets the radio's transmit power in dBm.
     *
     * @param[out]  aPower    The transmit power in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError GetTransmitPower(int8_t &aPower);

    /**
     * Sets the radio's transmit power in dBm.
     *
     * @param[in]   aPower     The transmit power in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetTransmitPower(int8_t aPower);

    /**
     * Gets the radio's CCA ED threshold in dBm.
     *
     * @param[out]  aThreshold    The CCA ED threshold in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError GetCcaEnergyDetectThreshold(int8_t &aThreshold);

    /**
     * Sets the radio's CCA ED threshold in dBm.
     *
     * @param[in]   aThreshold     The CCA ED threshold in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetCcaEnergyDetectThreshold(int8_t aThreshold);

    /**
     * Gets the FEM's Rx LNA gain in dBm.
     *
     * @param[out]  aGain    The FEM's Rx LNA gain in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError GetFemLnaGain(int8_t &aGain);

    /**
     * Sets the FEM's Rx LNA gain in dBm.
     *
     * @param[in]   aGain     The FEM's Rx LNA gain in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetFemLnaGain(int8_t aGain);

    /**
     * Returns the radio capabilities.
     *
     * @returns The radio capability bit vector.
     *
     */
    otRadioCaps GetRadioCaps(void) const { return sRadioCaps; }

    /**
     * Gets the most recent RSSI measurement.
     *
     * @returns The RSSI in dBm when it is valid.  127 when RSSI is invalid.
     */
    int8_t GetRssi(void);

    /**
     * Returns the radio receive sensitivity value.
     *
     * @returns The radio receive sensitivity value in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    int8_t GetReceiveSensitivity(void) const { return mRxSensitivity; }

    /**
     * Gets current state of the radio.
     *
     * @return  Current state of the radio.
     *
     */
    otRadioState GetState(void) const;

    /**
     * Gets the current receiving channel.
     *
     * @returns Current receiving channel.
     *
     */
    uint8_t GetChannel(void) const { return mChannel; }

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
    /**
     * Enable the radio coex.
     *
     * @param[in] aInstance  The OpenThread instance structure.
     * @param[in] aEnabled   TRUE to enable the radio coex, FALSE otherwise.
     *
     * @retval OT_ERROR_NONE     Successfully enabled.
     * @retval OT_ERROR_FAILED   The radio coex could not be enabled.
     *
     */
    otError SetCoexEnabled(bool aEnabled);

    /**
     * Check whether radio coex is enabled or not.
     *
     * @param[in] aInstance  The OpenThread instance structure.
     *
     * @returns TRUE if the radio coex is enabled, FALSE otherwise.
     *
     */
    bool IsCoexEnabled(void);

    /**
     * Retrieves the radio coexistence metrics.
     *
     * @param[out] aCoexMetrics  A reference to the coexistence metrics structure.
     *
     * @retval OT_ERROR_NONE          Successfully retrieved the coex metrics.
     * @retval OT_ERROR_INVALID_ARGS  @p aCoexMetrics was nullptr.
     *
     */
    otError GetCoexMetrics(otRadioCoexMetrics &aCoexMetrics);
#endif // OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE

    /**
     * Returns a reference to the transmit buffer.
     *
     * The caller forms the IEEE 802.15.4 frame in this buffer then calls otPlatRadioTransmit() to request transmission.
     *
     * @returns A reference to the transmit buffer.
     *
     */
    otRadioFrame &GetTransmitFrame(void) { return mTxRadioFrame; }

    /**
     * Enables or disables source address match feature.
     *
     * @param[in]  aEnable     Enable/disable source address match feature.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError EnableSrcMatch(bool aEnable);

    /**
     * Adds a short address to the source address match table.
     *
     * @param[in]  aInstance      The OpenThread instance structure.
     * @param[in]  aShortAddress  The short address to be added.
     *
     * @retval  OT_ERROR_NONE               Successfully added short address to the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_BUFS            No available entry in the source match table.
     */
    otError AddSrcMatchShortEntry(uint16_t aShortAddress);

    /**
     * Removes a short address from the source address match table.
     *
     * @param[in]  aInstance      The OpenThread instance structure.
     * @param[in]  aShortAddress  The short address to be removed.
     *
     * @retval  OT_ERROR_NONE               Successfully removed short address from the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_ADDRESS         The short address is not in source address match table.
     */
    otError ClearSrcMatchShortEntry(uint16_t aShortAddress);

    /**
     * Clear all short addresses from the source address match table.
     *
     * @param[in]  aInstance   The OpenThread instance structure.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError ClearSrcMatchShortEntries(void);

    /**
     * Add an extended address to the source address match table.
     *
     * @param[in]  aInstance    The OpenThread instance structure.
     * @param[in]  aExtAddress  The extended address to be added stored in little-endian byte order.
     *
     * @retval  OT_ERROR_NONE               Successfully added extended address to the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_BUFS            No available entry in the source match table.
     */
    otError AddSrcMatchExtEntry(const otExtAddress &aExtAddress);

    /**
     * Remove an extended address from the source address match table.
     *
     * @param[in]  aInstance    The OpenThread instance structure.
     * @param[in]  aExtAddress  The extended address to be removed stored in little-endian byte order.
     *
     * @retval  OT_ERROR_NONE               Successfully removed the extended address from the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_ADDRESS         The extended address is not in source address match table.
     */
    otError ClearSrcMatchExtEntry(const otExtAddress &aExtAddress);

    /**
     * Clear all the extended/long addresses from source address match table.
     *
     * @param[in]  aInstance   The OpenThread instance structure.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError ClearSrcMatchExtEntries(void);

    /**
     * Begins the energy scan sequence on the radio.
     *
     * @param[in]  aScanChannel     The channel to perform the energy scan on.
     * @param[in]  aScanDuration    The duration, in milliseconds, for the channel to be scanned.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError EnergyScan(uint8_t aScanChannel, uint16_t aScanDuration);

    /**
     * Switches the radio state from Receive to Transmit.
     *
     * @param[in] aFrame     A reference to the transmitted frame.
     *
     * @retval  OT_ERROR_NONE               Successfully transitioned to Transmit.
     * @retval  OT_ERROR_BUSY               Failed due to another transmission is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_INVALID_STATE      The radio was not in the Receive state.
     */
    otError Transmit(otRadioFrame &aFrame);

    /**
     * Switches the radio state from Sleep to Receive.
     *
     * @param[in]  aChannel   The channel to use for receiving.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Receive.
     * @retval OT_ERROR_INVALID_STATE The radio was disabled or transmitting.
     *
     */
    otError Receive(uint8_t aChannel);

    /**
     * Switches the radio state from Receive to Sleep.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Sleep.
     * @retval OT_ERROR_BUSY          The radio was transmitting
     * @retval OT_ERROR_INVALID_STATE The radio was disabled
     *
     */
    otError Sleep(void);

    /**
     * Enable the radio.
     *
     * @param[in]   aInstance   A pointer to the OpenThread instance.
     *
     * @retval OT_ERROR_NONE     Successfully enabled.
     * @retval OT_ERROR_FAILED   The radio could not be enabled.
     *
     */
    otError Enable(otInstance *aInstance);

    /**
     * Disable the radio.
     *
     * @retval  OT_ERROR_NONE               Successfully transitioned to Disabled.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError Disable(void);

    /**
     * Checks whether radio is enabled or not.
     *
     * @returns TRUE if the radio is enabled, FALSE otherwise.
     *
     */
    bool IsEnabled(void) const { return mState != kStateDisabled; }

    /**
     * Indicates whether there is a pending transmission.
     *
     * @retval TRUE  There is a pending transmission.
     * @retval FALSE There is no pending transmission.
     *
     */
    bool IsTransmitting(void) const { return mState == kStateTransmitting; }

    /**
     * Indicates whether a transmit has just finished.
     *
     * @retval TRUE  The transmission is done.
     * @retval FALSE The transmission is not done.
     *
     */
    bool IsTransmitDone(void) const { return mState == kStateTransmitDone; }

    /**
     * Returns the timeout timepoint for the pending transmission.
     *
     * @returns The timeout timepoint for the pending transmission.
     *
     */
    uint64_t GetTxRadioEndUs(void) const { return mTxRadioEndUs; }

    /**
     * Processes any pending the I/O data.
     *
     * @param[in]  aContext   The process context.
     *
     */
    void Process(const void *aContext);

#if OPENTHREAD_CONFIG_DIAG_ENABLE
    /**
     * Enables/disables the factory diagnostics mode.
     *
     * @param[in]  aMode  TRUE to enable diagnostics mode, FALSE otherwise.
     *
     */
    void SetDiagEnabled(bool aMode) { mDiagMode = aMode; }

    /**
     * Indicates whether or not factory diagnostics mode is enabled.
     *
     * @returns TRUE if factory diagnostics mode is enabled, FALSE otherwise.
     *
     */
    bool IsDiagEnabled(void) const { return mDiagMode; }

    /**
     * Processes platform diagnostics commands.
     *
     * @param[in]   aString         A null-terminated input string.
     * @param[out]  aOutput         The diagnostics execution result.
     * @param[in]   aOutputMaxLen   The output buffer size.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError PlatDiagProcess(const char *aString, char *aOutput, size_t aOutputMaxLen);
#endif

    /**
     * Returns the radio channel mask.
     *
     * @param[in]   aPreferred  TRUE to get preferred channel mask, FALSE to get supported channel mask.
     *
     * @returns The radio channel mask according to @aPreferred:
     *   The radio supported channel mask that the device is allowed to be on.
     *   The radio preferred channel mask that the device prefers to form on.
     *
     */
    uint32_t GetRadioChannelMask(bool aPreferred);

    /**
     * Sets MAC key and key index to RCP.
     *
     * @param[in] aKeyIdMode  The key ID mode.
     * @param[in] aKeyId      The key index.
     * @param[in] aPrevKey    Pointer to previous MAC key.
     * @param[in] aCurrKey    Pointer to current MAC key.
     * @param[in] aNextKey    Pointer to next MAC key.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_INVALID_ARGS       One of the keys passed is invalid..
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError SetMacKey(uint8_t                 aKeyIdMode,
                      uint8_t                 aKeyId,
                      const otMacKeyMaterial *aPrevKey,
                      const otMacKeyMaterial *aCurrKey,
                      const otMacKeyMaterial *aNextKey);

    /**
     * Sets the current MAC Frame Counter value.
     *
     * @param[in] aMacFrameCounter  The MAC Frame Counter value.
     * @param[in] aSetIfLarger      If `true`, set only if the new value is larger than the current value.
     *                              If `false`, set the new value independent of the current value.
     *
     */
    otError SetMacFrameCounter(uint32_t aMacFrameCounter, bool aSetIfLarger);

    /**
     * Sets the radio region code.
     *
     * @param[in]   aRegionCode  The radio region code.
     *
     * @retval  OT_ERROR_NONE             Successfully set region code.
     * @retval  OT_ERROR_FAILED           Other platform specific errors.
     *
     */
    otError SetRadioRegion(uint16_t aRegionCode);

    /**
     * Gets the radio region code.
     *
     * @param[out]   aRegionCode  The radio region code.
     *
     * @retval  OT_ERROR_INVALID_ARGS     @p aRegionCode is nullptr.
     * @retval  OT_ERROR_NONE             Successfully got region code.
     * @retval  OT_ERROR_FAILED           Other platform specific errors.
     *
     */
    otError GetRadioRegion(uint16_t *aRegionCode);

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    /**
     * Enable/disable or update Enhanced-ACK Based Probing in radio for a specific Initiator.
     *
     * After Enhanced-ACK Based Probing is configured by a specific Probing Initiator, the Enhanced-ACK sent to that
     * node should include Vendor-Specific IE containing Link Metrics data. This method informs the radio to start/stop
     * to collect Link Metrics data and include Vendor-Specific IE that containing the data in Enhanced-ACK sent to that
     * Probing Initiator.
     *
     * @param[in]  aLinkMetrics   This parameter specifies what metrics to query. Per spec 4.11.3.4.4.6, at most 2
     *                            metrics can be specified. The probing would be disabled if @p aLinkMetrics is
     *                            bitwise 0.
     * @param[in]  aShortAddress  The short address of the Probing Initiator.
     * @param[in]  aExtAddress    The extended source address of the Probing Initiator. @p aExtAddress MUST NOT be
     *                            nullptr.
     *
     * @retval  OT_ERROR_NONE            Successfully configured the Enhanced-ACK Based Probing.
     * @retval  OT_ERROR_INVALID_ARGS    @p aExtAddress is nullptr.
     * @retval  OT_ERROR_NOT_FOUND       The Initiator indicated by @p aShortAddress is not found when trying to clear.
     * @retval  OT_ERROR_NO_BUFS         No more Initiator can be supported.
     */
    otError ConfigureEnhAckProbing(otLinkMetrics         aLinkMetrics,
                                   const otShortAddress &aShortAddress,
                                   const otExtAddress   &aExtAddress);
#endif

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE || OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    /**
     * Get the current accuracy, in units of ± ppm, of the clock used for scheduling CSL operations.
     *
     * @note Platforms may optimize this value based on operational conditions (i.e.: temperature).
     *
     * @retval   The current CSL rx/tx scheduling drift, in units of ± ppm.
     *
     */
    uint8_t GetCslAccuracy(void);
#endif

#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    /**
     * Get the current uncertainty, in units of 10 us, of the clock used for scheduling CSL operations.
     *
     * @retval  The current CSL Clock Uncertainty in units of 10 us.
     *
     */
    uint8_t GetCslUncertainty(void);
#endif

    /**
     * Returns the next timepoint to recalculate RCP time offset.
     *
     * @returns The timepoint to start the recalculation of RCP time offset.
     *
     */
    uint64_t GetNextRadioTimeRecalcStart(void) const { return mRadioTimeRecalcStart; }

    /**
     * Gets the current estimated time on RCP.
     *
     * @returns The current estimated RCP time in microseconds.
     *
     */
    uint64_t GetNow(void);

    /**
     * Returns the bus speed between the host and the radio.
     *
     * @returns   bus speed in bits/second.
     *
     */
    uint32_t GetBusSpeed(void) const;

    /**
     * Sets the max transmit power.
     *
     * @param[in] aChannel    The radio channel.
     * @param[in] aMaxPower   The max transmit power in dBm.
     *
     * @retval  OT_ERROR_NONE           Successfully set the max transmit power.
     * @retval  OT_ERROR_INVALID_ARGS   Channel is not in valid range.
     *
     */
    otError SetChannelMaxTransmitPower(uint8_t aChannel, int8_t aMaxPower);

    /**
     * Returns the radio Spinel metrics.
     *
     * @returns The radio Spinel metrics.
     *
     */
    const otRadioSpinelMetrics *GetRadioSpinelMetrics(void) const { return &mRadioSpinelMetrics; }

#if OPENTHREAD_CONFIG_PLATFORM_POWER_CALIBRATION_ENABLE
    /**
     * Add a calibrated power of the specified channel to the power calibration table.
     *
     * @param[in] aChannel                The radio channel.
     * @param[in] aActualPower            The actual power in 0.01dBm.
     * @param[in] aRawPowerSetting        A pointer to the raw power setting byte array.
     * @param[in] aRawPowerSettingLength  The length of the @p aRawPowerSetting.
     *
     * @retval  OT_ERROR_NONE              Successfully added the calibrated power to the power calibration table.
     * @retval  OT_ERROR_NO_BUFS           No available entry in the power calibration table.
     * @retval  OT_ERROR_INVALID_ARGS      The @p aChannel, @p aActualPower or @p aRawPowerSetting is invalid.
     * @retval  OT_ERROR_NOT_IMPLEMENTED   This feature is not implemented.
     * @retval  OT_ERROR_BUSY              Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT  Failed due to no response received from the transceiver.
     *
     */
    otError AddCalibratedPower(uint8_t        aChannel,
                               int16_t        aActualPower,
                               const uint8_t *aRawPowerSetting,
                               uint16_t       aRawPowerSettingLength);

    /**
     * Clear all calibrated powers from the power calibration table.
     *
     * @retval  OT_ERROR_NONE              Successfully cleared all calibrated powers from the power calibration table.
     * @retval  OT_ERROR_NOT_IMPLEMENTED   This feature is not implemented.
     * @retval  OT_ERROR_BUSY              Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT  Failed due to no response received from the transceiver.
     *
     */
    otError ClearCalibratedPowers(void);

    /**
     * Set the target power for the given channel.
     *
     * @param[in]  aChannel      The radio channel.
     * @param[in]  aTargetPower  The target power in 0.01dBm. Passing `INT16_MAX` will disable this channel.
     *
     * @retval  OT_ERROR_NONE              Successfully set the target power.
     * @retval  OT_ERROR_INVALID_ARGS      The @p aChannel or @p aTargetPower is invalid..
     * @retval  OT_ERROR_NOT_IMPLEMENTED   The feature is not implemented.
     * @retval  OT_ERROR_BUSY              Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT  Failed due to no response received from the transceiver.
     *
     */
    otError SetChannelTargetPower(uint8_t aChannel, int16_t aTargetPower);
#endif

private:
    enum
    {
        kCapsBufferSize = 100, ///< Max buffer size used to store `SPINEL_PROP_CAPS` value.
    };

    enum State
    {
        kStateDisabled,     ///< Radio is disabled.
        kStateSleep,        ///< Radio is sleep.
        kStateReceive,      ///< Radio is in receive mode.
        kStateTransmitting, ///< Frame passed to radio for transmission, waiting for done event from radio.
        kStateTransmitDone, ///< Radio indicated frame transmission is done.
    };

    static constexpr uint32_t kUsPerMs  = 1000;                 ///< Microseconds per millisecond.
    static constexpr uint32_t kMsPerSec = 1000;                 ///< Milliseconds per second.
    static constexpr uint32_t kUsPerSec = kUsPerMs * kMsPerSec; ///< Microseconds per second.
    static constexpr uint64_t kTxWaitUs =
        OPENTHREAD_SPINEL_CONFIG_RCP_TX_WAIT_TIME_SECS *
        kUsPerSec; ///< Maximum time of waiting for `TransmitDone` event, in microseconds.

    /**
     * Checks whether the spinel interface is radio-only.
     *
     * @param[out] aSupportsRcpApiVersion          A reference to a boolean variable to update whether the list of
     *                                             spinel capabilities includes `SPINEL_CAP_RCP_API_VERSION`.
     * @param[out] aSupportsRcpMinHostApiVersion   A reference to a boolean variable to update whether the list of
     *                                             spinel capabilities includes `SPINEL_CAP_RCP_MIN_HOST_API_VERSION`.
     *
     * @retval  TRUE    The radio chip is in radio-only mode.
     * @retval  FALSE   Otherwise.
     *
     */
    bool IsRcp(bool &aSupportsRcpApiVersion, bool &aSupportsRcpMinHostApiVersion);

    otError CheckRadioCapabilities(void);
    otError CheckRcpApiVersion(bool aSupportsRcpApiVersion, bool aSupportsRcpMinHostApiVersion);

    /**
     * Triggers a state transfer of the state machine.
     *
     */
    void ProcessRadioStateMachine(void);

    otError ParseRadioFrame(otRadioFrame &aFrame, const uint8_t *aBuffer, uint16_t aLength, spinel_ssize_t &aUnpacked);

    /**
     * Returns if the property changed event is safe to be handled now.
     *
     * If a property handler will go up to core stack, it may cause reentrant issue of `Hdlc::Decode()` and
     * `WaitResponse()`.
     *
     * @param[in] aKey The identifier of the property.
     *
     * @returns Whether this property is safe to be handled now.
     *
     */
    bool IsSafeToHandleNow(spinel_prop_key_t aKey) const
    {
        return !(aKey == SPINEL_PROP_STREAM_RAW || aKey == SPINEL_PROP_MAC_ENERGY_SCAN_RESULT);
    }

    otError HandleCmdFromNotification(uint32_t          aCmd,
                                      spinel_prop_key_t aKey,
                                      const uint8_t    *aData,
                                      uint16_t          aLength,
                                      bool             &aShouldSaveFrame);

    otError HandleCmdFromSavedNotification(uint32_t          aCmd,
                                           spinel_prop_key_t aKey,
                                           const uint8_t    *aData,
                                           uint16_t          aLength);

    void HandleStreamRawResponse(uint32_t aCmd, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    otError HandleStreamMfgResponse(const uint8_t *aBuffer, uint16_t aLength);

    void HandleCpTimeout(void);

    void RecoverFromCpFailure(void);

    void HandleValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    void HandleTransmitDone(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);

    void RadioReceive(void);

    void TransmitDone(otRadioFrame *aFrame, otRadioFrame *aAckFrame, otError aError);

    void CalcRcpTimeOffset(void);

    void HandleRcpUnexpectedReset(spinel_status_t aStatus);

    void UpdateParseErrorCount(otError aError)
    {
        mRadioSpinelMetrics.mSpinelParseErrorCount += (aError == OT_ERROR_PARSE) ? 1 : 0;
    }

    otError SetMacKey(uint8_t         aKeyIdMode,
                      uint8_t         aKeyId,
                      const otMacKey &aPrevKey,
                      const otMacKey &aCurrKey,
                      const otMacKey &NextKey);
#if OPENTHREAD_CONFIG_PLATFORM_KEY_REFERENCES_ENABLE
    static otError ReadMacKey(const otMacKeyMaterial &aKeyMaterial, otMacKey &aKey);
#endif

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    /**
     * Restore the properties of Radio Co-processor (RCP).
     *
     */
    void RestoreProperties(void);
#endif

    otInstance *mInstance;

    SpinelBase &mSpinelBase;

    RadioSpinelCallbacks mCallbacks; ///< Callbacks for notifications of higher layer.

    uint8_t       mRxPsdu[OT_RADIO_FRAME_MAX_SIZE];
    uint8_t       mTxPsdu[OT_RADIO_FRAME_MAX_SIZE];
    uint8_t       mAckPsdu[OT_RADIO_FRAME_MAX_SIZE];
    otRadioFrame  mRxRadioFrame;
    otRadioFrame  mTxRadioFrame;
    otRadioFrame  mAckRadioFrame;
    otRadioFrame *mTransmitFrame; ///< Points to the frame to send

    otExtAddress        mExtendedAddress;
    uint16_t            mShortAddress;
    uint16_t            mPanId;
    uint8_t             mChannel;
    int8_t              mRxSensitivity;
    State               mState;
    otError             mTxError;
    static otExtAddress sIeeeEui64;
    static otRadioCaps  sRadioCaps;

    bool mIsPromiscuous : 1; ///< Promiscuous mode.
    bool mRxOnWhenIdle : 1;  ///< RxOnWhenIdle mode.
    bool mIsTimeSynced : 1;  ///< Host has calculated the time difference between host and RCP.

    static bool sSupportsLogStream; ///< RCP supports `LOG_STREAM` property with OpenThread log meta-data format.
    static bool sSupportsResetToBootloader; ///< RCP supports resetting into bootloader mode.

#if OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0
    enum
    {
        kRcpFailureNone,
        kRcpFailureTimeout,
        kRcpFailureUnexpectedReset,
    };

    bool    mResetRadioOnStartup : 1; ///< Whether should send reset command when init.
    int16_t mRcpFailureCount;         ///< Count of consecutive RCP failures.
    uint8_t mRcpFailure : 2;          ///< RCP failure reason, should recover and retry operation.
                                      ///
    // Properties set by core.
    uint8_t      mKeyIdMode;
    uint8_t      mKeyId;
    otMacKey     mPrevKey;
    otMacKey     mCurrKey;
    otMacKey     mNextKey;
    uint16_t     mSrcMatchShortEntries[OPENTHREAD_CONFIG_MLE_MAX_CHILDREN];
    int16_t      mSrcMatchShortEntryCount;
    otExtAddress mSrcMatchExtEntries[OPENTHREAD_CONFIG_MLE_MAX_CHILDREN];
    int16_t      mSrcMatchExtEntryCount;
    uint8_t      mScanChannel;
    uint16_t     mScanDuration;
    int8_t       mCcaEnergyDetectThreshold;
    int8_t       mTransmitPower;
    int8_t       mFemLnaGain;
    uint32_t     mMacFrameCounter;
    bool         mCoexEnabled : 1;

    bool mMacKeySet : 1;                   ///< Whether MAC key has been set.
    bool mCcaEnergyDetectThresholdSet : 1; ///< Whether CCA energy detect threshold has been set.
    bool mTransmitPowerSet : 1;            ///< Whether transmit power has been set.
    bool mCoexEnabledSet : 1;              ///< Whether coex enabled has been set.
    bool mFemLnaGainSet : 1;               ///< Whether FEM LNA gain has been set.
    bool mEnergyScanning : 1;              ///< If fails while scanning, restarts scanning.
    bool mMacFrameCounterSet : 1;          ///< Whether the MAC frame counter has been set.
#endif                                     // OPENTHREAD_SPINEL_CONFIG_RCP_RESTORATION_MAX_COUNT > 0

#if OPENTHREAD_CONFIG_DIAG_ENABLE
    bool   mDiagMode;
    char  *mDiagOutput;
    size_t mDiagOutputMaxLen;
#endif

    uint64_t mTxRadioEndUs;
    uint64_t mRadioTimeRecalcStart; ///< When to recalculate RCP time offset.
    uint64_t mRadioTimeOffset;      ///< Time difference with estimated RCP time minus host time.

    MaxPowerTable mMaxPowerTable;

    otRadioSpinelMetrics mRadioSpinelMetrics;
};

} // namespace Spinel
} // namespace ot

#endif // RADIO_SPINEL_HPP_
