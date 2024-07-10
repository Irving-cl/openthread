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

#ifndef LIB_SPINEL_MAC_FRAME_HPP_
#define LIB_SPINEL_MAC_FRAME_HPP_

#include <openthread/platform/radio.h>

namespace ot {
namespace Spinel {

/**
 * Implements IEEE 802.15.4 MAC frame parsing and modifiction.
 *
 */
class MacFrame : public otRadioFrame
{
public:
    /**
     * Returns a pointer to the PSDU.
     *
     * @returns A pointer to the PSDU.
     *
     */
    const uint8_t *GetPsdu(void) const { return mPsdu; }

    /**
     * Returns the Frame Control field of the frame.
     *
     * @returns The Frame Control field.
     *
     */
    uint16_t GetFrameControlField(void) const;

    /**
     * Indicates whether or not security is enabled.
     *
     * @retval TRUE   If security is enabled.
     * @retval FALSE  If security is not enabled.
     *
     */
    bool GetSecurityEnabled(void) const { return (GetPsdu()[0] & kFcfSecurityEnabled) != 0; }

    /**
     * Sets the Key Identifier.
     *
     * @param[in]  aKeyId  The Key Identifier.
     *
     */
    void SetKeyId(uint8_t aKeyId);

    /**
     * Sets the Frame Counter.
     *
     * @param[in]  aFrameCounter  The Frame Counter.
     *
     */
    void SetFrameCounter(uint32_t aFrameCounter);

    /**
     * Sets the header updated flag attribute.
     *
     * @param[in]  aIsHeaderUpdated  TRUE if the frame header is updated.
     *
     */
    void SetIsHeaderUpdated(bool aIsHeaderUpdated) { mInfo.mTxInfo.mIsHeaderUpdated = aIsHeaderUpdated; }

private:
    static constexpr uint8_t kFcfSize    = sizeof(uint16_t);
    static constexpr uint8_t kDsnSize    = sizeof(uint8_t);
    static constexpr uint8_t k154FcsSize = sizeof(uint16_t);

    static constexpr uint8_t kSecurityControlSize = sizeof(uint8_t);
    static constexpr uint8_t kFrameCounterSize    = sizeof(uint32_t);

    static constexpr uint16_t kFcfSecurityEnabled  = 1 << 3;
    static constexpr uint16_t kFcfFramePending     = 1 << 4;
    static constexpr uint16_t kFcfAckRequest       = 1 << 5;
    static constexpr uint16_t kFcfPanidCompression = 1 << 6;
    static constexpr uint16_t kFcfIePresent        = 1 << 9;
    static constexpr uint16_t kFcfDstAddrNone      = 0 << 10;
    static constexpr uint16_t kFcfDstAddrShort     = 2 << 10;
    static constexpr uint16_t kFcfDstAddrExt       = 3 << 10;
    static constexpr uint16_t kFcfDstAddrMask      = 3 << 10;
    static constexpr uint16_t kFcfFrameVersionMask = 3 << 12;
    static constexpr uint16_t kFcfSrcAddrNone      = 0 << 14;
    static constexpr uint16_t kFcfSrcAddrShort     = 2 << 14;
    static constexpr uint16_t kFcfSrcAddrExt       = 3 << 14;
    static constexpr uint16_t kFcfSrcAddrMask      = 3 << 14;

    static constexpr uint8_t kKeyIdModeMask = 3 << 3;

    static constexpr uint8_t kKeySourceSizeMode0 = 0;
    static constexpr uint8_t kKeySourceSizeMode1 = 0;
    static constexpr uint8_t kKeySourceSizeMode2 = 4;
    static constexpr uint8_t kKeySourceSizeMode3 = 8;

    static constexpr uint8_t kInvalidIndex = 0xff;
    static constexpr uint8_t kInvalidSize  = kInvalidIndex;

    static constexpr uint16_t kVersion2003 = 0 << 12;
    static constexpr uint16_t kVersion2006 = 1 << 12;
    static constexpr uint16_t kVersion2015 = 2 << 12;

    static constexpr uint8_t kKeyIdMode0 = 0 << 3; ///< Key ID Mode 0 - Key is determined implicitly.
    static constexpr uint8_t kKeyIdMode1 = 1 << 3; ///< Key ID Mode 1 - Key is determined from Key Index field.
    static constexpr uint8_t kKeyIdMode2 =
        2 << 3; ///< Key ID Mode 2 - Key is determined from 4-bytes Key Source and Index fields.
    static constexpr uint8_t kKeyIdMode3 =
        3 << 3; ///< Key ID Mode 3 - Key is determined from 8-bytes Key Source and Index fields.

    class LittleEndian
    {
    public:
        static uint16_t ReadUint16(const uint8_t *aBuffer)
        {
            return static_cast<uint16_t>((aBuffer[0] << 8) | aBuffer[1]);
        }
        static void WriteUint32(uint32_t aValue, uint8_t *aBuffer)
        {
            aBuffer[0] = (aValue >> 0) & 0xff;
            aBuffer[1] = (aValue >> 8) & 0xff;
            aBuffer[2] = (aValue >> 16) & 0xff;
            aBuffer[3] = (aValue >> 24) & 0xff;
        }
    };

    uint8_t FindSecurityHeaderIndex(void) const;
    uint8_t SkipAddrFieldIndex(void) const;

    static uint8_t GetFcsSize(void) { return k154FcsSize; }

    static uint8_t GetKeySourceLength(uint8_t aKeyIdMode);
    static bool    IsDstAddrPresent(uint16_t aFcf) { return (aFcf & kFcfDstAddrMask) != kFcfDstAddrNone; }
    static bool    IsDstPanIdPresent(uint16_t aFcf);
    static bool    IsSrcAddrPresent(uint16_t aFcf) { return (aFcf & kFcfSrcAddrMask) != kFcfSrcAddrNone; }
    static bool    IsSrcPanIdPresent(uint16_t aFcf);
    static bool    IsVersion2015(uint16_t aFcf) { return (aFcf & kFcfFrameVersionMask) == kVersion2015; }

    static uint8_t CalculateAddrFieldSize(uint16_t aFcf);
};

} // namespace Spinel
} // namespace ot

#endif // LIB_SPINEL_MAC_FRAME_HPP_
