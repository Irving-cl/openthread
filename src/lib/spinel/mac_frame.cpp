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

#include "lib/spinel/mac_frame.hpp"

#include <assert.h>

#include <lib/utils/utils.hpp>

namespace ot {
namespace Spinel {

uint16_t MacFrame::GetFrameControlField(void) const { return LittleEndian::ReadUint16(mPsdu); }

void MacFrame::SetKeyId(uint8_t aKeyId)
{
    uint8_t keySourceLength;
    uint8_t index = FindSecurityHeaderIndex();

    assert(index != kInvalidIndex);

    keySourceLength = GetKeySourceLength(mPsdu[index] & kKeyIdModeMask);

    mPsdu[index + kSecurityControlSize + kFrameCounterSize + keySourceLength] = aKeyId;
}

void MacFrame::SetFrameCounter(uint32_t aFrameCounter)
{
    uint8_t index = FindSecurityHeaderIndex();

    assert(index != kInvalidIndex);

    // Security Control
    index += kSecurityControlSize;

    LittleEndian::WriteUint32(aFrameCounter, &mPsdu[index]);

    SetIsHeaderUpdated(true);
}

uint8_t MacFrame::GetKeySourceLength(uint8_t aKeyIdMode)
{
    uint8_t len = 0;

    switch (aKeyIdMode)
    {
    case kKeyIdMode0:
        len = kKeySourceSizeMode0;
        break;

    case kKeyIdMode1:
        len = kKeySourceSizeMode1;
        break;

    case kKeyIdMode2:
        len = kKeySourceSizeMode2;
        break;

    case kKeyIdMode3:
        len = kKeySourceSizeMode3;
        break;
    }

    return len;
}

uint8_t MacFrame::FindSecurityHeaderIndex(void) const
{
    uint8_t index;

    EXPECT(kFcfSize < mLength, index = kInvalidIndex);
    EXPECT(GetSecurityEnabled(), index = kInvalidIndex);
    index = SkipAddrFieldIndex();

exit:
    return index;
}

uint8_t MacFrame::SkipAddrFieldIndex(void) const
{
    uint8_t index;

    EXPECT(kFcfSize + kDsnSize + GetFcsSize() <= mLength, index = kInvalidIndex);

    index = CalculateAddrFieldSize(GetFrameControlField());

exit:
    return index;
}

uint8_t MacFrame::CalculateAddrFieldSize(uint16_t aFcf)
{
    uint8_t size = kFcfSize + kDsnSize;

    // This static method calculates the size (number of bytes) of
    // Address header field for a given Frame Control `aFcf` value.
    // The size includes the Frame Control and Sequence Number fields
    // along with Destination and Source PAN ID and Short/Extended
    // Addresses. If the `aFcf` is not valid, this method returns
    // `kInvalidSize`.

    if (IsDstPanIdPresent(aFcf))
    {
        size += sizeof(otPanId);
    }

    switch (aFcf & kFcfDstAddrMask)
    {
    case kFcfDstAddrNone:
        break;

    case kFcfDstAddrShort:
        size += sizeof(otShortAddress);
        break;

    case kFcfDstAddrExt:
        size += sizeof(otExtAddress);
        break;

    default:
        EXIT_NOW(size = kInvalidSize);
    }

    if (IsSrcPanIdPresent(aFcf))
    {
        size += sizeof(otPanId);
    }

    switch (aFcf & kFcfSrcAddrMask)
    {
    case kFcfSrcAddrNone:
        break;

    case kFcfSrcAddrShort:
        size += sizeof(otShortAddress);
        break;

    case kFcfSrcAddrExt:
        size += sizeof(otExtAddress);
        break;

    default:
        EXIT_NOW(size = kInvalidSize);
    }

exit:
    return size;
}

bool MacFrame::IsDstPanIdPresent(uint16_t aFcf)
{
    bool present = true;

    if (IsVersion2015(aFcf))
    {
        // Original table at `InitMacHeader()`
        //
        // +----+--------------+--------------+--------------++--------------+
        // | No |  Dest Addr   |   Src Addr   |  PAN ID Comp ||   Dst PAN ID |
        // +----+--------------+--------------+--------------++--------------+
        // |  1 | Not Present  | Not Present  |      0       || Not Present  |
        // |  2 | Not Present  | Not Present  |      1       || Present      |
        // |  3 | Present      | Not Present  |      0       || Present      |
        // |  4 | Present      | Not Present  |      1       || Not Present  |
        // |  5 | Not Present  | Present      |      0       || Not Present  |
        // |  6 | Not Present  | Present      |      1       || Not Present  |
        // +----+--------------+--------------+--------------++--------------+
        // |  7 | Extended     | Extended     |      0       || Present      |
        // |  8 | Extended     | Extended     |      1       || Not Present  |
        // |----+--------------+--------------+--------------++--------------+
        // |  9 | Short        | Short        |      0       || Present      |
        // | 10 | Short        | Extended     |      0       || Present      |
        // | 11 | Extended     | Short        |      0       || Present      |
        // | 12 | Short        | Extended     |      1       || Present      |
        // | 13 | Extended     | Short        |      1       || Present      |
        // | 14 | Short        | Short        |      1       || Present      |
        // +----+--------------+--------------+--------------++--------------+

        switch (aFcf & (kFcfDstAddrMask | kFcfSrcAddrMask | kFcfPanidCompression))
        {
        case (kFcfDstAddrNone | kFcfSrcAddrNone):                         // 1
        case (kFcfDstAddrShort | kFcfSrcAddrNone | kFcfPanidCompression): // 4 (short dst)
        case (kFcfDstAddrExt | kFcfSrcAddrNone | kFcfPanidCompression):   // 4 (ext dst)
        case (kFcfDstAddrNone | kFcfSrcAddrShort):                        // 5 (short src)
        case (kFcfDstAddrNone | kFcfSrcAddrExt):                          // 5 (ext src)
        case (kFcfDstAddrNone | kFcfSrcAddrShort | kFcfPanidCompression): // 6 (short src)
        case (kFcfDstAddrNone | kFcfSrcAddrExt | kFcfPanidCompression):   // 6 (ext src)
        case (kFcfDstAddrExt | kFcfSrcAddrExt | kFcfPanidCompression):    // 8
            present = false;
            break;
        default:
            break;
        }
    }
    else
    {
        present = IsDstAddrPresent(aFcf);
    }

    return present;
}

bool MacFrame::IsSrcPanIdPresent(uint16_t aFcf)
{
    bool present = IsSrcAddrPresent(aFcf) && ((aFcf & kFcfPanidCompression) == 0);

    // Special case for a IEEE 802.15.4-2015 frame: When both
    // addresses are extended, then the source PAN iD is not present
    // independent of PAN ID Compression. In this case, if the PAN ID
    // compression is set, it indicates that no PAN ID is in the
    // frame, while if the PAN ID Compression is zero, it indicates
    // the presence of the destination PAN ID in the frame.
    //
    // +----+--------------+--------------+--------------++--------------+
    // | No |  Dest Addr   |   Src Addr   |  PAN ID Comp ||  Src PAN ID  |
    // +----+--------------+--------------+--------------++--------------+
    // |  1 | Not Present  | Not Present  |      0       || Not Present  |
    // |  2 | Not Present  | Not Present  |      1       || Not Present  |
    // |  3 | Present      | Not Present  |      0       || Not Present  |
    // |  4 | Present      | Not Present  |      1       || Not Present  |
    // |  5 | Not Present  | Present      |      0       || Present      |
    // |  6 | Not Present  | Present      |      1       || Not Present  |
    // +----+--------------+--------------+--------------++--------------+
    // |  7 | Extended     | Extended     |      0       || Not Present  |
    // |  8 | Extended     | Extended     |      1       || Not Present  |
    // |----+--------------+--------------+--------------++--------------+
    // |  9 | Short        | Short        |      0       || Present      |
    // | 10 | Short        | Extended     |      0       || Present      |
    // | 11 | Extended     | Short        |      0       || Present      |
    // | 12 | Short        | Extended     |      1       || Not Present  |
    // | 13 | Extended     | Short        |      1       || Not Present  |
    // | 14 | Short        | Short        |      1       || Not Present  |
    // +----+--------------+--------------+--------------++--------------+

    if (IsVersion2015(aFcf) && ((aFcf & (kFcfDstAddrMask | kFcfSrcAddrMask)) == (kFcfDstAddrExt | kFcfSrcAddrExt)))
    {
        present = false;
    }

    return present;
}

} // namespace Spinel
} // namespace ot
