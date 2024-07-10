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

#ifndef MAX_POWER_TABLE_HPP_
#define MAX_POWER_TABLE_HPP_

#include <cstring>

#include <openthread/platform/radio.h>

namespace ot {
namespace Spinel {

class MaxPowerTable
{
public:
    static constexpr uint8_t kChannelMin   = OPENTHREAD_SPINEL_CONFIG_RADIO_CHANNEL_MIN;
    static constexpr uint8_t kChannelMax   = OPENTHREAD_SPINEL_CONFIG_RADIO_CHANNEL_MAX;
    static constexpr int8_t  kPowerDefault = 30; ///< Default power 1 watt (30 dBm).

    MaxPowerTable(void) { memset(mPowerTable, kPowerDefault, sizeof(mPowerTable)); }

    /**
     * Gets the max allowed transmit power of channel @p aChannel.
     *
     * @param[in]  aChannel    The radio channel number.
     *
     * @returns The max supported transmit power in dBm.
     *
     */
    int8_t GetTransmitPower(uint8_t aChannel) const { return mPowerTable[aChannel - kChannelMin]; }

    /**
     * Sets the max allowed transmit power of channel @p aChannel.
     *
     * @param[in]  aChannel    The radio channel number.
     * @param[in]  aPower      The max supported transmit power in dBm.
     *
     */
    void SetTransmitPower(uint8_t aChannel, int8_t aPower) { mPowerTable[aChannel - kChannelMin] = aPower; }

    /**
     * Gets the supported channel masks.
     *
     */
    uint32_t GetSupportedChannelMask(void) const
    {
        uint32_t channelMask = 0;

        for (uint8_t i = kChannelMin; i <= kChannelMax; ++i)
        {
            if (mPowerTable[i - kChannelMin] != OT_RADIO_POWER_INVALID)
            {
                channelMask |= (1 << i);
            }
        }

        return channelMask;
    }

private:
    int8_t mPowerTable[kChannelMax - kChannelMin + 1];
};

} // namespace Spinel
} // namespace ot

#endif // MAX_POWER_TABLE_HPP_
