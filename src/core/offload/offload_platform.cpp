/*
 *    Copyright (c) 2023, The OpenThread Authors.
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the radio platform callbacks into OpenThread and default/weak radio platform APIs.
 */

#include <openthread/instance.h>
#include <openthread/platform/offload.h>

#include "common/as_core_type.hpp"
#include "common/code_utils.hpp"
#include "common/notifier.hpp"
#include "instance/instance.hpp"
#include "instance/offload.hpp"

using namespace ot;

extern "C" void otPlatCpActiveScanDone(otInstance *aInstance, otActiveScanResult *aScanResult)
{
    Instance &instance = AsCoreType(aInstance);

    instance.Get<Offload>().ActiveScanDone(aScanResult);

    return;
}


extern "C" void otPlatCpSignalEvent(otInstance *aInstance, uint32_t aEvent)
{
    Instance &instance = AsCoreType(aInstance);

    instance.Get<Notifier>().Signal(static_cast<Event>(aEvent));
    return;
}


extern "C" void otPlatOffloadDataUpdateIPv6AddressTable(otInstance *aInstance, otNetifAddress *aAddressList, uint8_t aCount)
{
    Instance &instance = AsCoreType(aInstance);

    instance.Get<Offload>().DataUpdateIPv6AddressTable(static_cast<Ip6::Netif::UnicastAddress *>(aAddressList), aCount);
    return;
}

