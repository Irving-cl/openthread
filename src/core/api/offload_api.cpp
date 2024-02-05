/*
 *  Copyright (c) 2023, The OpenThread Authors.
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

#include "openthread-core-config.h"

#include <openthread/offload.h>

#include "common/as_core_type.hpp"
#include "common/locator_getters.hpp"
#include "instance/instance.hpp"

using namespace ot;

otError otOffloadActiveScan(otInstance              *aInstance,
                         uint32_t                 aScanChannels,
                         uint16_t                 aScanDuration,
                         otHandleActiveScanResult aCallback,
                         void                    *aCallbackContext)
{
    return AsCoreType(aInstance).Get<Offload>().ActiveScan(aScanChannels, aScanDuration, aCallback, aCallbackContext);
}

otDeviceRole otOffloadThreadGetDeviceRole(otInstance *aInstance)
{
    return MapEnum(AsCoreType(aInstance).Get<Offload>().GetDeviceRole());
}

otError otOffloadDatasetInitNew(otInstance *aInstance, otOperationalDataset *aDataset)
{
    return AsCoreType(aInstance).Get<Offload>().DatasetInitNew(*aDataset);
}

otError otOffloadThreadStartStop(otInstance *aInstance, bool aStart)
{
    return AsCoreType(aInstance).Get<Offload>().ThreadStartStop(aStart);
}

void otOffloadSrpServerSetServiceUpdateHandler(otInstance *aInstance,
                                               otSrpServerServiceUpdateHandler aServiceHandler,
                                               void                           *aContext)
{
    return AsCoreType(aInstance).Get<Offload>().SetServiceUpdateHandler(aServiceHandler, aContext);
}
