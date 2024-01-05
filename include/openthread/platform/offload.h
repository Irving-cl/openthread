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

#ifndef OPENTHREAD_PLATFORM_OFFLOAD_H_
#define OPENTHREAD_PLATFORM_OFFLOAD_H_

#include <openthread/dataset.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/netdata.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <lib/spinel/radio_spinel_metrics.h>

#ifdef __cplusplus
extern "C" {
#endif

void otPlatCpEnable(otInstance *aInstance);

const char *otPlatCpGetVersionString(otInstance *aInstance);

otError otPlatCpIssueActiveScan(otInstance *aInstance,
                                uint32_t                 aScanChannels,
                                uint16_t                 aScanDuration);

void otPlatCpActiveScanDone(otInstance *aInstance,
                               otActiveScanResult *aScanResult);

otDeviceRole otPlatCpGetDeviceRole(otInstance *aInstance);

otDeviceRole otPlatCpGetDeviceRoleCached(otInstance *aInstance);

otError otPlatCpDatasetInitNew(otInstance *aInstance, otOperationalDataset *aDataset);

otError otPlatCpThreadStartStop(otInstance *aInstance, bool aStart);

uint8_t otPlatCpThreadGetRouterSelectionJitter(otInstance *aInstance);

void otPlatCpThreadSetRouterSelectionJitter(otInstance *aInstance, uint8_t aRouterJitter);

otError otPlatCpThreadGetNetworkKey(otNetworkKey *aNetworkKey);

otError otPlatCpFactoryReset(otInstance *aInstance);

otError otPlatCpGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics);

const otRadioSpinelMetrics *otPlatCpGetRadioSpinelMetrics(void);

const otRcpInterfaceMetrics *otPlatCpGetNcpInterfaceMetrics(void);

otError otPlatIp6SetEnabled(bool aEnabled);

otError otPlatCpGetIeeeEui64(uint8_t *aIeeeEui64);

otError otPlatCpSetStateChangedCallback(otStateChangedCallback aCallback, void *aContext);

otError otPlatCpGetNeighborTable(otNeighborInfo *aNeighborList, uint8_t &aCount);

otError otPlatCpGetChildTable(otChildInfo *aChildList, uint8_t &aCount);

otError otPlatCpDatasetGetActiveTlvs(otInstance *aInstance, otOperationalDatasetTlvs *aDataset);

otError otPlatCpDatasetGetPending(otInstance *aInstance, otOperationalDataset *aDataset);

otError otPlatCpDatasetGetPendingTlvs(otInstance *aInstance, otOperationalDatasetTlvs *aDataset);

otError otPlatCpDatasetSetActive(otOperationalDataset *aDataset);

otError otPlatCpDatasetSetPending(otOperationalDataset *aDataset);

otError otPlatCpBorderRouterAddOnMeshPrefix(const otBorderRouterConfig *aConfig);

otError otPlatCpBorderRouterRemoveOnMeshPrefix(const otIp6Prefix *aIp6Prefix);

void otPlatCpThreadGetExtendedPanIdCached(otExtendedPanId *extPanId);

otError otPlatCpNetDataUnpublishPrefix(otIp6Prefix* aIp6Prefix);

otError otPlatCpNetDataPublishExternalRoute(otExternalRouteConfig *aConfig);

otError otPlatCpNetDataReplacePublishedExternalRoute(otIp6Prefix *aPrefix, otExternalRouteConfig *aConfig);

otError otPlatCpNetDataGetOnMeshPrefix(otBorderRouterConfig *aConfigList, uint8_t &aCount);

otError otPlatCpNetDataGetExternalRouteConfig(otExternalRouteConfig *aConfigList, uint8_t &aCount);

// ----------------------------------------------------------------------------------------------------
// Platform call core
void otPlatCpSignalEvent(otInstance *aInstance, uint32_t aEvent);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_PLATFORM_OFFLOAD_H_
