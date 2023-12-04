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

#include "platform-posix.h"
#include "platform-posix-offload.h"

#include "common/callback.hpp"
#include "common/error.hpp"
#include "common/new.hpp"
#include "common/logging.hpp"
#include "posix/platform/cp_controller.hpp"
#include "posix/platform/cp_spinel.hpp"
#include "openthread/platform/offload.h"

static ot::Posix::CpController sCpController;
namespace ot {

namespace Posix {

namespace {
extern "C" void platformCpInit(const char *aUrl) { sCpController.Init(aUrl); }
} // namespace


Spinel::RadioSpinel &GetSpinel(void)
{
    return sCpController.GetRadioSpinel();
}

CpController::CpController(void)
    : mRadioUrl(nullptr)
    , mRadioSpinel()
    , mSpinelInterface(nullptr)
{
}

void CpController::Init(const char *aUrl)
{
    bool resetRadio;
    bool skipCompatibilityCheck;

    mRadioUrl = aUrl;
    VerifyOrDie(mRadioUrl.GetPath() != nullptr, OT_EXIT_INVALID_ARGUMENTS);

    mSpinelInterface = CreateSpinelInterface(mRadioUrl.GetProtocol());
    VerifyOrDie(mSpinelInterface != nullptr, OT_EXIT_FAILURE);

    resetRadio             = !mRadioUrl.HasParam("no-reset");
    skipCompatibilityCheck = mRadioUrl.HasParam("skip-rcp-compatibility-check");
    mRadioSpinel.Init(*mSpinelInterface, resetRadio, skipCompatibilityCheck);
}

Spinel::SpinelInterface *CpController::CreateSpinelInterface(const char *aInterfaceName)
{
    Spinel::SpinelInterface *interface;

    if (aInterfaceName == nullptr)
    {
        DieNow(OT_ERROR_FAILED);
    }
#if OPENTHREAD_POSIX_CONFIG_SPINEL_HDLC_INTERFACE_ENABLE
    else if (HdlcInterface::IsInterfaceNameMatch(aInterfaceName))
    {
        interface = new (&mSpinelInterfaceRaw) HdlcInterface(mRadioUrl);
    }
#endif
#if OPENTHREAD_POSIX_CONFIG_SPINEL_SPI_INTERFACE_ENABLE
    else if (Posix::SpiInterface::IsInterfaceNameMatch(aInterfaceName))
    {
        interface = new (&mSpinelInterfaceRaw) SpiInterface(mRadioUrl);
    }
#endif
#if OPENTHREAD_POSIX_CONFIG_SPINEL_VENDOR_INTERFACE_ENABLE
    else if (VendorInterface::IsInterfaceNameMatch(aInterfaceName))
    {
        interface = new (&mSpinelInterfaceRaw) VendorInterface(mRadioUrl);
    }
#endif
    else
    {
        otLogCritPlat("The Spinel interface name \"%s\" is not supported!", aInterfaceName);
        DieNow(OT_ERROR_FAILED);
    }

    return interface;
}

} // namespace Posix
} // namespace ot

static ot::Spinel::RadioSpinel &GetRadioSpinel(void) { return sCpController.GetRadioSpinel(); }

const char *otPlatCpGetVersionString(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetRadioSpinel().GetVersion();
}

otError otPlatCpIssueActiveScan(otInstance *aInstance,
                         uint32_t                 aScanChannels,
                         uint16_t                 aScanDuration)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetRadioSpinel().ActiveScan(aScanChannels, aScanDuration);
}

void platformCpProcess(otInstance *aInstance, const otSysMainloopContext *aContext)
{
    OT_UNUSED_VARIABLE(aInstance);

    GetRadioSpinel().ProcessCp(aContext);
}

void platformCpUpdateFdSet(otSysMainloopContext *aContext)
{
    sCpController.GetSpinelInterface().UpdateFdSet(aContext);
}

void otPlatCpEnable(otInstance *aInstance)
{
    GetRadioSpinel().EnableCp(aInstance);
}

otDeviceRole otPlatCpGetDeviceRole(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    otDeviceRole role;

    GetRadioSpinel().GetDeviceRole(role);

    return role;
}

otError otPlatCpDatasetInitNew(otInstance *aInstance, otOperationalDataset *aDataset)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetRadioSpinel().DatasetInitNew(aDataset);
}

otError otPlatCpThreadStartStop(otInstance *aInstance, bool aStart)
{
    OT_UNUSED_VARIABLE(aInstance);
    return GetRadioSpinel().ThreadStartStop(aStart);
}

uint8_t otPlatCpThreadGetRouterSelectionJitter(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    uint8_t jitter = 0;

    GetRadioSpinel().ThreadGetRouterSelectionJitter(jitter);

    return jitter;
}

void otPlatCpThreadSetRouterSelectionJitter(otInstance *aInstance, uint8_t aRouterJitter)
{
    OT_UNUSED_VARIABLE(aInstance);
    GetRadioSpinel().ThreadSetRouterSelectionJitter(aRouterJitter);
}

otError platformNetifOffloadSendIp6(uint8_t *aBuf, uint16_t aLen)
{
    otError error = OT_ERROR_NONE;

    error = GetRadioSpinel().Ip6Send(aBuf, aLen);

    return error;
}

otError otPlatCpFactoryReset(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;
    GetRadioSpinel().FactoryResetNcp();     

    return error;
}

otError otPlatCpGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    VerifyOrExit(aCoexMetrics != nullptr, error = OT_ERROR_INVALID_ARGS);

    error = GetRadioSpinel().GetCoexMetrics(*aCoexMetrics);

exit:
    return error;
}

const otRadioSpinelMetrics *otPlatCpGetRadioSpinelMetrics(void)
{
    return GetRadioSpinel().GetRadioSpinelMetrics();
}

const otRcpInterfaceMetrics *otPlatCpGetNcpInterfaceMetrics(void)
{
    return sCpController.GetSpinelInterface().GetRcpInterfaceMetrics();
}

otError otPlatIp6SetEnabled(bool aEnabled)
{
    return GetRadioSpinel().Ip6SetEnabled(aEnabled);
}

void platformCpDeinit(void)
{
    GetRadioSpinel().Deinit();
}

otError otPlatCpGetIeeeEui64(uint8_t *aIeeeEui64)
{
    return GetRadioSpinel().GetIeeeEui64(aIeeeEui64);
}

otError otPlatCpSetStateChangedCallback(otStateChangedCallback aCallback, void *aContext)
{
    return GetRadioSpinel().RegisterCallback(aCallback, aContext);
}

otError otPlatCpGetNeighborTable(otNeighborInfo *aNeighborList, uint8_t &aCount)
{
    return GetRadioSpinel().ThreadGetNeighborTable(aNeighborList, aCount);
}

otError otPlatCpGetChildTable(otChildInfo *aChildList, uint8_t &aCount)
{
    return GetRadioSpinel().ThreadGetChildTable(aChildList, aCount);
}

otError otPlatCpDatasetGetActiveTlvs(otInstance *aInstance, otOperationalDatasetTlvs *aDataset)
{
    (void)aInstance;
    return GetRadioSpinel().DatasetGetActiveTlvs(aDataset);
}

otError otPlatCpDatasetGetPending(otInstance *aInstance, otOperationalDataset *aDataset)
{
    (void)aInstance;
    return GetRadioSpinel().DatasetGetPending(aDataset);
}    

otError otPlatCpDatasetGetPendingTlvs(otInstance *aInstance, otOperationalDatasetTlvs *aDataset)
{
    (void)aInstance;
    return GetRadioSpinel().DatasetGetPendingTlvs(aDataset);
}

otError otPlatCpThreadGetNetworkKey(otNetworkKey *aNetworkKey)
{
    return GetRadioSpinel().ThreadGetNetworkKey(aNetworkKey);
}

otError otPlatCpDatasetSetActive(otOperationalDataset *aDataset)
{
    return GetRadioSpinel().DatasetSetActive(aDataset);
}

otError otPlatCpDatasetSetPending(otOperationalDataset *aDataset)
{
    return GetRadioSpinel().DatasetSetPending(aDataset);
}