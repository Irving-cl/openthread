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

#include "platform-posix.h"

#include "posix/platform/spinel_plat.hpp"

#include "common/code_utils.hpp"
#include "common/new.hpp"
#include "lib/spinel/spinel_base.hpp"
#include "posix/platform/hdlc_interface.hpp"
#include "posix/platform/radio_url.hpp"
#include "posix/platform/spi_interface.hpp"
#include "posix/platform/vendor_interface.hpp"

namespace ot {
namespace Posix {

SpinelPlat &GetSpinel(void)
{
    static SpinelPlat sSpinelPlat;
    return sSpinelPlat;
}

namespace {
extern "C" PosixSpinelMode platformSpinelInit(const char *aUrl) { return GetSpinel().Init(aUrl); }
} // namespace

SpinelPlat::SpinelPlat(void)
    : mUrl(nullptr)
    , mSpinelBase()
    , mSpinelInterface(nullptr)
{
}

PosixSpinelMode SpinelPlat::Init(const char *aUrl)
{
    bool resetRadio;
    bool skipCompatibilityCheck;

    mUrl = aUrl;
    VerifyOrDie(mUrl.GetPath() != nullptr, OT_EXIT_INVALID_ARGUMENTS);

    mSpinelInterface = CreateSpinelInterface(mUrl.GetProtocol());
    VerifyOrDie(mSpinelInterface != nullptr, OT_EXIT_FAILURE);

    resetRadio             = !mUrl.HasParam("no-reset");
    skipCompatibilityCheck = mUrl.HasParam("skip-rcp-compatibility-check");
    mSpinelBase.Init(*mSpinelInterface, resetRadio, skipCompatibilityCheck);

    ProcessRadioUrl(mUrl);

    return PosixSpinelMode::RCP;
}

void SpinelPlat::ProcessRadioUrl(const RadioUrl &aRadioUrl)
{
    (void)aRadioUrl;
}

Spinel::SpinelInterface *SpinelPlat::CreateSpinelInterface(const char *aInterfaceName)
{
    Spinel::SpinelInterface *interface;

    if (aInterfaceName == nullptr)
    {
        DieNow(OT_ERROR_FAILED);
    }
#if OPENTHREAD_POSIX_CONFIG_SPINEL_HDLC_INTERFACE_ENABLE
    else if (HdlcInterface::IsInterfaceNameMatch(aInterfaceName))
    {
        interface = new (&mSpinelInterfaceRaw) HdlcInterface(mUrl);
    }
#endif
#if OPENTHREAD_POSIX_CONFIG_SPINEL_SPI_INTERFACE_ENABLE
    else if (Posix::SpiInterface::IsInterfaceNameMatch(aInterfaceName))
    {
        interface = new (&mSpinelInterfaceRaw) SpiInterface(mUrl);
    }
#endif
#if OPENTHREAD_POSIX_CONFIG_SPINEL_VENDOR_INTERFACE_ENABLE
    else if (VendorInterface::IsInterfaceNameMatch(aInterfaceName))
    {
        interface = new (&mSpinelInterfaceRaw) VendorInterface(mUrl);
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
