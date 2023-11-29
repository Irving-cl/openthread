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

#ifndef POSIX_PLATFORM_CP_CONTROLLER_HPP_
#define POSIX_PLATFORM_CP_CONTROLLER_HPP_

#include "common/code_utils.hpp"
#include "common/error.hpp"
#include "lib/spinel/radio_spinel.hpp"
#include "posix/platform/hdlc_interface.hpp"
#include "posix/platform/radio_url.hpp"
#include "posix/platform/spi_interface.hpp"
#include "posix/platform/vendor_interface.hpp"

namespace ot {
namespace Posix {

class CpController
{
public:

    CpController(void);

    void Init(const char *aUrl);

    Spinel::SpinelInterface &GetSpinelInterface(void)
    {
        OT_ASSERT(mSpinelInterface != nullptr);
        return *mSpinelInterface;
    }

    Spinel::RadioSpinel &GetRadioSpinel(void) { return mRadioSpinel; }

private:
    Spinel::SpinelInterface *CreateSpinelInterface(const char *aInterfaceName);

#if OPENTHREAD_POSIX_CONFIG_SPINEL_HDLC_INTERFACE_ENABLE && OPENTHREAD_POSIX_CONFIG_SPINEL_SPI_INTERFACE_ENABLE
    static constexpr size_t kSpinelInterfaceRawSize = sizeof(ot::Posix::SpiInterface) > sizeof(ot::Posix::HdlcInterface)
                                                          ? sizeof(ot::Posix::SpiInterface)
                                                          : sizeof(ot::Posix::HdlcInterface);
#elif OPENTHREAD_POSIX_CONFIG_SPINEL_HDLC_INTERFACE_ENABLE
    static constexpr size_t kSpinelInterfaceRawSize = sizeof(ot::Posix::HdlcInterface);
#elif OPENTHREAD_POSIX_CONFIG_SPINEL_SPI_INTERFACE_ENABLE
    static constexpr size_t kSpinelInterfaceRawSize = sizeof(ot::Posix::SpiInterface);
#elif OPENTHREAD_POSIX_CONFIG_SPINEL_VENDOR_INTERFACE_ENABLE
    static constexpr size_t kSpinelInterfaceRawSize = sizeof(ot::Posix::VendorInterface);
#else
#error "No Spinel interface is specified!"
#endif

    RadioUrl                 mRadioUrl;
    Spinel::RadioSpinel      mRadioSpinel;
    Spinel::SpinelInterface *mSpinelInterface;

    OT_DEFINE_ALIGNED_VAR(mSpinelInterfaceRaw, kSpinelInterfaceRawSize, uint64_t);
};

} // namespace Posix
} // namespace ot

#endif // POSIX_PLATFORM_CP_CONTROLLER_HPP_
