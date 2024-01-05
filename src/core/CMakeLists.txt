#
#  Copyright (c) 2019, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

set(COMMON_INCLUDES
    ${PROJECT_SOURCE_DIR}/src/core
)

set(COMMON_SOURCES
    api/border_routing_offload_api.cpp
    api/dataset_api.cpp
    api/error_api.cpp
    api/instance_api.cpp
    api/ip6_api.cpp
    api/logging_api.cpp
    api/offload_api.cpp
    api/tasklet_api.cpp
    api/thread_api.cpp
    border_router/infra_if_offload.cpp
    border_router/routing_manager_offload.cpp
    common/binary_search.cpp
    common/error.cpp
    common/heap.cpp
    common/log.cpp
    common/notifier.cpp
    common/random.cpp
    common/settings.cpp
    common/string.cpp
    common/tasklet.cpp
    common/timer.cpp
    common/tlvs.cpp
    common/preference.cpp
    common/uptime.cpp
    crypto/crypto_platform.cpp
    crypto/mbedtls.cpp
    instance/instance.cpp
    instance/offload.cpp
    mac/mac_types.cpp
    meshcop/dataset.cpp
    meshcop/meshcop.cpp
    meshcop/meshcop_tlvs.cpp
    meshcop/network_name.cpp
    meshcop/timestamp.cpp
    net/ip4_types.cpp
    net/ip6_address.cpp
    net/nd6.cpp
    net/netif.cpp
    offload/offload_platform.cpp
    thread/key_manager.cpp
    thread/mle_types.cpp
    utils/heap.cpp
    utils/parse_cmdline.cpp
)

set(RADIO_COMMON_SOURCES
    api/diags_api.cpp
    api/error_api.cpp
    api/instance_api.cpp
    api/link_raw_api.cpp
    api/logging_api.cpp
    api/random_noncrypto_api.cpp
    api/tasklet_api.cpp
    common/binary_search.cpp
    common/error.cpp
    common/frame_builder.cpp
    common/log.cpp
    common/random.cpp
    common/string.cpp
    common/tasklet.cpp
    common/timer.cpp
    common/uptime.cpp
    crypto/aes_ccm.cpp
    crypto/aes_ecb.cpp
    crypto/crypto_platform.cpp
    crypto/storage.cpp
    diags/factory_diags.cpp
    instance/instance.cpp
    mac/link_raw.cpp
    mac/mac_frame.cpp
    mac/mac_types.cpp
    mac/sub_mac.cpp
    mac/sub_mac_callbacks.cpp
    radio/radio.cpp
    radio/radio_callbacks.cpp
    radio/radio_platform.cpp
    thread/link_quality.cpp
    utils/parse_cmdline.cpp
    utils/power_calibration.cpp
)

set(OT_VENDOR_EXTENSION "" CACHE STRING "specify a C++ source file built as part of OpenThread core library")
if(OT_VENDOR_EXTENSION)
  target_compile_definitions(ot-config INTERFACE "OPENTHREAD_ENABLE_VENDOR_EXTENSION=1")
  list(APPEND COMMON_SOURCES ${OT_VENDOR_EXTENSION})
endif()

if(OT_FTD)
    include(ftd.cmake)
endif()

if(OT_MTD)
    include(mtd.cmake)
endif()

if(OT_RCP)
    include(radio.cmake)
    if (OT_APP_CLI)
        include(radio_cli.cmake)
    endif()
endif()

set_property(SOURCE api/instance_api.cpp
    APPEND PROPERTY COMPILE_DEFINITIONS "PACKAGE_VERSION=\"${OT_PACKAGE_VERSION}\""
)
