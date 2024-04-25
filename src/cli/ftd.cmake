#
#  Copyright (c) 2020, The OpenThread Authors.
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

add_library(openthread-cli-ftd)

target_compile_definitions(openthread-cli-ftd
    PRIVATE
        OPENTHREAD_FTD=1
        OPENTHREAD_MTD=0
        OPENTHREAD_RADIO=0
)

target_compile_options(openthread-cli-ftd PRIVATE
    ${OT_CFLAGS}
)

target_include_directories(openthread-cli-ftd
    PUBLIC
    ${OT_PUBLIC_INCLUDES}
    PRIVATE
    ${COMMON_INCLUDES}
    ${PROJECT_SOURCE_DIR}/src/posix/platform/include
)

target_sources(openthread-cli-ftd PRIVATE ${COMMON_SOURCES})

target_link_libraries(openthread-cli-ftd
    PUBLIC
        openthread-ftd
    PRIVATE
        ${OT_MBEDTLS}
        ot-config-ftd
        ot-config
)

if(OT_CLI_VENDOR_TARGET)
    target_link_libraries(openthread-cli-ftd PRIVATE ${OT_CLI_VENDOR_TARGET})
endif()
