/*
 *  Copyright (c) 2021, The OpenThread Authors.
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

/**
 * @file
 *   This file includes definitions for SED-to-SED feature.
 *
 *   Note that this is not a config file. DO NOT add it to core config file.
 *
 */

#ifndef S2S_HPP_
#define S2S_HPP_

#include "openthread-core-config.h"

/**
 * @def OPENTHREAD_MTD_S2S
 *
 * This is a alias representing if `OPENTHREAD_MTD` and `OPENTHREAD_CONFIG_MAC_SSED_TO_SSED_LINK_ENABLE` are both
 * enabled.
 *
 */
#ifdef OPENTHREAD_MTD_S2S
#error \
    "OPENTHREAD_MTD_S2S should not be directly defined. Use `OPENTHREAD_CONFIG_MAC_SSED_TO_SSED_LINK_ENABLE` to enable SSED to SSED link instead."
#endif
#define OPENTHREAD_MTD_S2S (OPENTHREAD_MTD && OPENTHREAD_CONFIG_MAC_SSED_TO_SSED_LINK_ENABLE)

#endif // S2S_HPP_