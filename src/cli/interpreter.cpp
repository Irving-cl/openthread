/* *  Copyright (c) 2024, The OpenThread Authors.
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

#include "interpreter.hpp"

#include <openthread/cli.h>

#include "cli.hpp"

namespace ot {
namespace Cli {

static Interpreter sInterpreter;
static OT_DEFINE_ALIGNED_VAR(sInterpreterRaw, sizeof(InterpreterCoreMode), uint64_t);

void *Interpreter::sLineHandler = nullptr;

Interpreter &Interpreter::GetInterpreter(void) { return sInterpreter; }

void Interpreter::Initialize(otInstance *aInstance, otCliOutputCallback aCallback, void *aContext)
{
    (void)sInterpreterRaw;
    (void)aInstance;
    (void)aCallback;
    (void)aContext;
}

void Interpreter::OutputBytes(const uint8_t *aBytes, uint16_t aLength)
{
    mOutputBytesFunc.InvokeIfSet(aBytes, aLength);
}

void Interpreter::OutputFormatV(const char *aFormat, va_list aArguments)
{
    mOutputFormatVFunc.InvokeIfSet(aFormat, aArguments);
}

void Interpreter::OutputNewLine(void) { mOutputNewLineFunc.InvokeIfSet(); }

void Interpreter::OutputResult(otError aError) { mOutputResultFunc.InvokeIfSet(aError); }

void Interpreter::ProcessLine(char *aBuf) { mProcessLineFunc.InvokeIfSet(aBuf); }

otError Interpreter::SetUserCommands(const otCliCommand *aUserCommands, uint8_t aLength, void *aContext)
{
    otError error;

    mSetUserCommandsFunc.InvokeIfSet(aUserCommands, aLength, aContext, error);

    return error;
}

void Interpreter::SetEmittingCommandOutput(bool aEmittingOutput)
{
    mSetEmittingCommandOutputFunc.InvokeIfSet(aEmittingOutput);
}

extern "C" void otCliInit(otInstance *aInstance, otCliOutputCallback aCallback, void *aContext)
{
    Interpreter::Initialize(aInstance, aCallback, aContext);

#if OPENTHREAD_CONFIG_CLI_VENDOR_COMMANDS_ENABLE && OPENTHREAD_CONFIG_CLI_MAX_USER_CMD_ENTRIES > 1
    otCliVendorSetUserCommands();
#endif
}

extern "C" void otCliInputLine(char *aBuf) { Interpreter::GetInterpreter().ProcessLine(aBuf); }

extern "C" otError otCliSetUserCommands(const otCliCommand *aUserCommands, uint8_t aLength, void *aContext)
{
    return Interpreter::GetInterpreter().SetUserCommands(aUserCommands, aLength, aContext);
}

extern "C" void otCliOutputBytes(const uint8_t *aBytes, uint8_t aLength)
{
    Interpreter::GetInterpreter().OutputBytes(aBytes, aLength);
}

extern "C" void otCliOutputFormat(const char *aFmt, ...)
{
    va_list aAp;
    va_start(aAp, aFmt);
    Interpreter::GetInterpreter().OutputFormatV(aFmt, aAp);
    va_end(aAp);
}

extern "C" void otCliAppendResult(otError aError) { Interpreter::GetInterpreter().OutputResult(aError); }

extern "C" void otCliPlatLogv(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, va_list aArgs)
{
    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);

    VerifyOrExit(Interpreter::IsInitialized());

    // CLI output is being used for logging, so we set the flag
    // `EmittingCommandOutput` to false indicate this.
    Interpreter::GetInterpreter().SetEmittingCommandOutput(false);
    Interpreter::GetInterpreter().OutputFormatV(aFormat, aArgs);
    Interpreter::GetInterpreter().OutputNewLine();
    Interpreter::GetInterpreter().SetEmittingCommandOutput(true);

exit:
    return;
}

} // namespace Cli
} // namespace ot
