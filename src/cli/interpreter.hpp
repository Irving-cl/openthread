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

/**
 * @file
 *   This file contains definitions for the Interpreter.
 */

#ifndef INTERPRETER_HPP_
#define INTERPRETER_HPP_

#include <stdarg.h>

#include <openthread/cli.h>
#include <openthread/error.h>
#include <openthread/instance.h>

#include "common/callback.hpp"

namespace ot {
namespace Cli {

class Interpreter
{
public:
    typedef void (*OutputBytesFunc)(const uint8_t *aBytes, uint16_t aLength, void *aFuncContext);
    typedef void (*OutputFormatVFunc)(const char *aFormat, va_list aArguments, void *aFuncContext);
    typedef void (*OutputNewLineFunc)(void *aFuncContext);
    typedef void (*OutputResultFunc)(otError aError, void *aFuncContext);
    typedef void (*ProcessLineFunc)(char *aBuf, void *aFuncContext);
    typedef void (*SetUserCommandsFunc)(const otCliCommand *aUserCommands,
                                        uint8_t             aLength,
                                        void               *aContext,
                                        otError            &aError,
                                        void               *aFuncContext);
    typedef void (*SetEmittingCommandOutputFunc)(bool aEmittingOutput, void *aFuncContext);

    static Interpreter &GetInterpreter(void);

    static void Initialize(otInstance *aInstance, otCliOutputCallback aCallback, void *aContext);

    static bool IsInitialized(void) { return sLineHandler != nullptr; }

    void    OutputBytes(const uint8_t *aBytes, uint16_t aLength);
    void    OutputFormatV(const char *aFormat, va_list aArguments);
    void    OutputNewLine(void);
    void    OutputResult(otError aError);
    void    ProcessLine(char *aBuf);
    otError SetUserCommands(const otCliCommand *aUserCommands, uint8_t aLength, void *aContext);
    void    SetEmittingCommandOutput(bool aEmittingOutput);

private:
    static void *sLineHandler;

    Callback<OutputBytesFunc>              mOutputBytesFunc;
    Callback<OutputFormatVFunc>            mOutputFormatVFunc;
    Callback<OutputNewLineFunc>            mOutputNewLineFunc;
    Callback<OutputResultFunc>             mOutputResultFunc;
    Callback<ProcessLineFunc>              mProcessLineFunc;
    Callback<SetUserCommandsFunc>          mSetUserCommandsFunc;
    Callback<SetEmittingCommandOutputFunc> mSetEmittingCommandOutputFunc;
};

} // namespace Cli
} // namespace ot

#endif // INTERPRETER_HPP_
