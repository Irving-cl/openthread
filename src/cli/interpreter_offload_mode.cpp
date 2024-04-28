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

#include "interpreter_offload_mode.hpp"

#include "common/debug.hpp"
#include "utils/parse_cmdline.hpp"

namespace ot {
namespace Cli {

InterpreterOffloadMode::InterpreterOffloadMode(otCliOutputCallback aCallback, void *aContext)
    : OutputImplementer(aCallback, aContext)
    , Utils(nullptr, *this)
    , mCommandIsPending(false)
{
}

void InterpreterOffloadMode::ProcessLine(char *aBuf)
{
    Arg     args[kMaxArgs + 1];
    otError error = OT_ERROR_NONE;

    OT_ASSERT(aBuf != nullptr);
    VerifyOrExit(!mCommandIsPending, args[0].Clear());
    mCommandIsPending = true;

    SuccessOrExit(error = ot::Utils::CmdLineParser::ParseCmd(aBuf, args, kMaxArgs));

    error = ProcessCommand(args);

exit:
    if ((error != OT_ERROR_NONE) || !args[0].IsEmpty())
    {
        OutputResult(error);
    }
}

void InterpreterOffloadMode::OutputResult(otError aError)
{
    if (aError == OT_ERROR_NONE)
    {
        OutputLine("Done");
    }
    else
    {
        OutputLine("Error %u: %s", aError, otThreadErrorToString(aError));
    }

    OutputPrompt();
}

template <> otError InterpreterOffloadMode::Process<Cmd("countrycode")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("countrycode");
    return error;
}

template <> otError InterpreterOffloadMode::Process<Cmd("channelmasks")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("channelmasks");
    return error;
}

template <> otError InterpreterOffloadMode::Process<Cmd("join")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("join");
    return error;
}

template <> otError InterpreterOffloadMode::Process<Cmd("leave")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("leave");
    return error;
}

template <> otError InterpreterOffloadMode::Process<Cmd("migrate")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("migrate");
    return error;
}

template <> otError InterpreterOffloadMode::Process<Cmd("scan")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("scan");
    return error;
}

template <> otError InterpreterOffloadMode::Process<Cmd("thread")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    (void)aArgs;
    OutputLine("thread");
    return error;
}

otError InterpreterOffloadMode::ProcessCommand(Arg aArgs[])
{
#define CmdEntry(aCommandString)                                              \
    {                                                                         \
        aCommandString, &InterpreterOffloadMode::Process<Cmd(aCommandString)> \
    }

    static constexpr Command kCommands[] = {
        CmdEntry("channelmasks"), CmdEntry("countrycode"), CmdEntry("join"),   CmdEntry("leave"),
        CmdEntry("migrate"),      CmdEntry("scan"),        CmdEntry("thread"),
    };
#undef CmdEntry

    static_assert(BinarySearch::IsSorted(kCommands), "Command Table is not sorted");

    otError        error   = OT_ERROR_NONE;
    const Command *command = BinarySearch::Find(aArgs[0].GetCString(), kCommands);

    if (command != nullptr)
    {
        error = (this->*command->mHandler)(aArgs + 1);
    }
    else if (aArgs[0] == "help")
    {
        OutputCommandTable(kCommands);
    }

    return error;
}

void InterpreterOffloadMode::OutputPrompt(void)
{
#if OPENTHREAD_CONFIG_CLI_PROMPT_ENABLE
    static const char sPrompt[] = "> ";

    // The `OutputFormat()` below is adding the prompt which is not
    // part of any command output, so we set the `EmittingCommandOutput`
    // flag to false to avoid it being included in the command output
    // log (under `OPENTHREAD_CONFIG_CLI_LOG_INPUT_OUTPUT_ENABLE`).

    SetEmittingCommandOutput(false);
    OutputFormat("%s", sPrompt);
    SetEmittingCommandOutput(true);
#endif // OPENTHREAD_CONFIG_CLI_PROMPT_ENABLE
}

} // namespace Cli
} // namespace ot
