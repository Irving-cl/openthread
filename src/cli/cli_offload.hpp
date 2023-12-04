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

#ifndef CLI_OFFLOAD_HPP_
#define CLI_OFFLOAD_HPP_

#include "openthread-core-config.h"

#include "cli/cli_config.h"
#include "cli/cli_output.hpp"

#include "openthread/link.h"

namespace ot {
namespace Cli {

class CliOffload : public Output
{
public:
    using Arg = Utils::CmdLineParser::Arg;

    CliOffload(otInstance *aInstance, OutputImplementer &aOutputImplementer);

    otError Process(Arg aArgs[]);

private:

    using Command = CommandEntry<CliOffload>;
    using Components = otOperationalDatasetComponents;

    struct ComponentMapper
    {
        int Compare(const char *aName) const { return strcmp(aName, mName); }

        constexpr static bool AreInOrder(const ComponentMapper &aFirst, const ComponentMapper &aSecond)
        {
            return AreStringsInOrder(aFirst.mName, aSecond.mName);
        }

        const char *mName;
        bool Components::*mIsPresentPtr;
        void (CliOffload::*mOutput)(const otOperationalDataset &aDataset);
    };

    otOperationalDataset     sDataset;

    template <CommandId kCommandId> otError Process(Arg aArgs[]);

    static void HandleActiveScanResult(otActiveScanResult *aResult, void *aContext);
    void HandleActiveScanResult(otActiveScanResult *aResult);

    void OutputResult(otError aError);

    const ComponentMapper *LookupMapper(const char *aName) const;
    otError PrintAll(otOperationalDataset &aDataset);
    void Print(otOperationalDataset &aDataset);

    void OutputActiveTimestamp(const otOperationalDataset &aDataset);
    void OutputChannel(const otOperationalDataset &aDataset);
    void OutputChannelMask(const otOperationalDataset &aDataset);
    void OutputDelay(const otOperationalDataset &aDataset);
    void OutputExtendedPanId(const otOperationalDataset &aDataset);
    void OutputMeshLocalPrefix(const otOperationalDataset &aDataset);
    void OutputNetworkKey(const otOperationalDataset &aDataset);
    void OutputNetworkName(const otOperationalDataset &aDataset);
    void OutputPanId(const otOperationalDataset &aDataset);
    void OutputPendingTimestamp(const otOperationalDataset &aDataset);
    void OutputPskc(const otOperationalDataset &aDataset);
    void OutputSecurityPolicy(const otOperationalDataset &aDataset);

    void    OutputSecurityPolicy(const otSecurityPolicy &aSecurityPolicy);
};

} // namespace Cli
} // namespace ot

#endif // CLI_OFFLOAD_HPP_
