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

#include "openthread-core-config.h"
#include "cli_config.h"

#include "cli_offload.hpp"

#include <openthread/offload.h>
#include <openthread/thread.h>
#include <openthread/platform/offload.h>

#include "cli/cli.hpp"
#include "common/binary_search.hpp"
#include "common/code_utils.hpp"

namespace ot {
namespace Cli {

CliOffload::CliOffload(otInstance *aInstance, OutputImplementer &aOutputImplementer)
    : Output(aInstance, aOutputImplementer)
{
    memset(&sDataset, 0, sizeof(sDataset));
}

template <> otError CliOffload::Process<Cmd("scan")>(Arg aArgs[])
{

    otError error = OT_ERROR_PENDING;
    OT_UNUSED_VARIABLE(aArgs);

    static const char *const kScanTableTitles[]       = {"PAN", "MAC Address", "Ch", "dBm", "LQI"};
    static const uint8_t     kScanTableColumnWidths[] = {6, 18, 4, 5, 5};

    OutputTableHeader(kScanTableTitles, kScanTableColumnWidths);

    otOffloadActiveScan(GetInstancePtr(), 0, 0, &CliOffload::HandleActiveScanResult, this);

    return error;
}

template <> otError CliOffload::Process<Cmd("state")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    OT_UNUSED_VARIABLE(aArgs);

    otLogCritPlat("cli command dataset active");

    OutputLine("%s", otThreadDeviceRoleToString(otOffloadThreadGetDeviceRole(GetInstancePtr())));

    return error;
}

template <> otError CliOffload::Process<Cmd("thread")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0] == "start")
    {
        error = otOffloadThreadStartStop(GetInstancePtr(), true);
    }
    else if (aArgs[0] == "stop")
    {
        error = otOffloadThreadStartStop(GetInstancePtr(), false);
    }
    else if (aArgs[0] == "version")
    {
        OutputLine("%u", otThreadGetVersion());
    }
    else
    {
        error = OT_ERROR_INVALID_COMMAND;
    }

    return error;
}

template <> otError CliOffload::Process<Cmd("dataset")>(Arg aArgs[])
{
    otError error = OT_ERROR_INVALID_ARGS;

    otOperationalDatasetTlvs datasetTlvs;

    if (aArgs[0] == "init")
    {
        if (aArgs[1] == "new")
        {
            error = otOffloadDatasetInitNew(GetInstancePtr(), &sDataset);
            PrintAll(sDataset);
        }
        else
        {
            ExitNow();
        }
    }
    else if (aArgs[0] == "active")
    {
        SuccessOrExit(error = otPlatCpDatasetGetActiveTlvs(GetInstancePtr(), &datasetTlvs));
        SuccessOrExit(error = otDatasetParseTlvs(&datasetTlvs, &sDataset));
        Print(sDataset);
    }
    else if (aArgs[0] == "pending")
    {
        SuccessOrExit(error = otPlatCpDatasetGetPending(GetInstancePtr(), &sDataset));
        Print(sDataset);
    }
    else if (aArgs[0] == "commit")
    {
        if (aArgs[1] == "active")
        {
            error = otPlatCpDatasetSetActive(&sDataset);
        }
        else if (aArgs[1] == "pending")
        {
            error = otPlatCpDatasetSetPending(&sDataset);
        }
    }

exit:
    return error;
}

template <> otError CliOffload::Process<Cmd("factoryreset")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aArgs[0].IsEmpty(), error = OT_ERROR_INVALID_ARGS);

    error = otPlatCpFactoryReset(GetInstancePtr());

exit:
    return error;
}

template <> otError CliOffload::Process<Cmd("ncp")>(Arg aArgs[])
{
    otError error = OT_ERROR_INVALID_ARGS;

    if (aArgs[0] == "version")
    {
        OutputLine("%s", otPlatCpGetVersionString(GetInstancePtr()));
        error = OT_ERROR_NONE;
    }

    return error;
}

template <> otError CliOffload::Process<Cmd("ifconfig")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;

    if (aArgs[0].IsEmpty())
    {
    }
    else if (aArgs[0] == "up")
    {
        SuccessOrExit(error = otPlatIp6SetEnabled(true));
    }
    else if (aArgs[0] == "down")
    {
        SuccessOrExit(error = otPlatIp6SetEnabled(false));
    }
    else
    {
        ExitNow(error = OT_ERROR_INVALID_ARGS);
    }

exit:
    return error;
}

template <> otError CliOffload::Process<Cmd("routerselectionjitter")>(Arg aArgs[])
{
    return Interpreter::GetInterpreter().ProcessGetSet(aArgs, otPlatCpThreadGetRouterSelectionJitter, otPlatCpThreadSetRouterSelectionJitter);
}

template <> otError CliOffload::Process<Cmd("networkkey")>(Arg aArgs[])
{
    otError error = OT_ERROR_NONE;
    otNetworkKey key;
    (void)aArgs;

    SuccessOrExit(error = otPlatCpThreadGetNetworkKey(&key));
    OutputBytesLine(key.m8);

exit:
    return error;
}

otError CliOffload::Process(Arg aArgs[])
{
#define CmdEntry(aCommandString)                                  \
    {                                                             \
        aCommandString, &CliOffload::Process<Cmd(aCommandString)> \
    }

    static constexpr Command kCommands[] = {
        CmdEntry("dataset"), CmdEntry("factoryreset"), CmdEntry("ifconfig"), CmdEntry("ncp"),
        CmdEntry("networkkey"), CmdEntry("routerselectionjitter"), CmdEntry("scan"), CmdEntry("state"),
        CmdEntry("thread"),
    };

    static_assert(BinarySearch::IsSorted(kCommands), "kCommands is not sorted");

    otError        error = OT_ERROR_INVALID_COMMAND;
    const Command *command;

    if (aArgs[0].IsEmpty() || (aArgs[0] == "help"))
    {
        OutputCommandTable(kCommands);
        ExitNow(error = aArgs[0].IsEmpty() ? error : OT_ERROR_NONE);
    }

    command = BinarySearch::Find(aArgs[0].GetCString(), kCommands);
    VerifyOrExit(command != nullptr);

    error = (this->*command->mHandler)(aArgs + 1);

exit:
    return error;
}

void CliOffload::HandleActiveScanResult(otActiveScanResult * aResult)
{
    if (aResult == nullptr)
    {
        OutputResult(OT_ERROR_NONE);
        ExitNow();
    }

    if (aResult->mDiscover)
    {
        OutputFormat("| %-16s ", aResult->mNetworkName.m8);

        OutputFormat("| ");
        OutputBytes(aResult->mExtendedPanId.m8);
        OutputFormat(" ");
    }

    OutputFormat("| %04x | ", aResult->mPanId);
    OutputExtAddress(aResult->mExtAddress);
    OutputFormat(" | %2u ", aResult->mChannel);
    OutputFormat("| %3d ", aResult->mRssi);
    OutputLine("| %3u |", aResult->mLqi);

exit:
    return;
}

void CliOffload::HandleActiveScanResult(otActiveScanResult * aResult, void * aContext)
{
    static_cast<CliOffload *>(aContext)->HandleActiveScanResult(aResult);
}

void CliOffload::OutputResult(otError aError) { Interpreter::GetInterpreter().OutputResult(aError); }

const CliOffload::ComponentMapper *CliOffload::LookupMapper(const char *aName) const
{
    static constexpr ComponentMapper kMappers[] = {
        {
            "activetimestamp",
            &Components::mIsActiveTimestampPresent,
            &CliOffload::OutputActiveTimestamp,
        },
        {
            "channel",
            &Components::mIsChannelPresent,
            &CliOffload::OutputChannel,
        },
        {
            "channelmask",
            &Components::mIsChannelMaskPresent,
            &CliOffload::OutputChannelMask,
        },
        {
            "delay",
            &Components::mIsDelayPresent,
            &CliOffload::OutputDelay,
        },
        {
            "delaytimer", // Alias for "delay "to ensure backward compatibility for "mgmtsetcommand" command
            &Components::mIsDelayPresent,
            &CliOffload::OutputDelay,
        },
        {
            "extpanid",
            &Components::mIsExtendedPanIdPresent,
            &CliOffload::OutputExtendedPanId,
        },
        {
            "localprefix", // Alias for "meshlocalprefix" to ensure backward compatibility in "mgmtsetcommand" command
            &Components::mIsMeshLocalPrefixPresent,
            &CliOffload::OutputMeshLocalPrefix,
        },
        {
            "meshlocalprefix",
            &Components::mIsMeshLocalPrefixPresent,
            &CliOffload::OutputMeshLocalPrefix,
        },
        {
            "networkkey",
            &Components::mIsNetworkKeyPresent,
            &CliOffload::OutputNetworkKey,
        },
        {
            "networkname",
            &Components::mIsNetworkNamePresent,
            &CliOffload::OutputNetworkName,
        },
        {
            "panid",
            &Components::mIsPanIdPresent,
            &CliOffload::OutputPanId,
        },
        {
            "pendingtimestamp",
            &Components::mIsPendingTimestampPresent,
            &CliOffload::OutputPendingTimestamp,
        },
        {
            "pskc",
            &Components::mIsPskcPresent,
            &CliOffload::OutputPskc,
        },
        {
            "securitypolicy",
            &Components::mIsSecurityPolicyPresent,
            &CliOffload::OutputSecurityPolicy,
        },
    };

    static_assert(BinarySearch::IsSorted(kMappers), "kMappers is not sorted");

    return BinarySearch::Find(aName, kMappers);
}

otError CliOffload::PrintAll(otOperationalDataset &aDataset)
{ 
    otError              error = OT_ERROR_NONE;

    Print(aDataset);

    otOperationalDatasetTlvs datasetTlvs;
    error = otDatasetConvertToTlvs(&aDataset, &datasetTlvs);
    OutputBytesLine(datasetTlvs.mTlvs, datasetTlvs.mLength);

    return error;
}

void CliOffload::Print(otOperationalDataset &aDataset)
{ 
    struct ComponentTitle
    {
        const char *mTitle; // Title to output.
        const char *mName;  // To use with `LookupMapper()`.
    };

    static const ComponentTitle kTitles[] = {
        {"Pending Timestamp", "pendingtimestamp"},
        {"Active Timestamp", "activetimestamp"},
        {"Channel", "channel"},
        {"Channel Mask", "channelmask"},
        {"Delay", "delay"},
        {"Ext PAN ID", "extpanid"},
        {"Mesh Local Prefix", "meshlocalprefix"},
        {"Network Key", "networkkey"},
        {"Network Name", "networkname"},
        {"PAN ID", "panid"},
        {"PSKc", "pskc"},
        {"Security Policy", "securitypolicy"},
    };


    for (const ComponentTitle &title : kTitles)
    {
        const ComponentMapper *mapper = LookupMapper(title.mName);
        otLogInfoPlat("%s: %d", title.mTitle, aDataset.mComponents.*mapper->mIsPresentPtr);
        if (aDataset.mComponents.*mapper->mIsPresentPtr)
        {
            OutputFormat("%s: ", title.mTitle);
            (this->*mapper->mOutput)(aDataset);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------

/**
 * @cli dataset activetimestamp (get, set)
 * @code
 * dataset activetimestamp
 * 123456789
 * Done
 * @endcode
 * @code
 * dataset activetimestamp 123456789
 * Done
 * @endcode
 * @cparam dataset activetimestamp [@ca{timestamp}]
 * Pass the optional `timestamp` argument to set the active timestamp.
 * @par
 * Gets or sets #otOperationalCliOffload::mActiveTimestamp.
 */
void CliOffload::OutputActiveTimestamp(const otOperationalDataset &aDataset)
{
    OutputUint64Line(aDataset.mActiveTimestamp.mSeconds);
}

/**
 * @cli dataset channel (get,set)
 * @code
 * dataset channel
 * 12
 * Done
 * @endcode
 * @code
 * dataset channel 12
 * Done
 * @endcode
 * @cparam dataset channel [@ca{channel-num}]
 * Use the optional `channel-num` argument to set the channel.
 * @par
 * Gets or sets #otOperationalCliOffload::mChannel.
 */
void CliOffload::OutputChannel(const otOperationalDataset &aDataset) { OutputLine("%u", aDataset.mChannel); }

/**
 * @cli dataset channelmask (get,set)
 * @code
 * dataset channelmask
 * 0x07fff800
 * Done
 * @endcode
 * @code
 * dataset channelmask 0x07fff800
 * Done
 * @endcode
 * @cparam dataset channelmask [@ca{channel-mask}]
 * Use the optional `channel-mask` argument to set the channel mask.
 * @par
 * Gets or sets #otOperationalCliOffload::mChannelMask
 */
void CliOffload::OutputChannelMask(const otOperationalDataset &aDataset)
{
    OutputLine("0x%08lx", ToUlong(aDataset.mChannelMask));
}

/**
 * @cli dataset delay (get,set)
 * @code
 * dataset delay
 * 1000
 * Done
 * @endcode
 * @code
 * dataset delay 1000
 * Done
 * @endcode
 * @cparam dataset delay [@ca{delay}]
 * Use the optional `delay` argument to set the delay timer value.
 * @par
 * Gets or sets #otOperationalCliOffload::mDelay.
 * @sa otDatasetSetDelayTimerMinimal
 */
void CliOffload::OutputDelay(const otOperationalDataset &aDataset) { OutputLine("%lu", ToUlong(aDataset.mDelay)); }

/**
 * @cli dataset extpanid (get,set)
 * @code
 * dataset extpanid
 * 000db80123456789
 * Done
 * @endcode
 * @code
 * dataset extpanid 000db80123456789
 * Done
 * @endcode
 * @cparam dataset extpanid [@ca{extpanid}]
 * Use the optional `extpanid` argument to set the Extended Personal Area Network ID.
 * @par
 * Gets or sets #otOperationalCliOffload::mExtendedPanId.
 * @note The commissioning credential in the dataset buffer becomes stale after changing
 * this value. Use `dataset pskc` to reset.
 * @csa{dataset pskc (get,set)}
 */
void CliOffload::OutputExtendedPanId(const otOperationalDataset &aDataset) { OutputBytesLine(aDataset.mExtendedPanId.m8); }

/**
 * @cli dataset meshlocalprefix (get,set)
 * @code
 * dataset meshlocalprefix
 * fd00:db8:0:0::/64
 * Done
 * @endcode
 * @code
 * dataset meshlocalprefix fd00:db8:0:0::
 * Done
 * @endcode
 * @cparam dataset meshlocalprefix [@ca{meshlocalprefix}]
 * Use the optional `meshlocalprefix` argument to set the Mesh-Local Prefix.
 * @par
 * Gets or sets #otOperationalCliOffload::mMeshLocalPrefix.
 */
void CliOffload::OutputMeshLocalPrefix(const otOperationalDataset &aDataset)
{
    OutputIp6PrefixLine(aDataset.mMeshLocalPrefix);
}

/**
 * @cli dataset networkkey (get,set)
 * @code
 * dataset networkkey
 * 00112233445566778899aabbccddeeff
 * Done
 * @endcode
 * @code
 * dataset networkkey 00112233445566778899aabbccddeeff
 * Done
 * @endcode
 * @cparam dataset networkkey [@ca{key}]
 * Use the optional `key` argument to set the Network Key.
 * @par
 * Gets or sets #otOperationalCliOffload::mNetworkKey.
 */
void CliOffload::OutputNetworkKey(const otOperationalDataset &aDataset) { OutputBytesLine(aDataset.mNetworkKey.m8); }

/**
 * @cli dataset networkname (get,set)
 * @code
 * dataset networkname
 * OpenThread
 * Done
 * @endcode
 * @code
 * dataset networkname OpenThread
 * Done
 * @endcode
 * @cparam dataset networkname [@ca{name}]
 * Use the optional `name` argument to set the Network Name.
 * @par
 * Gets or sets #otOperationalCliOffload::mNetworkName.
 * @note The Commissioning Credential in the dataset buffer becomes stale after changing this value.
 * Use `dataset pskc` to reset.
 * @csa{dataset pskc (get,set)}
 */
void CliOffload::OutputNetworkName(const otOperationalDataset &aDataset) { OutputLine("%s", aDataset.mNetworkName.m8); }

/**
 * @cli dataset panid (get,set)
 * @code
 * dataset panid
 * 0x1234
 * Done
 * @endcode
 * @code
 * dataset panid 0x1234
 * Done
 * @endcode
 * @cparam dataset panid [@ca{panid}]
 * Use the optional `panid` argument to set the PAN ID.
 * @par
 * Gets or sets #otOperationalCliOffload::mPanId.
 */
void CliOffload::OutputPanId(const otOperationalDataset &aDataset) { OutputLine("0x%04x", aDataset.mPanId); }

/**
 * @cli dataset pendingtimestamp (get,set)
 * @code
 * dataset pendingtimestamp
 * 123456789
 * Done
 * @endcode
 * @code
 * dataset pendingtimestamp 123456789
 * Done
 * @endcode
 * @cparam dataset pendingtimestamp [@ca{timestamp}]
 * Use the optional `timestamp` argument to set the pending timestamp seconds.
 * @par
 * Gets or sets #otOperationalCliOffload::mPendingTimestamp.
 */
void CliOffload::OutputPendingTimestamp(const otOperationalDataset &aDataset)
{
    OutputUint64Line(aDataset.mPendingTimestamp.mSeconds);
}

/**
 * @cli dataset pskc (get,set)
 * @code
 * dataset pskc
 * 67c0c203aa0b042bfb5381c47aef4d9e
 * Done
 * @endcode
 * @code
 * dataset pskc -p 123456
 * Done
 * @endcode
 * @code
 * dataset pskc 67c0c203aa0b042bfb5381c47aef4d9e
 * Done
 * @endcode
 * @cparam dataset pskc [@ca{-p} @ca{passphrase}] | [@ca{key}]
 * For FTD only, use `-p` with the `passphrase` argument. `-p` generates a pskc from
 * the UTF-8 encoded `passphrase` that you provide, together with
 * the network name and extended PAN ID. If set, `-p` uses the dataset buffer;
 * otherwise, it uses the current stack.
 * Alternatively, you can set pskc as `key` (hex format).
 * @par
 * Gets or sets #otOperationalCliOffload::mPskc.
 */
void CliOffload::OutputPskc(const otOperationalDataset &aDataset) { OutputBytesLine(aDataset.mPskc.m8); }

/**
 * @cli dataset securitypolicy (get,set)
 * @code
 * dataset securitypolicy
 * 672 onrc
 * Done
 * @endcode
 * @code
 * dataset securitypolicy 672 onrc
 * Done
 * @endcode
 * @cparam dataset securitypolicy [@ca{rotationtime} [@ca{onrcCepR}]]
 * *   Use `rotationtime` for `thrKeyRotation`, in units of hours.
 * *   Security Policy commands use the `onrcCepR` argument mappings to get and set
 * #otSecurityPolicy members, for example `o` represents
 * #otSecurityPolicy::mObtainNetworkKeyEnabled.
 * @moreinfo{@dataset}.
 * @par
 * Gets or sets the %Dataset security policy.
 */
void CliOffload::OutputSecurityPolicy(const otOperationalDataset &aDataset)
{
    OutputSecurityPolicy(aDataset.mSecurityPolicy);
}

void CliOffload::OutputSecurityPolicy(const otSecurityPolicy &aSecurityPolicy)
{
    OutputFormat("%u ", aSecurityPolicy.mRotationTime);

    if (aSecurityPolicy.mObtainNetworkKeyEnabled)
    {
        OutputFormat("o");
    }

    if (aSecurityPolicy.mNativeCommissioningEnabled)
    {
        OutputFormat("n");
    }

    if (aSecurityPolicy.mRoutersEnabled)
    {
        OutputFormat("r");
    }

    if (aSecurityPolicy.mExternalCommissioningEnabled)
    {
        OutputFormat("c");
    }

    if (aSecurityPolicy.mCommercialCommissioningEnabled)
    {
        OutputFormat("C");
    }

    if (aSecurityPolicy.mAutonomousEnrollmentEnabled)
    {
        OutputFormat("e");
    }

    if (aSecurityPolicy.mNetworkKeyProvisioningEnabled)
    {
        OutputFormat("p");
    }

    if (aSecurityPolicy.mNonCcmRoutersEnabled)
    {
        OutputFormat("R");
    }

    OutputLine(" %u", aSecurityPolicy.mVersionThresholdForRouting);
}

} // namespace Cli
} // namespace ot
