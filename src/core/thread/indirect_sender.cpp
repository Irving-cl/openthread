/*
 *  Copyright (c) 2019, The OpenThread Authors.
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
 *   This file includes definitions for handling indirect transmission.
 */

#include "indirect_sender.hpp"

#include "common/code_utils.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/message.hpp"
#include "thread/mesh_forwarder.hpp"
#include "thread/mle_tlvs.hpp"
#include "thread/topology.hpp"

namespace ot {

const Mac::Address &IndirectSender::IndirectTxInfo::GetMacAddress(Mac::Address &aMacAddress) const
{
    if (mUseShortAddress)
    {
        aMacAddress.SetShort(static_cast<const SedCapableNeighbor *>(this)->GetRloc16());
    }
    else
    {
        aMacAddress.SetExtended(static_cast<const SedCapableNeighbor *>(this)->GetExtAddress());
    }

    return aMacAddress;
}

IndirectSender::IndirectSender(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mEnabled(false)
    , mSourceMatchController(aInstance)
    , mDataPollHandler(aInstance)
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    , mCslTxScheduler(aInstance)
#endif
{
}

void IndirectSender::Stop(void)
{
    VerifyOrExit(mEnabled);

    for (SedCapableNeighbor &sed : Get<SedCapableNeighborTable>().Iterate(Neighbor::kInStateAnyExceptInvalid))
    {
        sed.SetIndirectMessage(nullptr);
        mSourceMatchController.ResetMessageCount(sed);
    }

    mDataPollHandler.Clear();
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    mCslTxScheduler.Clear();
#endif

exit:
    mEnabled = false;
}

void IndirectSender::AddMessageForSedCapableNeighbor(Message &aMessage, SedCapableNeighbor &aSedCapableNeighbor)
{
    uint16_t sedIndex;

    OT_ASSERT(!aSedCapableNeighbor.IsRxOnWhenIdle());

    sedIndex = Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor);
    VerifyOrExit(!aMessage.GetSedNeighborMask(sedIndex));

    aMessage.SetSedNeighborMask(sedIndex);
    mSourceMatchController.IncrementMessageCount(aSedCapableNeighbor);

    if ((aMessage.GetType() != Message::kTypeSupervision) && (aSedCapableNeighbor.GetIndirectMessageCount() > 1))
    {
        Message *supervisionMessage = FindIndirectMessage(aSedCapableNeighbor, /* aSupervisionTypeOnly */ true);

        if (supervisionMessage != nullptr)
        {
            IgnoreError(RemoveMessageFromSedCapableNeighbor(*supervisionMessage, aSedCapableNeighbor));
        }
    }

    RequestMessageUpdate(aSedCapableNeighbor);

exit:
    return;
}

otError IndirectSender::RemoveMessageFromSedCapableNeighbor(Message &aMessage, SedCapableNeighbor &aSedCapableNeighbor)
{
    otError  error    = OT_ERROR_NONE;
    uint16_t sedIndex = Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor);

    VerifyOrExit(aMessage.GetSedNeighborMask(sedIndex), error = OT_ERROR_NOT_FOUND);

    aMessage.ClearSedNeighborMask(sedIndex);
    mSourceMatchController.DecrementMessageCount(aSedCapableNeighbor);

    RequestMessageUpdate(aSedCapableNeighbor);

exit:
    return error;
}

void IndirectSender::ClearAllMessagesForSedCapableNeighbor(SedCapableNeighbor &aSedCapableNeighbor)
{
    Message *message;
    Message *nextMessage;

    VerifyOrExit(aSedCapableNeighbor.GetIndirectMessageCount() > 0);

    for (message = Get<MeshForwarder>().mSendQueue.GetHead(); message; message = nextMessage)
    {
        nextMessage = message->GetNext();

        message->ClearSedNeighborMask(Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor));

        if (!message->IsSedNeighborPending() && !message->GetDirectTransmission())
        {
            if (Get<MeshForwarder>().mSendMessage == message)
            {
                Get<MeshForwarder>().mSendMessage = nullptr;
            }

            Get<MeshForwarder>().mSendQueue.Dequeue(*message);
            message->Free();
        }
    }

    aSedCapableNeighbor.SetIndirectMessage(nullptr);
    mSourceMatchController.ResetMessageCount(aSedCapableNeighbor);

    mDataPollHandler.RequestFrameChange(DataPollHandler::kPurgeFrame, aSedCapableNeighbor);
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    mCslTxScheduler.Update();
#endif

exit:
    return;
}

void IndirectSender::SetSedCapableNeighborUseShortAddress(SedCapableNeighbor &aSedCapableNeighbor,
                                                          bool                aUseShortAddress)
{
    VerifyOrExit(aSedCapableNeighbor.IsIndirectSourceMatchShort() != aUseShortAddress);

    mSourceMatchController.SetSrcMatchAsShort(aSedCapableNeighbor, aUseShortAddress);

exit:
    return;
}

void IndirectSender::HandleSedCapableNeighborModeChange(SedCapableNeighbor &aSedCapableNeighbor,
                                                        Mle::DeviceMode     aOldMode)
{
    if (!aSedCapableNeighbor.IsRxOnWhenIdle() && (aSedCapableNeighbor.IsStateValid()))
    {
        SetSedCapableNeighborUseShortAddress(aSedCapableNeighbor, true);
    }

    // On sleepy to non-sleepy mode change, convert indirect messages in
    // the send queue destined to the child to direct.

    if (!aOldMode.IsRxOnWhenIdle() && aSedCapableNeighbor.IsRxOnWhenIdle() &&
        (aSedCapableNeighbor.GetIndirectMessageCount() > 0))
    {
        uint16_t sedIndex = Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor);

        for (Message *message = Get<MeshForwarder>().mSendQueue.GetHead(); message; message = message->GetNext())
        {
            if (message->GetSedNeighborMask(sedIndex))
            {
                message->ClearSedNeighborMask(sedIndex);
                message->SetDirectTransmission();
            }
        }

        aSedCapableNeighbor.SetIndirectMessage(nullptr);
        mSourceMatchController.ResetMessageCount(aSedCapableNeighbor);

        mDataPollHandler.RequestFrameChange(DataPollHandler::kPurgeFrame, aSedCapableNeighbor);
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
        mCslTxScheduler.Update();
#endif
    }

    // Since the queuing delays for direct transmissions are expected to
    // be relatively small especially when compared to indirect, for a
    // non-sleepy to sleepy mode change, we allow any direct message
    // (for the child) already in the send queue to remain as is. This
    // is equivalent to dropping the already queued messages in this
    // case.
}

Message *IndirectSender::FindIndirectMessage(SedCapableNeighbor &aSedCapableNeighbor, bool aSupervisionTypeOnly)
{
    Message *message;
    uint16_t sedIndex = Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor);

    for (message = Get<MeshForwarder>().mSendQueue.GetHead(); message; message = message->GetNext())
    {
        if (message->GetSedNeighborMask(sedIndex) &&
            (!aSupervisionTypeOnly || (message->GetType() == Message::kTypeSupervision)))
        {
            break;
        }
    }

    return message;
}

void IndirectSender::RequestMessageUpdate(SedCapableNeighbor &aSedCapableNeighbor)
{
    Message *curMessage = aSedCapableNeighbor.GetIndirectMessage();
    Message *newMessage;

    // Purge the frame if the current message is no longer destined
    // for the child. This check needs to be done first to cover the
    // case where we have a pending "replace frame" request and while
    // waiting for the callback, the current message is removed.

    if ((curMessage != nullptr) &&
        !curMessage->GetSedNeighborMask(Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor)))
    {
        // Set the indirect message for this child to nullptr to ensure
        // it is not processed on `HandleSentFrameToSedCapableNeighbor()` callback.

        aSedCapableNeighbor.SetIndirectMessage(nullptr);

        // Request a "frame purge" using `RequestFrameChange()` and
        // wait for `HandleFrameChangeDone()` callback for completion
        // of the request. Note that the callback may be directly
        // called from the `RequestFrameChange()` itself when the
        // request can be handled immediately.

        aSedCapableNeighbor.SetWaitingForMessageUpdate(true);
        mDataPollHandler.RequestFrameChange(DataPollHandler::kPurgeFrame, aSedCapableNeighbor);
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
        mCslTxScheduler.Update();
#endif

        ExitNow();
    }

    VerifyOrExit(!aSedCapableNeighbor.IsWaitingForMessageUpdate());

    newMessage = FindIndirectMessage(aSedCapableNeighbor);

    VerifyOrExit(curMessage != newMessage);

    if (curMessage == nullptr)
    {
        // Current message is nullptr, but new message is not.
        // We have a new indirect message.

        UpdateIndirectMessage(aSedCapableNeighbor);
        ExitNow();
    }

    // Current message and new message differ and are both non-nullptr.
    // We need to request the frame to be replaced. The current
    // indirect message can be replaced only if it is the first
    // fragment. If a next fragment frame for message is already
    // prepared, we wait for the entire message to be delivered.

    VerifyOrExit(aSedCapableNeighbor.GetIndirectFragmentOffset() == 0);

    aSedCapableNeighbor.SetWaitingForMessageUpdate(true);
    mDataPollHandler.RequestFrameChange(DataPollHandler::kReplaceFrame, aSedCapableNeighbor);
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    mCslTxScheduler.Update();
#endif

exit:
    return;
}

void IndirectSender::HandleFrameChangeDone(SedCapableNeighbor &aSedCapableNeighbor)
{
    VerifyOrExit(aSedCapableNeighbor.IsWaitingForMessageUpdate());
    UpdateIndirectMessage(aSedCapableNeighbor);

exit:
    return;
}

void IndirectSender::UpdateIndirectMessage(SedCapableNeighbor &aSedCapableNeighbor)
{
    Message *message = FindIndirectMessage(aSedCapableNeighbor);

    aSedCapableNeighbor.SetWaitingForMessageUpdate(false);
    aSedCapableNeighbor.SetIndirectMessage(message);
    aSedCapableNeighbor.SetIndirectFragmentOffset(0);
    aSedCapableNeighbor.SetIndirectTxSuccess(true);

#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
    mCslTxScheduler.Update();
#endif

    if (message != nullptr)
    {
        Mac::Address childAddress;

        mDataPollHandler.HandleNewFrame(aSedCapableNeighbor);

        aSedCapableNeighbor.GetMacAddress(childAddress);
        Get<MeshForwarder>().LogMessage(MeshForwarder::kMessagePrepareIndirect, *message, &childAddress, OT_ERROR_NONE);
    }
}

otError IndirectSender::PrepareFrameForSedCapableNeighbor(Mac::TxFrame &      aFrame,
                                                          FrameContext &      aContext,
                                                          SedCapableNeighbor &aSedCapableNeighbor)
{
    otError  error   = OT_ERROR_NONE;
    Message *message = aSedCapableNeighbor.GetIndirectMessage();

    VerifyOrExit(mEnabled, error = OT_ERROR_ABORT);

    if (message == nullptr)
    {
        PrepareEmptyFrame(aFrame, aSedCapableNeighbor, /* aAckRequest */ true);
        ExitNow();
    }

    switch (message->GetType())
    {
    case Message::kTypeIp6:
        aContext.mMessageNextOffset = PrepareDataFrame(aFrame, aSedCapableNeighbor, *message);
        break;

    case Message::kTypeSupervision:
        PrepareEmptyFrame(aFrame, aSedCapableNeighbor, kSupervisionMsgAckRequest);
        aContext.mMessageNextOffset = message->GetLength();
        break;

    default:
        OT_ASSERT(false);
        OT_UNREACHABLE_CODE(break);
    }

exit:
    return error;
}

uint16_t IndirectSender::PrepareDataFrame(Mac::TxFrame &      aFrame,
                                          SedCapableNeighbor &aSedCapableNeighbor,
                                          Message &           aMessage)
{
    Ip6::Header  ip6Header;
    Mac::Address macSource, macDest;
    uint16_t     directTxOffset;
    uint16_t     nextOffset;

    // Determine the MAC source and destination addresses.

    IgnoreError(aMessage.Read(0, ip6Header));

    Get<MeshForwarder>().GetMacSourceAddress(ip6Header.GetSource(), macSource);

    if (ip6Header.GetDestination().IsLinkLocal())
    {
        Get<MeshForwarder>().GetMacDestinationAddress(ip6Header.GetDestination(), macDest);
    }
    else
    {
        aSedCapableNeighbor.GetMacAddress(macDest);
    }

    // Prepare the data frame from previous child's indirect offset.

    directTxOffset = aMessage.GetOffset();
    aMessage.SetOffset(aSedCapableNeighbor.GetIndirectFragmentOffset());

    nextOffset = Get<MeshForwarder>().PrepareDataFrame(aFrame, aMessage, macSource, macDest);

    aMessage.SetOffset(directTxOffset);

    // Set `FramePending` if there are more queued messages (excluding
    // the current one being sent out) for the child (note `> 1` check).
    // The case where the current message itself requires fragmentation
    // is already checked and handled in `PrepareDataFrame()` method.

    if (aSedCapableNeighbor.GetIndirectMessageCount() > 1)
    {
        aFrame.SetFramePending(true);
    }

    return nextOffset;
}

void IndirectSender::PrepareEmptyFrame(Mac::TxFrame &aFrame, SedCapableNeighbor &aSedCapableNeighbor, bool aAckRequest)
{
    Mac::Address macDest;
    aSedCapableNeighbor.GetMacAddress(macDest);
    Get<MeshForwarder>().PrepareEmptyFrame(aFrame, macDest, aAckRequest);
}

void IndirectSender::HandleSentFrameToSedCapableNeighbor(const Mac::TxFrame &aFrame,
                                                         const FrameContext &aContext,
                                                         otError             aError,
                                                         SedCapableNeighbor &aSedCapableNeighbor)
{
    Message *message    = aSedCapableNeighbor.GetIndirectMessage();
    uint16_t nextOffset = aContext.mMessageNextOffset;

    VerifyOrExit(mEnabled);

    switch (aError)
    {
#if OPENTHREAD_FTD
    case OT_ERROR_NONE:
        Get<Utils::ChildSupervisor>().UpdateOnSend(
            *Get<ChildTable>().MapSedCapableNeighborToChild(aSedCapableNeighbor));
        break;
#endif
    case OT_ERROR_NO_ACK:
    case OT_ERROR_CHANNEL_ACCESS_FAILURE:
    case OT_ERROR_ABORT:

        aSedCapableNeighbor.SetIndirectTxSuccess(false);

#if OPENTHREAD_CONFIG_DROP_MESSAGE_ON_FRAGMENT_TX_FAILURE
        // We set the nextOffset to end of message, since there is no need to
        // send any remaining fragments in the message to the child, if all tx
        // attempts of current frame already failed.

        if (message != nullptr)
        {
            nextOffset = message->GetLength();
        }
#endif
        break;

    default:
        OT_ASSERT(false);
        OT_UNREACHABLE_CODE(break);
    }

    if ((message != nullptr) && (nextOffset < message->GetLength()))
    {
        aSedCapableNeighbor.SetIndirectFragmentOffset(nextOffset);
        mDataPollHandler.HandleNewFrame(aSedCapableNeighbor);
#if OPENTHREAD_CONFIG_MAC_CSL_TRANSMITTER_ENABLE
        mCslTxScheduler.Update();
#endif
        ExitNow();
    }

    if (message != nullptr)
    {
        // The indirect tx of this message to the child is done.

        otError      txError  = aError;
        uint16_t     sedIndex = Get<SedCapableNeighborTable>().GetSedCapableNeighborIndex(aSedCapableNeighbor);
        Mac::Address macDest;

        aSedCapableNeighbor.SetIndirectMessage(nullptr);
        aSedCapableNeighbor.GetLinkInfo().AddMessageTxStatus(aSedCapableNeighbor.GetIndirectTxSuccess());

        // Enable short source address matching after the first indirect
        // message transmission attempt to the child. We intentionally do
        // not check for successful tx here to address the scenario where
        // the child does receive "SedCapableNeighbor ID Response" but parent misses the
        // 15.4 ack from child. If the "SedCapableNeighbor ID Response" does not make it
        // to the child, then the child will need to send a new "SedCapableNeighbor ID
        // Request" which will cause the parent to switch to using long
        // address mode for source address matching.

        mSourceMatchController.SetSrcMatchAsShort(aSedCapableNeighbor, true);

#if !OPENTHREAD_CONFIG_DROP_MESSAGE_ON_FRAGMENT_TX_FAILURE

        // When `CONFIG_DROP_MESSAGE_ON_FRAGMENT_TX_FAILURE` is
        // disabled, all fragment frames of a larger message are
        // sent even if the transmission of an earlier fragment fail.
        // Note that `GetIndirectTxSuccess() tracks the tx success of
        // the entire message to the child, while `txError = aError`
        // represents the error status of the last fragment frame
        // transmission.

        if (!aSedCapableNeighbor.GetIndirectTxSuccess() && (txError == OT_ERROR_NONE))
        {
            txError = OT_ERROR_FAILED;
        }
#endif

        if (!aFrame.IsEmpty())
        {
            IgnoreError(aFrame.GetDstAddr(macDest));
            Get<MeshForwarder>().LogMessage(MeshForwarder::kMessageTransmit, *message, &macDest, txError);
        }

        if (message->GetType() == Message::kTypeIp6)
        {
            if (aSedCapableNeighbor.GetIndirectTxSuccess())
            {
                Get<MeshForwarder>().mIpCounters.mTxSuccess++;
            }
            else
            {
                Get<MeshForwarder>().mIpCounters.mTxFailure++;
            }
        }

        if (message->GetSedNeighborMask(sedIndex))
        {
            message->ClearSedNeighborMask(sedIndex);
            mSourceMatchController.DecrementMessageCount(aSedCapableNeighbor);
        }

        if (!message->GetDirectTransmission() && !message->IsSedNeighborPending())
        {
            Get<MeshForwarder>().mSendQueue.Dequeue(*message);
            message->Free();
        }
    }

    UpdateIndirectMessage(aSedCapableNeighbor);

exit:
    if (mEnabled)
    {
        ClearMessagesForRemovedSedCapableNeighbors();
    }
}

void IndirectSender::ClearMessagesForRemovedSedCapableNeighbors(void)
{
    for (SedCapableNeighbor &sed :
         Get<SedCapableNeighborTable>().Iterate(SedCapableNeighbor::kInStateAnyExceptValidOrRestoring))
    {
        if (sed.GetIndirectMessageCount() == 0)
        {
            continue;
        }

        ClearAllMessagesForSedCapableNeighbor(sed);
    }
}

} // namespace ot
