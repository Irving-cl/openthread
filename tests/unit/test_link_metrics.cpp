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

#include "test_platform.h"
#include "test_util.h"

#include "common/message.hpp"
#include "net/ip6_address.hpp"
#include "thread/link_metrics.hpp"
#include "thread/link_metrics_tlvs.hpp"
#include "thread/topology.hpp"

namespace ot {

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE && OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE

void TestLinkMetricsScaling(void)
{
    printf("\nTestLinkMetricsScaling\n");

    // Test Link Margin scaling from [0,130] -> [0, 255]

    for (uint8_t linkMargin = 0; linkMargin <= 130; linkMargin++)
    {
        double  scaled     = 255.0 / 130.0 * linkMargin;
        uint8_t scaledAsU8 = static_cast<uint8_t>(scaled + 0.5);

        printf("\nLinkMargin : %-3u -> Scaled : %.1f (rounded:%u)", linkMargin, scaled, scaledAsU8);

        VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleLinkMarginToRawValue(linkMargin) == scaledAsU8);
        VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleRawValueToLinkMargin(scaledAsU8) == linkMargin);
    }

    VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleLinkMarginToRawValue(131) == 255);
    VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleLinkMarginToRawValue(150) == 255);
    VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleLinkMarginToRawValue(255) == 255);

    // Test RSSI scaling from [-130, 0] -> [0, 255]

    for (int8_t rssi = -128; rssi <= 0; rssi++)
    {
        double  scaled     = 255.0 / 130.0 * (rssi + 130.0);
        uint8_t scaledAsU8 = static_cast<uint8_t>(scaled + 0.5);

        printf("\nRSSI : %-3d -> Scaled :%.1f (rounded:%u)", rssi, scaled, scaledAsU8);

        VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleRssiToRawValue(rssi) == scaledAsU8);
        VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleRawValueToRssi(scaledAsU8) == rssi);
    }

    VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleRssiToRawValue(1) == 255);
    VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleRssiToRawValue(10) == 255);
    VerifyOrQuit(LinkMetrics::LinkMetrics::ScaleRssiToRawValue(127) == 255);
}

class TestLinkMetrics
{
private:
    TestLinkMetrics()
        : mResultMessage(nullptr)
    {
        mOtInstance = static_cast<Instance *>(testInitInstance());
        mMsgDest.Clear();
    }
    ~TestLinkMetrics() = default;

    Instance *mOtInstance;

public:
    TestLinkMetrics(const TestLinkMetrics &)            = delete;
    TestLinkMetrics &operator=(const TestLinkMetrics &) = delete;

    static TestLinkMetrics &GetInstance()
    {
        static TestLinkMetrics instance;
        return instance;
    }

    Instance &GetOtInstance() { return *mOtInstance; }

    // Implement underlying APIs for LinkMetricsInitiator and LinkMetricsSubject
    static Message *TestNewMleMessage(uint8_t aCommand, void *aApiContext);
    static Error    TestSendMleMessage(const Ip6::Address &aDestination, Message &aMessage, void *ApiContext);
    static Error    TestSendMleDataRequest(const Ip6::Address &aDestination,
                                           const uint8_t      *aTlvs,
                                           uint8_t             aTlvsLength,
                                           const uint8_t      *aExtraTlvsBuf,
                                           uint8_t             aExtraTlvsBufLength,
                                           void               *aApiContext);
    static Error    TestFindNeighbor(const Ip6::Address &aDestination, Neighbor *&aNeighbor, void *aApiContext);
    static uint8_t  TestComputeLinkMargin(int8_t aRss, void *aApiContext);
    static Error    TestConfigureEnhAckProbing(LinkMetrics::Metrics     aLinkMetrics,
                                               const Mac::ShortAddress &aShortAddress,
                                               const Mac::ExtAddress   &aExtAddress,
                                               void                    *aApiContext);

private:
    Message *TestNewMleMessage(uint8_t aCommand);
    Error    TestSendMleMessage(const Ip6::Address &aDestination, Message &aMessage);
    Error    TestSendMleDataRequest(const Ip6::Address &aDestination,
                                    const uint8_t      *aTlvs,
                                    uint8_t             aTlvsLength,
                                    const uint8_t      *aExtraTlvsBuf,
                                    uint8_t             aExtraTlvsBufLength);
    Error    TestFindNeighbor(const Ip6::Address &aDestination, Neighbor *&aNeighbor);
    uint8_t  TestComputeLinkMargin(int8_t aRss);
    Error    TestConfigureEnhAckProbing(LinkMetrics::Metrics     aLinkMetrics,
                                        const Mac::ShortAddress &aShortAddress,
                                        const Mac::ExtAddress   &aExtAddress);

    Ip6::Address mMsgDest;
    Message     *mResultMessage;
    uint8_t      mMleTlvsBuf[8];
    uint8_t      mMleTlvsBufLength;
    uint8_t      mExtraTlvsBuf[50];
    uint8_t      mExtraTlvsBufLength;

public:
    static constexpr uint8_t kMaxNeighbors = 3;
    Neighbor                 mNeighbors[kMaxNeighbors];

    bool CheckMleDataRequest(uint8_t aSeriesId, LinkMetrics::Metrics &aMetrics);
    void AppendDataRequestTlvsToMessage(Message &aMessage);
};

Message *TestLinkMetrics::TestNewMleMessage(uint8_t aCommand, void *aApiContext)
{
    return static_cast<TestLinkMetrics *>(aApiContext)->TestNewMleMessage(aCommand);
}

Error TestLinkMetrics::TestSendMleMessage(const Ip6::Address &aDestination, Message &aMessage, void *aApiContext)
{
    return static_cast<TestLinkMetrics *>(aApiContext)->TestSendMleMessage(aDestination, aMessage);
}

Error TestLinkMetrics::TestSendMleDataRequest(const Ip6::Address &aDestination,
                                              const uint8_t      *aTlvs,
                                              uint8_t             aTlvsLength,
                                              const uint8_t      *aExtraTlvsBuf,
                                              uint8_t             aExtraTlvsBufLength,
                                              void               *aApiContext)
{
    return static_cast<TestLinkMetrics *>(aApiContext)
        ->TestSendMleDataRequest(aDestination, aTlvs, aTlvsLength, aExtraTlvsBuf, aExtraTlvsBufLength);
}

Error TestLinkMetrics::TestFindNeighbor(const Ip6::Address &aDestination, Neighbor *&aNeighbor, void *aApiContext)
{
    return static_cast<TestLinkMetrics *>(aApiContext)->TestFindNeighbor(aDestination, aNeighbor);
}

uint8_t TestLinkMetrics::TestComputeLinkMargin(int8_t aRss, void *aApiContext)
{
    return static_cast<TestLinkMetrics *>(aApiContext)->TestComputeLinkMargin(aRss);
}

Error TestLinkMetrics::TestConfigureEnhAckProbing(LinkMetrics::Metrics     aLinkMetrics,
                                                  const Mac::ShortAddress &aShortAddress,
                                                  const Mac::ExtAddress   &aExtAddress,
                                                  void                    *aApiContext)
{
    return static_cast<TestLinkMetrics *>(aApiContext)
        ->TestConfigureEnhAckProbing(aLinkMetrics, aShortAddress, aExtAddress);
}

Message *TestLinkMetrics::TestNewMleMessage(uint8_t aCommand)
{
    Message          *message = nullptr;
    Message::SubType  subType = Message::kSubTypeMleGeneral;
    Message::Settings settings(Message::kNoLinkSecurity, Message::kPriorityNet);
    uint8_t           securitySuite = 255; // kNoSecurity

    message = mOtInstance->Get<MessagePool>().Allocate(Message::kTypeIp6, 0, settings);
    VerifyOrExit(message != nullptr);

    message->SetSubType(subType);

    SuccessOrQuit(message->Append(securitySuite));
    SuccessOrQuit(message->Append<uint8_t>(aCommand));

exit:
    return message;
}

Error TestLinkMetrics::TestSendMleMessage(const Ip6::Address &aDestination, Message &aMessage)
{
    mMsgDest       = aDestination;
    mResultMessage = &aMessage;
    return OT_ERROR_NONE;
}

Error TestLinkMetrics::TestSendMleDataRequest(const Ip6::Address &aDestination,
                                              const uint8_t      *aTlvs,
                                              uint8_t             aTlvsLength,
                                              const uint8_t      *aExtraTlvsBuf,
                                              uint8_t             aExtraTlvsBufLength)
{
    mMsgDest = aDestination;

    VerifyOrQuit(aTlvsLength < sizeof(mMleTlvsBuf));
    mMleTlvsBufLength = aTlvsLength;
    memcpy(mMleTlvsBuf, aTlvs, mMleTlvsBufLength);

    VerifyOrQuit(aExtraTlvsBufLength < sizeof(mExtraTlvsBuf));
    mExtraTlvsBufLength = aExtraTlvsBufLength;
    memcpy(mExtraTlvsBuf, aExtraTlvsBuf, mExtraTlvsBufLength);

    return OT_ERROR_NONE;
}

Error TestLinkMetrics::TestFindNeighbor(const Ip6::Address &aDestination, Neighbor *&aNeighbor)
{
    Error        error = kErrorUnknownNeighbor;
    Mac::Address macAddress;

    aNeighbor = nullptr;
    VerifyOrExit(aDestination.IsLinkLocal());
    aDestination.GetIid().ConvertToMacAddress(macAddress);

    for (auto &neighbor : mNeighbors)
    {
        if (neighbor.Matches(Neighbor::AddressMatcher(macAddress, Neighbor::kInStateAny)))
        {
            aNeighbor = &neighbor;
        }
    }
    VerifyOrExit(aNeighbor != nullptr);

    VerifyOrExit(aNeighbor->GetVersion() >= kThreadVersion1p2, error = kErrorNotCapable);
    error = kErrorNone;

exit:
    return error;
}

uint8_t TestLinkMetrics::TestComputeLinkMargin(int8_t aRss)
{
    return mOtInstance->Get<Mac::Mac>().ComputeLinkMargin(aRss);
}

Error TestLinkMetrics::TestConfigureEnhAckProbing(LinkMetrics::Metrics     aLinkMetrics,
                                                  const Mac::ShortAddress &aShortAddress,
                                                  const Mac::ExtAddress   &aExtAddress)
{
    OT_UNUSED_VARIABLE(aLinkMetrics);
    OT_UNUSED_VARIABLE(aShortAddress);
    OT_UNUSED_VARIABLE(aExtAddress);
    return OT_ERROR_NONE;
}

bool TestLinkMetrics::CheckMleDataRequest(uint8_t aSeriesId, LinkMetrics::Metrics &aMetrics)
{
    bool     result = false;
    uint8_t  typeIds[LinkMetrics::kMaxTypeIds];
    uint8_t  typeIdCount         = aMetrics.ConvertToTypeIds(typeIds);
    uint8_t  expectedQueryTlvLen = sizeof(Tlv) + sizeof(uint8_t) + (typeIdCount > 0 ? sizeof(Tlv) : 0) + typeIdCount;
    uint8_t *ptr                 = mExtraTlvsBuf;

    VerifyOrExit(mMleTlvsBufLength == 1);
    VerifyOrExit(mMleTlvsBuf[0] == Mle::Tlv::kLinkMetricsReport);

    VerifyOrExit(mExtraTlvsBufLength == sizeof(Tlv) + expectedQueryTlvLen);
    VerifyOrExit(reinterpret_cast<Tlv *>(ptr)->GetType() == Mle::Tlv::kLinkMetricsQuery);
    VerifyOrExit(reinterpret_cast<Tlv *>(ptr)->GetLength() == expectedQueryTlvLen);

    ptr += sizeof(Tlv);
    VerifyOrExit(reinterpret_cast<Tlv *>(ptr)->GetType() == LinkMetrics::SubTlv::kQueryId);
    VerifyOrExit(reinterpret_cast<Tlv *>(ptr)->GetLength() == sizeof(uint8_t));
    ptr += sizeof(Tlv);
    VerifyOrExit(*ptr == aSeriesId);
    ptr += sizeof(uint8_t);

    if (typeIdCount > 0)
    {
        VerifyOrExit(reinterpret_cast<Tlv *>(ptr)->GetType() == LinkMetrics::SubTlv::kQueryOptions);
        VerifyOrExit(reinterpret_cast<Tlv *>(ptr)->GetLength() == typeIdCount);
        ptr += sizeof(Tlv);

        for (uint8_t i = 0; i < typeIdCount; i++)
        {
            VerifyOrExit(ptr[i] == typeIds[i]);
        }
    }
    result = true;
exit:
    return result;
}

void TestLinkMetrics::AppendDataRequestTlvsToMessage(Message &aMessage)
{
    Tlv::Append<Mle::TlvRequestTlv>(aMessage, mMleTlvsBuf, mMleTlvsBufLength);
    aMessage.AppendBytes(mExtraTlvsBuf, mExtraTlvsBufLength);
}

void TestLinkMetricsInitiator(void)
{
    TestLinkMetrics                  &testInstance = TestLinkMetrics::GetInstance();
    LinkMetrics::LinkMetricsInitiator initiator(TestLinkMetrics::TestSendMleDataRequest,
                                                TestLinkMetrics::TestNewMleMessage, TestLinkMetrics::TestSendMleMessage,
                                                TestLinkMetrics::TestFindNeighbor, &testInstance);

    otError error;

    // Test Query
    // - Initiator queries with wrong IP address
    Ip6::Address testIpAddr;
    testIpAddr.FromString("fde5:8dba:82e1:1:416:993c:8399:35ab");
    error = initiator.Query(testIpAddr, 0, nullptr);
    VerifyOrQuit(error == kErrorUnknownNeighbor);

    // - Initiator queries with non-existed neighbor
    testIpAddr.FromString("fe80::54db:881c:3845:57f4");
    error = initiator.Query(testIpAddr, 0, nullptr);
    VerifyOrQuit(error = kErrorUnknownNeighbor);

    // - Set a neighbor
    Mac::Address macAddress;
    testIpAddr.GetIid().ConvertToMacAddress(macAddress);
    testInstance.mNeighbors[0].SetExtAddress(macAddress.GetExtended());
    testInstance.mNeighbors[0].SetVersion(kThreadVersion1p2);

    // - Initiator queries with invalid args
    LinkMetrics::Metrics metrics;
    metrics.Clear();
    metrics.mPduCount = true;
    error             = initiator.Query(testIpAddr, 1, &metrics);
    VerifyOrQuit(error == kErrorInvalidArgs);

    // - Valid query
    metrics.Clear();
    metrics.mPduCount = true;
    error             = initiator.Query(testIpAddr, 0, &metrics);
    VerifyOrQuit(error == kErrorNone);
    VerifyOrQuit(testInstance.CheckMleDataRequest(0, metrics));

    // TODO: add more unit tests for LinkMetricsInitiator
}

void TestLinkMetricsSubject(void)
{
    TestLinkMetrics                  &testInstance = TestLinkMetrics::GetInstance();
    LinkMetrics::LinkMetricsInitiator initiator(TestLinkMetrics::TestSendMleDataRequest,
                                                TestLinkMetrics::TestNewMleMessage, TestLinkMetrics::TestSendMleMessage,
                                                TestLinkMetrics::TestFindNeighbor, &testInstance);
    LinkMetrics::LinkMetricsSubject subject(TestLinkMetrics::TestComputeLinkMargin, TestLinkMetrics::TestNewMleMessage,
                                            TestLinkMetrics::TestSendMleMessage,
                                            TestLinkMetrics::TestConfigureEnhAckProbing, &testInstance);

    otError error;

    // Test HandleQueryAndAppendReport
    // - Set a neighbor for initiator to generate a query
    Ip6::Address testIpAddr;
    testIpAddr.FromString("fe80::54db:881c:3845:57f4");
    Mac::Address macAddress;
    testIpAddr.GetIid().ConvertToMacAddress(macAddress);
    testInstance.mNeighbors[0].SetExtAddress(macAddress.GetExtended());
    testInstance.mNeighbors[0].SetVersion(kThreadVersion1p2);
    LinkMetrics::Metrics metrics;
    metrics.Clear();
    metrics.mPduCount = true;
    error             = initiator.Query(testIpAddr, 0, &metrics);
    VerifyOrQuit(error == kErrorNone);

    // - Generate the query message
    constexpr uint8_t kCommandMleDataRequest = 7;
    constexpr uint8_t kCommandDataResponse   = 8;
    Message          *reqMsg = TestLinkMetrics::TestNewMleMessage(kCommandMleDataRequest, &testInstance);
    VerifyOrQuit(reqMsg != nullptr);
    reqMsg->SetSubType(Message::kSubTypeMleDataRequest);
    testInstance.AppendDataRequestTlvsToMessage(*reqMsg);

    // - Call HandleQueryAndAppendReport and get report
    Message *rspMsg = TestLinkMetrics::TestNewMleMessage(kCommandDataResponse, &testInstance);
    VerifyOrQuit(rspMsg != nullptr);
    reqMsg->AddRss(-10);
    reqMsg->AddRss(-8);
    reqMsg->AddLqi(60);
    reqMsg->AddLqi(120);
    reqMsg->SetOffset(2); // Skip security header and command
    error = subject.HandleQueryAndAppendReport(*rspMsg, *reqMsg, testInstance.mNeighbors[1]);
    VerifyOrQuit(error == kErrorNone);

    FreeMessage(reqMsg);
    FreeMessage(rspMsg);

    // TODO: add more unit tests for LinkMetricsSubject
}

#endif // OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE && OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE

} // namespace ot

int main(void)
{
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE && OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    ot::TestLinkMetricsScaling();
    ot::TestLinkMetricsInitiator();
    ot::TestLinkMetricsSubject();
#endif

    printf("\nAll tests passed\n");
    return 0;
}
