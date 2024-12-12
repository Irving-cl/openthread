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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <map>

#include <openthread/border_agent.h>
#include <openthread/dataset.h>
#include <openthread/dataset_ftd.h>
#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>

#include "gmock/gmock.h"

#include "fake_platform.hpp"

using namespace ot;

static const char kBorderAgentServiceType[]      = "_meshcop._udp";   ///< Border agent service type of mDNS
static const char kBorderAgentEpskcServiceType[] = "_meshcop-e._udp"; ///< Border agent ePSKc service

#if OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1
static const char kThreadVersionString[] = "1.1.1";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_2
static const char kThreadVersionString[] = "1.2.0";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_3
static const char kThreadVersionString[] = "1.3.0";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_3_1
static const char kThreadVersionString[] = "1.3.1";
#elif OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_4
static const char kThreadVersionString[] = "1.4.0";
#endif

using TxtEntryMap = std::map<std::string, std::vector<uint8_t>>;

TxtEntryMap ParseTxtEntires(const std::vector<uint8_t> &aTxtData)
{
    static constexpr char kKeyValueSeparator = '=';

    TxtEntryMap txtEntryMap;
    uint16_t    index = 0;

    // In MeshCoP service, there is no boolean attritute.
    while (index < aTxtData.size())
    {
        uint8_t  length        = aTxtData[index++];
        uint16_t keyStartIndex = index;

        while (keyStartIndex < index + length && index < aTxtData.size() && aTxtData[index] != kKeyValueSeparator)
        {
            index++;
        }
        assert(aTxtData[index] == kKeyValueSeparator && index < aTxtData.size());

        std::string key(reinterpret_cast<const char *>(&aTxtData[keyStartIndex]), index - keyStartIndex);
        index++;

        txtEntryMap.emplace(key,
                            std::vector<uint8_t>(aTxtData.begin() + index, aTxtData.begin() + keyStartIndex + length));
        index = keyStartIndex + length;
    }

    return txtEntryMap;
}

std::string GetStringFromBytes(const std::vector<uint8_t> &aBytes)
{
    assert(!aBytes.empty());

    return std::string(reinterpret_cast<const char *>(&aBytes[0]), aBytes.size());
}

void InitializeThreadNetwork(FakePlatform &aFakePlatform)
{
    otError error;

    constexpr otExtendedPanId extPanId = {.m8 = {0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0xca, 0xfe}};
    otOperationalDataset      dataset;
    otOperationalDatasetTlvs  datasetTlvs;

    // 0. Initialize the network
    error = otDatasetCreateNewNetwork(aFakePlatform.CurrentInstance(), &dataset);
    assert(error == OT_ERROR_NONE);

    dataset.mNetworkName = {"BA_UnitTest"};
    memcpy(dataset.mExtendedPanId.m8, extPanId.m8, sizeof(otExtendedPanId));

    otDatasetConvertToTlvs(&dataset, &datasetTlvs);
    error = otDatasetSetActiveTlvs(aFakePlatform.CurrentInstance(), &datasetTlvs);
    assert(error == OT_ERROR_NONE);

    error = otIp6SetEnabled(aFakePlatform.CurrentInstance(), /* aEnabled */ true);
    assert(error == OT_ERROR_NONE);
    error = otThreadSetEnabled(aFakePlatform.CurrentInstance(), /* aEnabled */ true);
    assert(error == OT_ERROR_NONE);

    aFakePlatform.GoInMs(10000);

    assert(otThreadGetDeviceRole(aFakePlatform.CurrentInstance()) == OT_DEVICE_ROLE_LEADER);
}

TEST(BorderAgent, MeshCopServicePublishedAndUnpublishedSuccessfully)
{
    FakePlatform fakePlatform;
    otError      error;
    otInstance  *instance = fakePlatform.CurrentInstance();

    // 0. Initialize the network
    InitializeThreadNetwork(fakePlatform);

    // 1. Enable the Border Agent Service Publisher
    otBorderAgentSetServicePublisherEnabled(instance, /* aEnabled */ true);

    const char                        kServiceInstanceName[] = "BorderAgentUnitTest";
    const char                        kProductName[]         = "UnitTestProduct";
    const char                        kVendorName[]          = "OpenThreadUnitTest";
    const uint8_t                     kVendorOui[]           = {0x01, 0x02, 0x03};
    const otBorderAgentVendorTxtEntry kVendorTxtEntries[]    = {
           {"vn", reinterpret_cast<const uint8_t *>(kVendorName), sizeof(kVendorName)},
           {"vo", kVendorOui, sizeof(kVendorOui)}};
    error = otBorderAgentSetMeshCopServiceValues(instance, kServiceInstanceName, kProductName, kVendorTxtEntries,
                                                 sizeof(kVendorTxtEntries) / sizeof(otBorderAgentVendorTxtEntry));
    EXPECT_EQ(error, OT_ERROR_NONE);

    fakePlatform.GetDnssd().SetState(OT_PLAT_DNSSD_READY);

    // 2. Check the service published
    auto &serviceRegistrationEntries = fakePlatform.GetDnssd().GetServiceRegistrationEntries();
    EXPECT_EQ(serviceRegistrationEntries.size(), 1);
    auto &entry = serviceRegistrationEntries.front();

    EXPECT_STREQ(entry.mService.mHostName, "");
    EXPECT_STREQ(entry.mService.mServiceType, kBorderAgentServiceType);
    std::string serviceInstanceNamePrefix(entry.mService.mServiceInstance, strlen(kServiceInstanceName));
    std::string serviceInstanceName(entry.mService.mServiceInstance);
    EXPECT_STREQ(serviceInstanceNamePrefix.c_str(), kServiceInstanceName);
    EXPECT_EQ(entry.mService.mPort, otBorderAgentGetUdpPort(instance));
    EXPECT_EQ(entry.mRequestId, 0);

    // 2.1 Check the Txt Data if service pulished
    TxtEntryMap txtEntryMap = ParseTxtEntires(entry.mTxtData);

    EXPECT_NE(txtEntryMap.find("vo"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("vn"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("mn"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("nn"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("xp"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("tv"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("xa"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("sb"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("pt"), txtEntryMap.end());
    EXPECT_NE(txtEntryMap.find("at"), txtEntryMap.end());

    EXPECT_EQ(txtEntryMap["vo"], std::vector<uint8_t>(kVendorOui, kVendorOui + sizeof(kVendorOui)));
    EXPECT_STREQ(GetStringFromBytes(txtEntryMap["vn"]).c_str(), kVendorName);
    EXPECT_STREQ(GetStringFromBytes(txtEntryMap["mn"]).c_str(), kProductName);
    EXPECT_STREQ(GetStringFromBytes(txtEntryMap["nn"]).c_str(), otThreadGetNetworkName(instance));
    const otExtendedPanId *extPanId = otThreadGetExtendedPanId(instance);
    EXPECT_EQ(txtEntryMap["xp"], std::vector<uint8_t>(extPanId->m8, extPanId->m8 + sizeof(otExtendedPanId)));
    EXPECT_STREQ(GetStringFromBytes(txtEntryMap["tv"]).c_str(), kThreadVersionString);
    uint32_t partitionId    = otThreadGetPartitionId(instance);
    uint8_t *partitionIdPtr = reinterpret_cast<uint8_t *>(&partitionId);
    EXPECT_EQ(txtEntryMap["pt"],
              std::vector<uint8_t>(partitionIdPtr, partitionIdPtr + sizeof(partitionId) / sizeof(uint8_t)));

    // Invoke callback
    entry.mCallback(instance, entry.mRequestId, OT_ERROR_NONE);
    serviceRegistrationEntries.clear();

    // 3. Disable Border Agent Service Publisher
    otBorderAgentSetServicePublisherEnabled(instance, /* aEnabled */ false);
    fakePlatform.GoInMs(1);

    // 4. Check the service unregistration
    auto &serviceUnregistrationEntries = fakePlatform.GetDnssd().GetServiceUnregistrationEntries();
    EXPECT_EQ(serviceUnregistrationEntries.size(), 1);
    auto &entryUnreg = serviceUnregistrationEntries.front();
    EXPECT_STREQ(entryUnreg.mService.mHostName, "");
    EXPECT_STREQ(entryUnreg.mService.mServiceInstance, serviceInstanceName.c_str());
    EXPECT_STREQ(entryUnreg.mService.mServiceType, kBorderAgentServiceType);
    EXPECT_EQ(entryUnreg.mTxtData.size(), 0);

    // Invoke callback
    entry.mCallback(instance, entry.mRequestId, OT_ERROR_NONE);
    serviceUnregistrationEntries.clear();
}

TEST(BorderAgent, EpskcServicePublishedAndUnpublishedSuccessfully)
{
    FakePlatform fakePlatform;
    otError      error;
    otInstance  *instance = fakePlatform.CurrentInstance();

    // 0. Initialize the network
    InitializeThreadNetwork(fakePlatform);

    // 1. Enable the Border Agent Service Publisher
    otBorderAgentSetServicePublisherEnabled(instance, /* aEnabled */ true);

    const char                        kServiceInstanceName[] = "BorderAgentUnitTest";
    const char                        kProductName[]         = "UnitTestProduct";
    const char                        kVendorName[]          = "OpenThreadUnitTest";
    const uint8_t                     kVendorOui[]           = {0x01, 0x02, 0x03};
    const otBorderAgentVendorTxtEntry kVendorTxtEntries[]    = {
           {"vn", reinterpret_cast<const uint8_t *>(kVendorName), sizeof(kVendorName)},
           {"vo", kVendorOui, sizeof(kVendorOui)}};
    error = otBorderAgentSetMeshCopServiceValues(instance, kServiceInstanceName, kProductName, kVendorTxtEntries,
                                                 sizeof(kVendorTxtEntries) / sizeof(otBorderAgentVendorTxtEntry));
    EXPECT_EQ(error, OT_ERROR_NONE);
    fakePlatform.GetDnssd().SetState(OT_PLAT_DNSSD_READY);

    // Invoke callback
    auto &serviceRegistrationEntries = fakePlatform.GetDnssd().GetServiceRegistrationEntries();
    EXPECT_EQ(serviceRegistrationEntries.size(), 1);
    auto &entry = serviceRegistrationEntries.front();

    entry.mCallback(instance, entry.mRequestId, OT_ERROR_NONE);
    serviceRegistrationEntries.clear();

    // 2. Set Ephemeral Key
    const char kEphemeralKey[] = "EpskcUnitTest";
    error = otBorderAgentSetEphemeralKey(instance, kEphemeralKey, 0 /* aTimeout */, 0 /* aUdpPort */);
    EXPECT_EQ(error, OT_ERROR_NONE);

    fakePlatform.GoInMs(1);

    // 3. Check the Epskc service published
    EXPECT_EQ(serviceRegistrationEntries.size(), 1);
    entry = serviceRegistrationEntries.front();
    std::string serviceInstanceNamePrefix(entry.mService.mServiceInstance, strlen(kServiceInstanceName));
    std::string serviceInstanceName(entry.mService.mServiceInstance);
    EXPECT_STREQ(serviceInstanceNamePrefix.c_str(), kServiceInstanceName);
    EXPECT_STREQ(entry.mService.mServiceType, kBorderAgentEpskcServiceType);
    EXPECT_EQ(entry.mService.mPort, otBorderAgentGetUdpPort(instance));

    // Invoke callback
    entry.mCallback(instance, entry.mRequestId, OT_ERROR_NONE);
    serviceRegistrationEntries.clear();

    // 4. Clear Ephemeral Key
    otBorderAgentClearEphemeralKey(instance);
    fakePlatform.GoInMs(1);

    // 5. Check the Epskc service unregistration
    auto &serviceUnregistrationEntries = fakePlatform.GetDnssd().GetServiceUnregistrationEntries();
    EXPECT_EQ(serviceUnregistrationEntries.size(), 1);

    auto &entryUnreg = serviceUnregistrationEntries.front();
    EXPECT_STREQ(entryUnreg.mService.mHostName, "");
    EXPECT_STREQ(entryUnreg.mService.mServiceInstance, serviceInstanceName.c_str());
    EXPECT_STREQ(entryUnreg.mService.mServiceType, kBorderAgentEpskcServiceType);
    EXPECT_EQ(entryUnreg.mTxtData.size(), 0);

    // Invoke callback
    entry.mCallback(instance, entry.mRequestId, OT_ERROR_NONE);
    serviceRegistrationEntries.clear();
}
