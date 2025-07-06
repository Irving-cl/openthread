/*
 *  Copyright (c) 2025, The OpenThread Authors.
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

#include <openthread/config.h>

#include "test_platform.h"
#include "test_util.hpp"

#include "common/string.hpp"
#include "net/dns_types.hpp"

namespace ot {

static void CheckSplitFullDnsName(const char *aFullName,
                                  bool        aIsServiceInstance,
                                  bool        aIsService,
                                  bool        aIsHost,
                                  const char *aInstanceName,
                                  const char *aServiceName,
                                  const char *aHostName,
                                  const char *aDomain)
{
    Dns::NameInfo info;
    char          buffer[Dns::Name::kMaxNameSize];
    uint16_t      fullNameLength = StringLength(aFullName, Dns::Name::kMaxNameSize);

    info = Dns::SplitFullDnsName(aFullName);

    VerifyOrQuit(aIsServiceInstance == info.IsServiceInstance());
    VerifyOrQuit(aIsService == info.IsService());
    VerifyOrQuit(aIsHost == info.IsHost());
    VerifyOrQuit(Dns::NameData(aInstanceName, strlen(aInstanceName)) == info.mInstanceName);
    VerifyOrQuit(Dns::NameData(aServiceName, strlen(aServiceName)) == info.mServiceName);
    VerifyOrQuit(Dns::NameData(aHostName, strlen(aHostName)) == info.mHostName);
    VerifyOrQuit(Dns::NameData(aDomain, strlen(aDomain)) == info.mDomain);

    memcpy(buffer, aFullName, fullNameLength);
    buffer[fullNameLength] = '.';

    info = Dns::SplitFullDnsName(buffer);

    VerifyOrQuit(aIsServiceInstance == info.IsServiceInstance());
    VerifyOrQuit(aIsService == info.IsService());
    VerifyOrQuit(aIsHost == info.IsHost());
    VerifyOrQuit(Dns::NameData(aInstanceName, strlen(aInstanceName)) == info.mInstanceName);
    VerifyOrQuit(Dns::NameData(aServiceName, strlen(aServiceName)) == info.mServiceName);
    VerifyOrQuit(Dns::NameData(aHostName, strlen(aHostName)) == info.mHostName);
    VerifyOrQuit(Dns::NameData(aDomain, strlen(aDomain)) == info.mDomain);
}

void TestDnssdUtils(void)
{
    // Check service instance names
    CheckSplitFullDnsName("ins1._ipps._tcp.default.service.arpa", true, false, false, "ins1", "_ipps._tcp", "",
                          "default.service.arpa.");
    CheckSplitFullDnsName("Instance Name._ipps._tcp.default.service.arpa", true, false, false, "Instance Name",
                          "_ipps._tcp", "", "default.service.arpa.");
    CheckSplitFullDnsName("Instance.Name.With.Dots._ipps._tcp.default.service.arpa", true, false, false,
                          "Instance.Name.With.Dots", "_ipps._tcp", "", "default.service.arpa.");

    // Check service names
    CheckSplitFullDnsName("_ipps._tcp.default.service.arpa", false, true, false, "", "_ipps._tcp", "",
                          "default.service.arpa.");
    CheckSplitFullDnsName("_meshcop._udp.default.service.arpa", false, true, false, "", "_meshcop._udp", "",
                          "default.service.arpa.");

    // Check invalid service names
    CheckSplitFullDnsName("_meshcop._abc.default.service.arpa", false, false, true, "", "", "_meshcop",
                          "_abc.default.service.arpa.");
    CheckSplitFullDnsName("_tcp.default.service.arpa", false, false, true, "", "", "_tcp", "default.service.arpa.");

    // Check host names
    CheckSplitFullDnsName("abc.example.com", false, false, true, "", "", "abc", "example.com.");
    CheckSplitFullDnsName("example.com", false, false, true, "", "", "example", "com.");
    CheckSplitFullDnsName("com", false, false, true, "", "", "com", ".");
    CheckSplitFullDnsName("", false, false, true, "", "", "", ".");
}

} // namespace ot

int main(void)
{
    ot::TestDnssdUtils();

    printf("All tests passed\n");
    return 0;
}
