
#ifndef PLATFORM_POSIX_OFFLOAD_H_
#define PLATFORM_POSIX_OFFLOAD_H_

#include "openthread-posix-config.h"

#include <errno.h>
#include <net/if.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/logging.h>

#ifdef __cplusplus
extern "C" {
#endif

void platformNetifOffloadUpdateIp6Addresses(otNetifAddress *aAddressArray, uint8_t aNumAddr);

void platformNetifOffloadReceiveIp6(uint8_t *aBuf, uint16_t aLen);

otError platformNetifOffloadSendIp6(uint8_t *aBuf, uint16_t aLen);

#ifdef __cplusplus
}
#endif
#endif // PLATFORM_POSIX_OFFLOAD_H_