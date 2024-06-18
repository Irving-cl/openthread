/*
 *  Copyright (c) 2016, The OpenThread Authors.
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
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include "platform-simulation.h"

#if OPENTHREAD_SIMULATION_VIRTUAL_TIME == 0

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <libgen.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <openthread/tasklet.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/radio.h>

#include "simul_utils.h"

uint32_t gNodeId = 1;

extern bool        gPlatformPseudoResetWasRequested;
extern otRadioCaps gRadioCaps;

static volatile bool gTerminate = false;

static void handleSignal(int aSignal)
{
    OT_UNUSED_VARIABLE(aSignal);

    gTerminate = true;
}

/**
 * Defines the argument return values.
 *
 */
enum
{
    OT_SIM_OPT_HELP               = 'h',
    OT_SIM_OPT_ENABLE_ENERGY_SCAN = 'E',
    OT_SIM_OPT_LOCAL_INTERFACE    = 'L',
    OT_SIM_OPT_SLEEP_TO_TX        = 't',
    OT_SIM_OPT_TIME_SPEED         = 's',
    OT_SIM_OPT_LOG_FILE           = 'l',
    OT_SIM_OPT_UNKNOWN            = '?',
};

static void PrintUsage(const char *aProgramName, int aExitCode)
{
    fprintf(stderr,
            "Syntax:\n"
            "    %s [Options] NodeId\n"
            "Options:\n"
            "    -h --help                  Display this usage information.\n"
            "    -L --local-interface=val   The address or name of the netif to simulate Thread radio.\n"
            "    -E --enable-energy-scan    Enable energy scan capability.\n"
            "    -t --sleep-to-tx           Let radio support direct transition from sleep to TX with CSMA.\n"
            "    -s --time-speed=val        Speed up the time in simulation.\n"
#if (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED)
            "    -l --log-file=name         File name to write logs.\n"
#endif
            ,
            aProgramName);

    exit(aExitCode);
}

void otSysInit(int aArgCount, char *aArgVector[])
{
    char    *endptr;
    uint32_t speedUpFactor = 1;

    static const struct option long_options[] = {
        {"help", no_argument, 0, OT_SIM_OPT_HELP},
        {"enable-energy-scan", no_argument, 0, OT_SIM_OPT_ENABLE_ENERGY_SCAN},
        {"sleep-to-tx", no_argument, 0, OT_SIM_OPT_SLEEP_TO_TX},
        {"time-speed", required_argument, 0, OT_SIM_OPT_TIME_SPEED},
        {"local-interface", required_argument, 0, OT_SIM_OPT_LOCAL_INTERFACE},
#if (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED)
        {"log-file", required_argument, 0, OT_SIM_OPT_LOG_FILE},
#endif
        {0, 0, 0, 0},
    };

#if (OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED)
    static const char options[] = "Ehts:L:l:";
#else
    static const char options[] = "Ehts:L:";
#endif

    if (gPlatformPseudoResetWasRequested)
    {
        gPlatformPseudoResetWasRequested = false;
        return;
    }

    optind = 1;

    while (true)
    {
        int c = getopt_long(aArgCount, aArgVector, options, long_options, NULL);

        if (c == -1)
        {
            break;
        }

        switch (c)
        {
        case OT_SIM_OPT_UNKNOWN:
            PrintUsage(aArgVector[0], EXIT_FAILURE);
            break;
        case OT_SIM_OPT_HELP:
            PrintUsage(aArgVector[0], EXIT_SUCCESS);
            break;
        case OT_SIM_OPT_ENABLE_ENERGY_SCAN:
            gRadioCaps |= OT_RADIO_CAPS_ENERGY_SCAN;
            break;
        case OT_SIM_OPT_SLEEP_TO_TX:
            gRadioCaps |= OT_RADIO_CAPS_SLEEP_TO_TX;
            break;
        case OT_SIM_OPT_LOCAL_INTERFACE:
            gLocalInterface = optarg;
            break;
        case OT_SIM_OPT_TIME_SPEED:
            speedUpFactor = (uint32_t)strtol(optarg, &endptr, 10);
            if (*endptr != '\0' || speedUpFactor == 0)
            {
                fprintf(stderr, "Invalid value for TimerSpeedUpFactor: %s\n", optarg);
                exit(EXIT_FAILURE);
            }
            break;
#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED
        case OT_SIM_OPT_LOG_FILE:
            platformLoggingSetFileName(optarg);
            break;
#endif
        default:
            break;
        }
    }

    if (optind != aArgCount - 1)
    {
        PrintUsage(aArgVector[0], EXIT_FAILURE);
    }

    gNodeId = (uint32_t)strtol(aArgVector[optind], &endptr, 0);

    if (*endptr != '\0' || gNodeId < 1 || gNodeId > MAX_NETWORK_SIZE)
    {
        fprintf(stderr, "Invalid NodeId: %s\n", aArgVector[optind]);
        exit(EXIT_FAILURE);
    }

    signal(SIGTERM, &handleSignal);
    signal(SIGHUP, &handleSignal);

    platformLoggingInit(basename(aArgVector[0]));
    platformAlarmInit(speedUpFactor);
    platformRadioInit();
#if OPENTHREAD_CONFIG_RADIO_LINK_TREL_ENABLE
    platformTrelInit(speedUpFactor);
#endif
#if OPENTHREAD_SIMULATION_IMPLEMENT_INFRA_IF
    platformInfraIfInit();
#endif
    platformRandomInit();
}

bool otSysPseudoResetWasRequested(void) { return gPlatformPseudoResetWasRequested; }

void otSysDeinit(void)
{
    platformRadioDeinit();
#if OPENTHREAD_CONFIG_RADIO_LINK_TREL_ENABLE
    platformTrelDeinit();
#endif
#if OPENTHREAD_SIMULATION_IMPLEMENT_INFRA_IF
    platformInfraIfDeinit();
#endif
    platformLoggingDeinit();
}

void otSysProcessDrivers(otInstance *aInstance)
{
    fd_set         read_fds;
    fd_set         write_fds;
    fd_set         error_fds;
    int            max_fd = -1;
    struct timeval timeout;
    int            rval;

    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&error_fds);

    platformUartUpdateFdSet(&read_fds, &write_fds, &error_fds, &max_fd);
    platformAlarmUpdateTimeout(&timeout);
    platformRadioUpdateFdSet(&read_fds, &write_fds, &timeout, &max_fd);
#if OPENTHREAD_CONFIG_RADIO_LINK_TREL_ENABLE
    platformTrelUpdateFdSet(&read_fds, &write_fds, &timeout, &max_fd);
#endif
#if OPENTHREAD_SIMULATION_IMPLEMENT_INFRA_IF
    platformInfraIfUpdateFdSet(&read_fds, &write_fds, &max_fd);
#endif
#if OPENTHREAD_CONFIG_MULTICAST_DNS_ENABLE && OPENTHREAD_SIMULATION_MDNS_SOCKET_IMPLEMENT_POSIX
    platformMdnsSocketUpdateFdSet(&read_fds, &max_fd);
#endif

#if OPENTHREAD_CONFIG_BLE_TCAT_ENABLE
    platformBleUpdateFdSet(&read_fds, &write_fds, &timeout, &max_fd);
#endif

    if (otTaskletsArePending(aInstance))
    {
        timeout.tv_sec  = 0;
        timeout.tv_usec = 0;
    }

    rval = select(max_fd + 1, &read_fds, &write_fds, &error_fds, &timeout);

    if (rval >= 0)
    {
        platformUartProcess();
        platformRadioProcess(aInstance, &read_fds, &write_fds);
#if OPENTHREAD_CONFIG_BLE_TCAT_ENABLE
        platformBleProcess(aInstance, &read_fds, &write_fds);
#endif
    }
    else if (errno != EINTR)
    {
        perror("select");
        exit(EXIT_FAILURE);
    }

    platformAlarmProcess(aInstance);
#if OPENTHREAD_CONFIG_RADIO_LINK_TREL_ENABLE
    platformTrelProcess(aInstance, &read_fds, &write_fds);
#endif
#if OPENTHREAD_SIMULATION_IMPLEMENT_INFRA_IF
    platformInfraIfProcess(aInstance, &read_fds, &write_fds);
#endif
#if OPENTHREAD_CONFIG_MULTICAST_DNS_ENABLE && OPENTHREAD_SIMULATION_MDNS_SOCKET_IMPLEMENT_POSIX
    platformMdnsSocketProcess(aInstance, &read_fds);
#endif

    if (gTerminate)
    {
        exit(0);
    }
}

#endif // OPENTHREAD_SIMULATION_VIRTUAL_TIME == 0
