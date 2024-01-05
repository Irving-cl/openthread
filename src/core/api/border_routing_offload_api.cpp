
/**
 * @file
 *   This file implements the OpenThread Border Routing Manager API.
 */

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#include <openthread/border_routing.h>
#include <openthread/platform/border_routing.h>

#include "border_router/routing_manager_offload.hpp"
#include "instance/instance.hpp"

using namespace ot;

otError otBorderRoutingInit(otInstance *aInstance, uint32_t aInfraIfIndex, bool aInfraIfIsRunning)
{
    return AsCoreType(aInstance).Get<BorderRouter::RoutingManagerOffload>().Init(aInfraIfIndex, aInfraIfIsRunning);
}

otError otBorderRoutingSetEnabled(otInstance *aInstance, bool aEnabled)
{
    return AsCoreType(aInstance).Get<BorderRouter::RoutingManagerOffload>().SetEnabled(aEnabled);
}

otBorderRoutingState otBorderRoutingGetState(otInstance *aInstance)
{
    return MapEnum(AsCoreType(aInstance).Get<BorderRouter::RoutingManagerOffload>().GetState());
}

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE