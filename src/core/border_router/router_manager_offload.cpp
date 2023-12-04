
#include "border_router/routing_manager.hpp"

#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/locator_getters.hpp"
#include "common/log.hpp"

namespace ot {

namespace BorderRouter {

RegisterLogModule("RoutingManagerOffload");

RoutingManager::RoutingManager(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mIsRunning(false)
    , mIsEnabled(false)
    , mInfraIf(aInstance)
    , mOmrPrefixManager(aInstance)
    , mOnLinkPrefixManager(aInstance)
{
}

} // namespace BorderRouter

} // namespace ot

#endif // OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE