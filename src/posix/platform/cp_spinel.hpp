
#ifndef POSIX_PLATFORM_CP_SPINEL_HPP_
#define POSIX_PLATFORM_CP_SPINEL_HPP_

#include "lib/spinel/radio_spinel.hpp"

namespace ot {
namespace Posix {

Spinel::RadioSpinel &GetSpinel(void);

} // namespace Posix
} // namespace ot

#endif /// POSIX_PLATFORM_CP_SPINEL_HPP_
