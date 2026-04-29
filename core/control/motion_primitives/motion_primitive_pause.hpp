#pragma once

#include "core/control/motion_primitives/motion_primitives_common.hpp"

namespace motion_primitive_pause
{
  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, std::uint32_t now_ms);
}
