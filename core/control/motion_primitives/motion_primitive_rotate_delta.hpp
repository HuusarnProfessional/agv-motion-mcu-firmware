#pragma once

#include "core/control/motion_primitives/motion_primitives_common.hpp"

namespace motion_primitive_rotate_delta
{
  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, const motion_primitives::input_snapshot &input, std::uint32_t now_ms);
}
