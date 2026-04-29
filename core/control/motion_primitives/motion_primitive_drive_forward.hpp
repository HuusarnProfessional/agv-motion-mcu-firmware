#pragma once

#include "core/control/motion_primitives/motion_primitives_common.hpp"

namespace motion_primitive_drive_forward
{
  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, const motion_primitives::input_snapshot &input, std::uint32_t now_ms);
}
