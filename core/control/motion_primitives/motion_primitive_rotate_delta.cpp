#include "core/control/motion_primitives/motion_primitive_rotate_delta.hpp"

namespace motion_primitive_rotate_delta
{
  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, const motion_primitives::input_snapshot &input, std::uint32_t now_ms)
  {
    if (!input.pose.has_pose)
    {
      return motion_primitives_common::tick_result::complete_failure;
    }

    if (motion_primitives_common::has_elapsed(now_ms, primitive_state.snapshot.stop_time_ms))
    {
      return motion_primitives_common::tick_result::complete_timeout;
    }

    motion_primitives_common::send_motion_command(true, primitive_state.active_request.rotate_delta.linear_velocity_mm_s, primitive_state.active_request.rotate_delta.yaw_rate_mdeg_s, now_ms);

    if (!motion_primitives_common::has_reached_rotation_target(primitive_state.snapshot.phase_start_pose, input.pose, primitive_state.active_request.rotate_delta.target_rotation_urad))
    {
      return motion_primitives_common::tick_result::keep_running;
    }

    return motion_primitives_common::tick_result::begin_settling;
  }
}
