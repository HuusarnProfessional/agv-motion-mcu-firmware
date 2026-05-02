#include "core/control/motion_primitives/motion_primitive_drive_forward.hpp"

namespace motion_primitive_drive_forward
{
  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, const motion_primitives::input_snapshot &input, std::uint32_t now_ms)
  {
    if (motion_primitives_common::has_elapsed(now_ms, primitive_state.snapshot.stop_time_ms))
    {
      return motion_primitives_common::tick_result::complete_timeout;
    }

    motion_primitives_common::send_motion_command(true, primitive_state.active_request.drive_forward.velocity_mm_s, 0, now_ms);

    const bool has_fresh_pose = motion_primitives_common::update_latest_pose_if_fresh(primitive_state, input.pose, now_ms);

    if (!has_fresh_pose)
    {
      if (motion_primitives_common::has_latest_pose_timed_out(primitive_state, now_ms))
      {
        return motion_primitives_common::tick_result::complete_timeout;
      }

      return motion_primitives_common::tick_result::keep_running;
    }

    if (!motion_primitives_common::has_reached_forward_target(primitive_state.snapshot.phase_start_pose, primitive_state.latest_pose, primitive_state.active_request.drive_forward.target_distance_um))
    {
      return motion_primitives_common::tick_result::keep_running;
    }

    return motion_primitives_common::tick_result::begin_settling;
  }
}
