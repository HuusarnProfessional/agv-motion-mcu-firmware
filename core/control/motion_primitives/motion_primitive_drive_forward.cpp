#include "core/control/motion_primitives/motion_primitive_drive_forward.hpp"
#include "core/control/motion_primitives/motion_primitives_tuning.hpp"

namespace motion_primitive_drive_forward
{
  namespace
  {
    void reanchor_after_branch_change(motion_primitives_common::state &primitive_state)
    {
      if (!primitive_state.has_previous_latest_pose)
      {
        return;
      }

      const std::int64_t remaining_distance_um =
        motion_primitives_common::compute_remaining_forward_distance_um(
          primitive_state.snapshot.phase_start_pose,
          primitive_state.previous_latest_pose,
          primitive_state.active_request.drive_forward.target_distance_um);

      primitive_state.snapshot.phase_start_pose = primitive_state.latest_pose;
      primitive_state.active_request.drive_forward.target_distance_um = remaining_distance_um;
    }

    std::int32_t absolute_i32(std::int32_t value)
    {
      if (value < 0)
      {
        return -value;
      }

      return value;
    }

    std::int32_t sign_i32(std::int32_t value)
    {
      if (value > 0)
      {
        return 1;
      }

      if (value < 0)
      {
        return -1;
      }

      return 0;
    }

    std::int32_t clamp_i32(std::int32_t value, std::int32_t low, std::int32_t high)
    {
      if (value < low)
      {
        return low;
      }

      if (value > high)
      {
        return high;
      }

      return value;
    }

    std::int32_t compute_max_velocity_mm_s(const motion_primitives_common::state &primitive_state, std::int64_t remaining_distance_um)
    {
      std::int32_t max_velocity_mm_s = absolute_i32(primitive_state.active_request.drive_forward.velocity_mm_s);
      const bool is_in_final_window = motion_primitives_common::abs_i64(remaining_distance_um) <= motion_primitives_tuning::k_drive_forward_final_window_um;
      const bool exceeds_final_window_limit = max_velocity_mm_s > motion_primitives_tuning::k_drive_forward_final_max_velocity_mm_s;

      if (is_in_final_window && exceeds_final_window_limit)
      {
        max_velocity_mm_s = motion_primitives_tuning::k_drive_forward_final_max_velocity_mm_s;
      }

      return max_velocity_mm_s;
    }

    std::int32_t compute_commanded_velocity_mm_s(const motion_primitives_common::state &primitive_state, std::int64_t remaining_distance_um)
    {
      const std::int32_t max_velocity_mm_s = compute_max_velocity_mm_s(primitive_state, remaining_distance_um);
      std::int32_t min_velocity_mm_s = motion_primitives_tuning::k_drive_forward_min_velocity_mm_s;
      std::int32_t commanded_velocity_mm_s = static_cast<std::int32_t>(remaining_distance_um / motion_primitives_tuning::k_drive_forward_p_um_per_s_divisor);

      if (min_velocity_mm_s > max_velocity_mm_s)
      {
        min_velocity_mm_s = max_velocity_mm_s;
      }

      commanded_velocity_mm_s = clamp_i32(commanded_velocity_mm_s, -max_velocity_mm_s, max_velocity_mm_s);

      if (commanded_velocity_mm_s == 0)
      {
        return 0;
      }

      if (absolute_i32(commanded_velocity_mm_s) >= min_velocity_mm_s)
      {
        return commanded_velocity_mm_s;
      }

      return sign_i32(commanded_velocity_mm_s) * min_velocity_mm_s;
    }

    bool is_ready_to_begin_settling(std::int64_t remaining_distance_um, const motion_primitives::input_snapshot &input)
    {
      const bool is_within_target_tolerance = motion_primitives_common::abs_i64(remaining_distance_um) <= motion_primitives_tuning::k_drive_forward_target_tolerance_um;
      const bool has_low_encoder_translation = motion_primitives_common::abs_i64(input.encoder_state.encoder_motion_snapshot.translation) <= motion_primitives_tuning::k_drive_forward_settle_translation_threshold_um;

      if (!is_within_target_tolerance)
      {
        return false;
      }

      if (!has_low_encoder_translation)
      {
        return false;
      }

      return true;
    }
  }

  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, const motion_primitives::input_snapshot &input, std::uint32_t now_ms)
  {
    if (motion_primitives_common::has_elapsed(now_ms, primitive_state.snapshot.stop_time_ms))
    {
      return motion_primitives_common::tick_result::complete_timeout;
    }

    motion_primitives_common::update_latest_pose_if_fresh(primitive_state, input.pose, now_ms);

    if (motion_primitives_common::has_latest_pose_timed_out(primitive_state, now_ms))
    {
      return motion_primitives_common::tick_result::complete_timeout;
    }

    if (motion_primitives_common::did_latest_pose_branch_change(primitive_state))
    {
      reanchor_after_branch_change(primitive_state);
    }

    const std::int64_t remaining_distance_um = motion_primitives_common::compute_remaining_forward_distance_um(primitive_state.snapshot.phase_start_pose, primitive_state.latest_pose, primitive_state.active_request.drive_forward.target_distance_um);

    if (is_ready_to_begin_settling(remaining_distance_um, input))
    {
      return motion_primitives_common::tick_result::begin_settling;
    }

    const std::int32_t commanded_velocity_mm_s = compute_commanded_velocity_mm_s(primitive_state, remaining_distance_um);
    motion_primitives_common::send_motion_command(true, commanded_velocity_mm_s, 0, now_ms);
    return motion_primitives_common::tick_result::keep_running;
  }
}
