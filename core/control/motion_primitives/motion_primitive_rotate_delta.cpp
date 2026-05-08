#include "core/control/motion_primitives/motion_primitive_rotate_delta.hpp"
#include "core/control/motion_primitives/motion_primitives_tuning.hpp"

namespace motion_primitive_rotate_delta
{
  namespace
  {
    constexpr std::int64_t k_pi_scaled = 3141593;
    constexpr std::int64_t k_milliseconds_per_second = 1000LL;

    std::int32_t absolute_i32(std::int32_t value)
    {
      if (value < 0)
      {
        return -value;
      }

      return value;
    }

    std::int64_t absolute_i64(std::int64_t value)
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

    std::int32_t convert_urad_to_mdeg(std::int64_t value_urad)
    {
      return static_cast<std::int32_t>((value_urad * 180000) / k_pi_scaled);
    }

    std::int64_t compute_remaining_rotation_urad(const motion_primitives_common::state &primitive_state)
    {
      const std::int64_t heading_delta_urad = static_cast<std::int64_t>(primitive_state.latest_pose.heading_urad) - static_cast<std::int64_t>(primitive_state.snapshot.phase_start_pose.heading_urad);
      return primitive_state.active_request.rotate_delta.target_rotation_urad - heading_delta_urad;
    }

    std::int32_t compute_measured_yaw_rate_mdps(const motion_primitives_common::state &primitive_state)
    {
      if (!primitive_state.has_previous_latest_pose)
      {
        return 0;
      }

      if (!primitive_state.has_latest_pose)
      {
        return 0;
      }

      const std::uint32_t previous_time_ms = primitive_state.previous_latest_pose.time_ms;
      const std::uint32_t current_time_ms = primitive_state.latest_pose.time_ms;

      if (current_time_ms <= previous_time_ms)
      {
        return 0;
      }

      const std::int64_t delta_heading_urad =
          static_cast<std::int64_t>(primitive_state.latest_pose.heading_urad) -
          static_cast<std::int64_t>(primitive_state.previous_latest_pose.heading_urad);
      const std::int64_t delta_rotation_mdeg = convert_urad_to_mdeg(delta_heading_urad);
      const std::int64_t delta_time_ms = static_cast<std::int64_t>(current_time_ms - previous_time_ms);
      const std::int64_t measured_yaw_rate_mdps = (delta_rotation_mdeg * k_milliseconds_per_second) / delta_time_ms;
      return static_cast<std::int32_t>(measured_yaw_rate_mdps);
    }

    std::int32_t compute_max_yaw_rate_mdeg_s(const motion_primitives_common::state &primitive_state, std::int64_t remaining_rotation_urad)
    {
      std::int32_t max_yaw_rate_mdeg_s = absolute_i32(primitive_state.active_request.rotate_delta.yaw_rate_mdeg_s);
      const bool is_in_final_window = absolute_i64(remaining_rotation_urad) <= motion_primitives_tuning::k_rotate_delta_final_window_urad;

      if (max_yaw_rate_mdeg_s < motion_primitives_tuning::k_rotate_delta_min_request_yaw_rate_mdeg_s)
      {
        max_yaw_rate_mdeg_s = motion_primitives_tuning::k_rotate_delta_min_request_yaw_rate_mdeg_s;
      }

      if (is_in_final_window && max_yaw_rate_mdeg_s > motion_primitives_tuning::k_rotate_delta_final_max_yaw_rate_mdeg_s)
      {
        max_yaw_rate_mdeg_s = motion_primitives_tuning::k_rotate_delta_final_max_yaw_rate_mdeg_s;
      }

      return max_yaw_rate_mdeg_s;
    }

    std::int32_t compute_min_yaw_rate_mdeg_s(std::int32_t max_yaw_rate_mdeg_s)
    {
      if (motion_primitives_tuning::k_rotate_delta_min_yaw_rate_mdeg_s < max_yaw_rate_mdeg_s)
      {
        return motion_primitives_tuning::k_rotate_delta_min_yaw_rate_mdeg_s;
      }

      return max_yaw_rate_mdeg_s;
    }

    std::int32_t compute_pd_yaw_rate_mdeg_s(std::int32_t remaining_rotation_mdeg, std::int32_t measured_yaw_rate_mdps)
    {
      const std::int32_t proportional_yaw_rate_mdeg_s = remaining_rotation_mdeg / motion_primitives_tuning::k_rotate_delta_p_divisor;
      const std::int32_t damping_yaw_rate_mdeg_s = measured_yaw_rate_mdps / motion_primitives_tuning::k_rotate_delta_d_divisor;
      return proportional_yaw_rate_mdeg_s - damping_yaw_rate_mdeg_s;
    }

    std::int32_t apply_min_yaw_rate_if_needed(std::int32_t commanded_yaw_rate_mdeg_s, std::int32_t min_yaw_rate_mdeg_s)
    {
      if (commanded_yaw_rate_mdeg_s == 0)
      {
        return 0;
      }

      if (absolute_i32(commanded_yaw_rate_mdeg_s) >= min_yaw_rate_mdeg_s)
      {
        return commanded_yaw_rate_mdeg_s;
      }

      return sign_i32(commanded_yaw_rate_mdeg_s) * min_yaw_rate_mdeg_s;
    }

    std::int32_t compute_commanded_yaw_rate_mdeg_s(const motion_primitives_common::state &primitive_state)
    {
      const std::int64_t remaining_rotation_urad = compute_remaining_rotation_urad(primitive_state);
      const std::int32_t remaining_rotation_mdeg = convert_urad_to_mdeg(remaining_rotation_urad);
      const std::int32_t measured_yaw_rate_mdps = compute_measured_yaw_rate_mdps(primitive_state);
      const std::int32_t max_yaw_rate_mdeg_s = compute_max_yaw_rate_mdeg_s(primitive_state, remaining_rotation_urad);
      const std::int32_t min_yaw_rate_mdeg_s = compute_min_yaw_rate_mdeg_s(max_yaw_rate_mdeg_s);
      std::int32_t commanded_yaw_rate_mdeg_s = compute_pd_yaw_rate_mdeg_s(remaining_rotation_mdeg, measured_yaw_rate_mdps);

      commanded_yaw_rate_mdeg_s = clamp_i32(commanded_yaw_rate_mdeg_s, -max_yaw_rate_mdeg_s, max_yaw_rate_mdeg_s);
      return apply_min_yaw_rate_if_needed(commanded_yaw_rate_mdeg_s, min_yaw_rate_mdeg_s);
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

    const std::int64_t remaining_rotation_urad = compute_remaining_rotation_urad(primitive_state);
    const std::int32_t measured_yaw_rate_mdps = compute_measured_yaw_rate_mdps(primitive_state);
    const bool is_within_target_tolerance = absolute_i64(remaining_rotation_urad) <= motion_primitives_tuning::k_rotate_delta_target_tolerance_urad;
    const bool has_low_yaw_rate = absolute_i32(measured_yaw_rate_mdps) <= motion_primitives_tuning::k_rotate_delta_settle_yaw_rate_mdps;

    if (is_within_target_tolerance && has_low_yaw_rate)
    {
      return motion_primitives_common::tick_result::begin_settling;
    }

    const std::int32_t commanded_yaw_rate_mdeg_s = compute_commanded_yaw_rate_mdeg_s(primitive_state);
    motion_primitives_common::send_motion_command(true, primitive_state.active_request.rotate_delta.linear_velocity_mm_s, commanded_yaw_rate_mdeg_s, now_ms);
    return motion_primitives_common::tick_result::keep_running;
  }
}
