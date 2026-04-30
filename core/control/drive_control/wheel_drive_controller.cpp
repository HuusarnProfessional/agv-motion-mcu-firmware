#include "core/control/drive_control/wheel_drive_controller.hpp"

#include <array>
#include <cmath>

#include "core/api/motor_api.hpp"
#include "core/control/drive_control/wheel_drive_controller_tuning.hpp"
#include "core/control/safe_guard/safe_guard_pipeline.hpp"
#include "core/mechanical_config/mechanical_config.hpp"

namespace
{
  constexpr double k_pi = 3.14159265358979323846;
  wheel_drive_controller::rotation_drive_tuning g_rotation_drive_tuning_override = {};

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

  std::int64_t clamp_i64(std::int64_t value, std::int64_t low, std::int64_t high)
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

  bool is_rotation_only_command(const middleware_incoming_payloads::motion_command_payload_data &motion_command)
  {
    if (motion_command.linear_velocity_mm_s != 0)
    {
      return false;
    }

    if (motion_command.yaw_rate_mdeg_s == 0)
    {
      return false;
    }

    return true;
  }

  bool is_motion_command_stale(std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command)
  {
    if (now_ms < motion_command.received_time_ms)
    {
      return true;
    }

    if (now_ms - motion_command.received_time_ms > wheel_drive_controller_tuning::k_motion_command_timeout_ms)
    {
      return true;
    }

    return false;
  }

  bool is_feedback_grace_active(std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command)
  {
    if (now_ms < motion_command.received_time_ms)
    {
      return false;
    }

    if (now_ms - motion_command.received_time_ms <= wheel_drive_controller_tuning::k_feedback_not_ready_grace_ms)
    {
      return true;
    }

    return false;
  }

  bool is_encoder_sample_valid(const encoder_api::encoder_sample &sample)
  {
    if (sample.status != encoder_api::encoder_status::ok)
    {
      return false;
    }

    return true;
  }

  std::int32_t unwrap_encoder_delta(std::uint16_t previous_raw, std::uint16_t current_raw)
  {
    std::int32_t delta = static_cast<std::int32_t>(current_raw) - static_cast<std::int32_t>(previous_raw);

    if (delta > 2048)
    {
      delta -= 4096;
    }
    else if (delta < -2048)
    {
      delta += 4096;
    }

    return delta;
  }

  std::int32_t compute_wheel_speed_mm_s(const encoder_api::encoder_sample &previous_sample, const encoder_api::encoder_sample &current_sample)
  {
    const mechanical_config::drivetrain drivetrain = {};
    const std::int32_t delta_counts = unwrap_encoder_delta(previous_sample.angle_raw_12bit, current_sample.angle_raw_12bit);
    const std::uint32_t dt_ms = current_sample.time_ms - previous_sample.time_ms;

    if (dt_ms == 0u)
    {
      return 0;
    }

    return static_cast<std::int32_t>((static_cast<std::int64_t>(delta_counts) * drivetrain.wheel_diameter_mm * 3142) / (4096 * static_cast<std::int64_t>(dt_ms)));
  }

  bool update_wheel_feedback(wheel_drive_controller::state &controller_state, std::uint8_t wheel_id, wheel_drive_controller::wheel_feedback &out, bool &invalid_feedback)
  {
    encoder_api::encoder_sample current_sample = {};
    out = {};
    invalid_feedback = false;

    if (!encoder_api::read_sample(wheel_id, current_sample))
    {
      invalid_feedback = true;
      return false;
    }

    if (!is_encoder_sample_valid(current_sample))
    {
      invalid_feedback = true;
      return false;
    }

    if (!controller_state.has_previous_encoder_sample[wheel_id])
    {
      controller_state.previous_encoder_samples[wheel_id] = current_sample;
      controller_state.has_previous_encoder_sample[wheel_id] = true;
      return false;
    }

    if (current_sample.time_ms <= controller_state.previous_encoder_samples[wheel_id].time_ms)
    {
      controller_state.previous_encoder_samples[wheel_id] = current_sample;
      return false;
    }

    out.speed_mm_s = compute_wheel_speed_mm_s(controller_state.previous_encoder_samples[wheel_id], current_sample);
    out.is_ready = true;
    out.is_valid = true;
    controller_state.previous_encoder_samples[wheel_id] = current_sample;
    return true;
  }

  bool read_all_wheel_feedback(wheel_drive_controller::state &controller_state, std::array<wheel_drive_controller::wheel_feedback, 4u> &out, bool &has_not_ready_feedback, bool &has_invalid_feedback)
  {
    out = {};
    has_not_ready_feedback = false;
    has_invalid_feedback = false;

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      bool invalid_feedback = false;
      const bool wheel_valid = update_wheel_feedback(controller_state, wheel_id, out[wheel_id], invalid_feedback);

      if (invalid_feedback)
      {
        has_invalid_feedback = true;
      }
      else if (!wheel_valid)
      {
        has_not_ready_feedback = true;
      }
    }

    if (has_invalid_feedback)
    {
      return false;
    }

    return true;
  }

  std::int32_t compute_measured_yaw_rate_mdeg_s(wheel_drive_controller::state &controller_state, std::uint32_t now_ms, const local_positioning::snapshot &local_position_snapshot)
  {
    if (!local_position_snapshot.has_pose)
    {
      controller_state.has_previous_local_position_snapshot = false;
      controller_state.previous_local_position_time_ms = 0;
      return 0;
    }

    if (!controller_state.has_previous_local_position_snapshot)
    {
      controller_state.previous_local_position_snapshot = local_position_snapshot;
      controller_state.has_previous_local_position_snapshot = true;
      controller_state.previous_local_position_time_ms = now_ms;
      return 0;
    }

    const local_positioning::snapshot previous_snapshot = controller_state.previous_local_position_snapshot;
    const std::uint32_t previous_time_ms = controller_state.previous_local_position_time_ms;
    controller_state.previous_local_position_snapshot = local_position_snapshot;
    controller_state.previous_local_position_time_ms = now_ms;

    if (!previous_snapshot.has_pose)
    {
      return 0;
    }

    if (local_position_snapshot.pose_id == previous_snapshot.pose_id)
    {
      return 0;
    }

    if (now_ms <= previous_time_ms)
    {
      return 0;
    }

    const std::int32_t delta_heading_urad = local_position_snapshot.heading_urad - previous_snapshot.heading_urad;
    const double delta_time_ms = static_cast<double>(now_ms - previous_time_ms);
    const double measured_yaw_rate_mdeg_s = static_cast<double>(delta_heading_urad) * 180000.0 / (k_pi * 1000000.0 * delta_time_ms);
    return static_cast<std::int32_t>(std::lround(measured_yaw_rate_mdeg_s));
  }

  std::int32_t compute_corrected_yaw_rate_mdeg_s(wheel_drive_controller::state &controller_state, std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command, const local_positioning::snapshot &local_position_snapshot)
  {
    const std::int32_t commanded_yaw_rate_mdeg_s = motion_command.yaw_rate_mdeg_s;

    if (commanded_yaw_rate_mdeg_s == 0)
    {
      return 0;
    }

    if (!local_position_snapshot.has_pose || local_position_snapshot.confidence_heading < wheel_drive_controller_tuning::k_outer_yaw_min_heading_confidence)
    {
      return commanded_yaw_rate_mdeg_s;
    }

    const std::int32_t measured_yaw_rate_mdeg_s = compute_measured_yaw_rate_mdeg_s(controller_state, now_ms, local_position_snapshot);
    const std::int32_t yaw_error_mdeg_s = commanded_yaw_rate_mdeg_s - measured_yaw_rate_mdeg_s;
    const std::int32_t correction_mdeg_s = yaw_error_mdeg_s / wheel_drive_controller_tuning::k_outer_yaw_feedback_per_mdeg_s_divisor;
    const std::int32_t corrected_yaw_rate_mdeg_s = clamp_i32(commanded_yaw_rate_mdeg_s + correction_mdeg_s,
                                                              -wheel_drive_controller_tuning::k_outer_yaw_rate_limit_mdeg_s,
                                                              wheel_drive_controller_tuning::k_outer_yaw_rate_limit_mdeg_s);

    return corrected_yaw_rate_mdeg_s;
  }

  std::int32_t compute_yaw_component_mm_s(std::int32_t yaw_rate_mdeg_s)
  {
    const mechanical_config::drivetrain drivetrain = {};
    const double yaw_rate_rad_s = static_cast<double>(yaw_rate_mdeg_s) * k_pi / 180000.0;
    const double yaw_component_mm_s = yaw_rate_rad_s * static_cast<double>(drivetrain.wheel_separation_mm) * 0.5;
    return static_cast<std::int32_t>(std::lround(yaw_component_mm_s));
  }

  void compute_wheel_targets(const middleware_incoming_payloads::motion_command_payload_data &motion_command, std::int32_t corrected_yaw_rate_mdeg_s, std::array<std::int32_t, 4u> &out)
  {
    const std::int32_t linear_target_mm_s = motion_command.linear_velocity_mm_s;
    const std::int32_t yaw_component_mm_s = compute_yaw_component_mm_s(corrected_yaw_rate_mdeg_s);
    const std::int32_t left_target_mm_s = linear_target_mm_s - yaw_component_mm_s;
    const std::int32_t right_target_mm_s = linear_target_mm_s + yaw_component_mm_s;
    out[0] = left_target_mm_s;
    out[1] = right_target_mm_s;
    out[2] = left_target_mm_s;
    out[3] = right_target_mm_s;
  }

  void reset_wheel_integrator_if_needed(wheel_drive_controller::state &controller_state, std::uint8_t wheel_id, std::int32_t target_speed_mm_s)
  {
    if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
    {
      controller_state.integral_error_mm_s_ms[wheel_id] = 0;
      controller_state.previous_targets_mm_s[wheel_id] = target_speed_mm_s;
      return;
    }

    if (sign_i32(target_speed_mm_s) != sign_i32(controller_state.previous_targets_mm_s[wheel_id]))
    {
      controller_state.integral_error_mm_s_ms[wheel_id] = 0;
      controller_state.previous_targets_mm_s[wheel_id] = target_speed_mm_s;
      return;
    }

    controller_state.previous_targets_mm_s[wheel_id] = target_speed_mm_s;
  }

  std::int16_t compute_open_loop_u(std::int32_t target_speed_mm_s, bool use_rotation_override)
  {
    if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
    {
      return 0;
    }

    std::int32_t u = wheel_drive_controller_tuning::k_feedforward_per_mm_s * target_speed_mm_s;
    std::int32_t startup_drive_u = wheel_drive_controller_tuning::k_startup_drive_u;

    if (use_rotation_override && g_rotation_drive_tuning_override.has_override)
    {
      startup_drive_u = g_rotation_drive_tuning_override.startup_drive_u;
    }

    if (absolute_i32(u) < startup_drive_u)
    {
      u = sign_i32(target_speed_mm_s) * startup_drive_u;
    }

    u = clamp_i32(u, -wheel_drive_controller_tuning::k_max_drive_u, wheel_drive_controller_tuning::k_max_drive_u);
    return static_cast<std::int16_t>(u);
  }

  std::int16_t compute_closed_loop_u(wheel_drive_controller::state &controller_state, std::uint8_t wheel_id, std::uint32_t dt_ms, std::int32_t target_speed_mm_s, std::int32_t measured_speed_mm_s, bool use_rotation_override)
  {
    if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
    {
      controller_state.integral_error_mm_s_ms[wheel_id] = 0;
      return 0;
    }

    const std::int32_t error_mm_s = target_speed_mm_s - measured_speed_mm_s;
    const std::int64_t error_area_mm_s_ms = static_cast<std::int64_t>(error_mm_s) * static_cast<std::int64_t>(dt_ms);
    controller_state.integral_error_mm_s_ms[wheel_id] += error_area_mm_s_ms;
    controller_state.integral_error_mm_s_ms[wheel_id] = clamp_i64(controller_state.integral_error_mm_s_ms[wheel_id], -wheel_drive_controller_tuning::k_integral_limit_mm_s_ms, wheel_drive_controller_tuning::k_integral_limit_mm_s_ms);

    const std::int32_t feedforward_u = wheel_drive_controller_tuning::k_feedforward_per_mm_s * target_speed_mm_s;
    const std::int32_t proportional_u = wheel_drive_controller_tuning::k_feedback_per_mm_s * error_mm_s;
    const std::int32_t integral_u = static_cast<std::int32_t>(controller_state.integral_error_mm_s_ms[wheel_id] / wheel_drive_controller_tuning::k_integral_divisor);
    std::int32_t u = feedforward_u + proportional_u + integral_u;
    std::int32_t min_drive_u = wheel_drive_controller_tuning::k_min_drive_u;

    if (use_rotation_override && g_rotation_drive_tuning_override.has_override)
    {
      min_drive_u = g_rotation_drive_tuning_override.min_drive_u;
    }

    if (absolute_i32(u) < min_drive_u)
    {
      u = sign_i32(target_speed_mm_s) * min_drive_u;
    }

    u = clamp_i32(u, -wheel_drive_controller_tuning::k_max_drive_u, wheel_drive_controller_tuning::k_max_drive_u);
    return static_cast<std::int16_t>(u);
  }

  void reset_integrators(wheel_drive_controller::state &controller_state)
  {
    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      controller_state.integral_error_mm_s_ms[wheel_id] = 0;
    }
  }
}

namespace wheel_drive_controller
{
  /*
  Wheel drive controller used by the drive-control pipeline.
  */

  void init(state &controller_state)
  {
    controller_state = {};
  }

  void tick(state &controller_state, std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command, const local_positioning::snapshot &local_position_snapshot)
  {
    std::array<wheel_feedback, 4u> wheel_feedbacks = {};
    std::array<std::int32_t, 4u> wheel_targets_mm_s = {};
    bool has_not_ready_feedback = false;
    bool has_invalid_feedback = false;
    const bool use_rotation_override = is_rotation_only_command(motion_command);
    const std::int32_t corrected_yaw_rate_mdeg_s = compute_corrected_yaw_rate_mdeg_s(controller_state, now_ms, motion_command, local_position_snapshot);

    if (safe_guard::is_latched())
    {
      reset_integrators(controller_state);
      stop();
      return;
    }

    if (!motion_command.drive_enabled)
    {
      reset_integrators(controller_state);
      stop();
      return;
    }

    if (is_motion_command_stale(now_ms, motion_command))
    {
      reset_integrators(controller_state);
      stop();
      return;
    }

    if (!read_all_wheel_feedback(controller_state, wheel_feedbacks, has_not_ready_feedback, has_invalid_feedback))
    {
      reset_integrators(controller_state);
      stop();
      return;
    }

    if (has_not_ready_feedback && !is_feedback_grace_active(now_ms, motion_command))
    {
      reset_integrators(controller_state);
      stop();
      return;
    }

    compute_wheel_targets(motion_command, corrected_yaw_rate_mdeg_s, wheel_targets_mm_s);

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      reset_wheel_integrator_if_needed(controller_state, wheel_id, wheel_targets_mm_s[wheel_id]);

      if (!wheel_feedbacks[wheel_id].is_valid)
      {
        motor_api::set_u(wheel_id, compute_open_loop_u(wheel_targets_mm_s[wheel_id], use_rotation_override));
        continue;
      }

      const std::uint32_t dt_ms = 1u;
      motor_api::set_u(wheel_id, compute_closed_loop_u(controller_state, wheel_id, dt_ms, wheel_targets_mm_s[wheel_id], wheel_feedbacks[wheel_id].speed_mm_s, use_rotation_override));
    }
  }

  void stop(void)
  {
    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      motor_api::set_u(wheel_id, 0);
    }
  }

  void set_rotation_drive_tuning_override(const rotation_drive_tuning &tuning)
  {
    g_rotation_drive_tuning_override = tuning;
  }

  void clear_rotation_drive_tuning_override(void)
  {
    g_rotation_drive_tuning_override = {};
  }
}
