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
  wheel_drive_controller::motion_debug_snapshot g_motion_debug_snapshot = {};

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

  bool is_encoder_sample_valid(const encoder_api::encoder_sample &sample)
  {
    if (sample.status != encoder_api::encoder_status::ok)
    {
      return false;
    }

    return true;
  }

  std::uint32_t compute_delta_time_ms(std::uint32_t previous_time_ms, std::uint32_t current_time_ms)
  {
    if (current_time_ms <= previous_time_ms)
    {
      return 0u;
    }

    return current_time_ms - previous_time_ms;
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
    const std::uint32_t dt_ms = compute_delta_time_ms(previous_sample.time_ms, current_sample.time_ms);

    if (dt_ms == 0u)
    {
      return 0;
    }

    return static_cast<std::int32_t>((static_cast<std::int64_t>(delta_counts) * drivetrain.wheel_diameter_mm * 3142) / (4096 * static_cast<std::int64_t>(dt_ms)));
  }

  bool is_wheel_feedback_fresh(std::uint32_t now_ms, const wheel_drive_controller::wheel_runtime &wheel_state)
  {
    if (!wheel_state.has_encoder_sample)
    {
      return false;
    }

    if (now_ms < wheel_state.latest_encoder_sample.time_ms)
    {
      return false;
    }

    if (now_ms - wheel_state.latest_encoder_sample.time_ms > wheel_drive_controller_tuning::k_encoder_feedback_stale_timeout_ms)
    {
      return false;
    }

    return true;
  }

  std::uint32_t compute_wheel_sample_age_ms(std::uint32_t now_ms, const wheel_drive_controller::wheel_runtime &wheel_state)
  {
    if (!wheel_state.has_encoder_sample)
    {
      return 0u;
    }

    if (now_ms <= wheel_state.latest_encoder_sample.time_ms)
    {
      return 0u;
    }

    return now_ms - wheel_state.latest_encoder_sample.time_ms;
  }

  std::uint32_t compute_pose_age_ms(std::uint32_t now_ms, const wheel_drive_controller::state &controller_state)
  {
    if (!controller_state.has_previous_local_position_snapshot)
    {
      return 0u;
    }

    const local_positioning::snapshot &snapshot = controller_state.previous_local_position_snapshot;
    if (now_ms <= snapshot.time_ms)
    {
      return 0u;
    }

    return now_ms - snapshot.time_ms;
  }

  bool is_pose_feedback_fresh(std::uint32_t now_ms, const wheel_drive_controller::state &controller_state)
  {
    if (!controller_state.has_previous_local_position_snapshot)
    {
      return false;
    }

    const local_positioning::snapshot &snapshot = controller_state.previous_local_position_snapshot;
    if (snapshot.time_ms > now_ms)
    {
      return false;
    }

    if (now_ms - snapshot.time_ms > wheel_drive_controller_tuning::k_pose_feedback_stale_timeout_ms)
    {
      return false;
    }

    return true;
  }

  bool update_wheel_runtime_from_encoder(wheel_drive_controller::wheel_runtime &wheel_state, std::uint8_t wheel_id, bool &has_invalid_feedback)
  {
    encoder_api::encoder_sample current_sample = {};
    has_invalid_feedback = false;

    if (!encoder_api::read_sample(wheel_id, current_sample))
    {
      has_invalid_feedback = true;
      return false;
    }

    if (!is_encoder_sample_valid(current_sample))
    {
      has_invalid_feedback = true;
      return false;
    }

    if (!wheel_state.has_encoder_sample)
    {
      wheel_state.latest_encoder_sample = current_sample;
      wheel_state.has_encoder_sample = true;
      return false;
    }

    if (current_sample.sample_id == wheel_state.latest_encoder_sample.sample_id)
    {
      return false;
    }

    const std::uint32_t dt_ms = compute_delta_time_ms(wheel_state.latest_encoder_sample.time_ms, current_sample.time_ms);
    if (dt_ms == 0u)
    {
      wheel_state.latest_encoder_sample = current_sample;
      return false;
    }

    wheel_state.measured_speed_mm_s = compute_wheel_speed_mm_s(wheel_state.latest_encoder_sample, current_sample);
    wheel_state.measured_dt_ms = dt_ms;
    wheel_state.has_measured_speed = true;
    wheel_state.latest_encoder_sample = current_sample;
    return true;
  }

  bool all_wheels_have_measured_speed(const wheel_drive_controller::state &controller_state)
  {
    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      if (!controller_state.wheels[wheel_id].has_measured_speed)
      {
        return false;
      }
    }

    return true;
  }

  bool all_wheels_are_fresh(std::uint32_t now_ms, const wheel_drive_controller::state &controller_state)
  {
    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      if (!is_wheel_feedback_fresh(now_ms, controller_state.wheels[wheel_id]))
      {
        return false;
      }
    }

    return true;
  }

  void reset_wheel_integrator_if_needed(wheel_drive_controller::wheel_runtime &wheel_state, std::int32_t target_speed_mm_s)
  {
    if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
    {
      wheel_state.integral_error_mm_s_ms = 0;
      wheel_state.previous_target_mm_s = target_speed_mm_s;
      return;
    }

    if (sign_i32(target_speed_mm_s) != sign_i32(wheel_state.previous_target_mm_s))
    {
      wheel_state.integral_error_mm_s_ms = 0;
      wheel_state.previous_target_mm_s = target_speed_mm_s;
      return;
    }

    wheel_state.previous_target_mm_s = target_speed_mm_s;
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

  void compute_open_loop_outputs(const std::array<std::int32_t, 4u> &wheel_targets_mm_s, bool use_rotation_override, std::array<std::int16_t, 4u> &out)
  {
    out = {};

    if (use_rotation_override)
    {
      for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
      {
        out[wheel_id] = compute_open_loop_u(wheel_targets_mm_s[wheel_id], true);
      }
      return;
    }

    std::int32_t startup_drive_u = wheel_drive_controller_tuning::k_startup_drive_u;
    std::int32_t max_feedforward_u = 0;

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      const std::int32_t target_speed_mm_s = wheel_targets_mm_s[wheel_id];
      if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
      {
        continue;
      }

      const std::int32_t feedforward_u = wheel_drive_controller_tuning::k_feedforward_per_mm_s * absolute_i32(target_speed_mm_s);
      if (feedforward_u > max_feedforward_u)
      {
        max_feedforward_u = feedforward_u;
      }
    }

    if (max_feedforward_u == 0)
    {
      return;
    }

    std::int32_t scale_numerator = max_feedforward_u;
    std::int32_t scale_denominator = max_feedforward_u;
    if (max_feedforward_u < startup_drive_u)
    {
      scale_numerator = startup_drive_u;
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      const std::int32_t target_speed_mm_s = wheel_targets_mm_s[wheel_id];
      if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
      {
        out[wheel_id] = 0;
        continue;
      }

      std::int32_t u = wheel_drive_controller_tuning::k_feedforward_per_mm_s * target_speed_mm_s;
      u = static_cast<std::int32_t>((static_cast<std::int64_t>(u) * static_cast<std::int64_t>(scale_numerator)) / static_cast<std::int64_t>(scale_denominator));
      u = clamp_i32(u, -wheel_drive_controller_tuning::k_max_drive_u, wheel_drive_controller_tuning::k_max_drive_u);
      out[wheel_id] = static_cast<std::int16_t>(u);
    }
  }

  std::int16_t compute_closed_loop_u(wheel_drive_controller::wheel_runtime &wheel_state, std::int32_t target_speed_mm_s, bool use_rotation_override)
  {
    if (absolute_i32(target_speed_mm_s) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
    {
      wheel_state.integral_error_mm_s_ms = 0;
      return 0;
    }

    const std::int32_t error_mm_s = target_speed_mm_s - wheel_state.measured_speed_mm_s;
    const std::int64_t error_area_mm_s_ms = static_cast<std::int64_t>(error_mm_s) * static_cast<std::int64_t>(wheel_state.measured_dt_ms);
    wheel_state.integral_error_mm_s_ms += error_area_mm_s_ms;
    wheel_state.integral_error_mm_s_ms = clamp_i64(wheel_state.integral_error_mm_s_ms, -wheel_drive_controller_tuning::k_integral_limit_mm_s_ms, wheel_drive_controller_tuning::k_integral_limit_mm_s_ms);

    const std::int32_t feedforward_u = wheel_drive_controller_tuning::k_feedforward_per_mm_s * target_speed_mm_s;
    const std::int32_t proportional_u = wheel_drive_controller_tuning::k_feedback_per_mm_s * error_mm_s;
    const std::int32_t integral_u = static_cast<std::int32_t>(wheel_state.integral_error_mm_s_ms / wheel_drive_controller_tuning::k_integral_divisor);
    std::int32_t u = feedforward_u + proportional_u + integral_u;
    std::int32_t min_drive_u = wheel_drive_controller_tuning::k_min_drive_u;

    if (use_rotation_override && g_rotation_drive_tuning_override.has_override)
    {
      min_drive_u = g_rotation_drive_tuning_override.min_drive_u;
    }

    const bool output_pushes_toward_target = sign_i32(u) == sign_i32(target_speed_mm_s);
    if (output_pushes_toward_target && absolute_i32(u) < min_drive_u)
    {
      u = sign_i32(target_speed_mm_s) * min_drive_u;
    }

    u = clamp_i32(u, -wheel_drive_controller_tuning::k_max_drive_u, wheel_drive_controller_tuning::k_max_drive_u);
    return static_cast<std::int16_t>(u);
  }

  void clear_closed_loop_state(wheel_drive_controller::state &controller_state)
  {
    controller_state.outer_correction_mdeg_s = 0;
    controller_state.latched_corrected_yaw_rate_mdeg_s = 0;
    controller_state.latched_measured_yaw_rate_mdeg_s = 0;
    controller_state.heading_feedback_active = false;
    controller_state.latched_wheel_targets_mm_s = {};

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      controller_state.wheels[wheel_id].has_measured_speed = false;
      controller_state.wheels[wheel_id].measured_speed_mm_s = 0;
      controller_state.wheels[wheel_id].measured_dt_ms = 0u;
      controller_state.wheels[wheel_id].integral_error_mm_s_ms = 0;
      controller_state.wheels[wheel_id].previous_target_mm_s = 0;
      controller_state.wheels[wheel_id].applied_drive_u = 0;
    }
  }

  void start_motion_session_if_needed(wheel_drive_controller::state &controller_state, std::uint32_t now_ms)
  {
    if (controller_state.motion_session_active)
    {
      return;
    }

    controller_state.motion_session_active = true;
    controller_state.motion_activation_time_ms = now_ms;
    clear_closed_loop_state(controller_state);
  }

  void stop_motion_session(wheel_drive_controller::state &controller_state)
  {
    controller_state.motion_session_active = false;
    controller_state.motion_activation_time_ms = 0u;
    clear_closed_loop_state(controller_state);
  }

  std::int32_t compute_measured_yaw_rate_mdeg_s(const local_positioning::snapshot &previous_snapshot, const local_positioning::snapshot &current_snapshot)
  {
    const std::uint32_t dt_ms = compute_delta_time_ms(previous_snapshot.time_ms, current_snapshot.time_ms);
    if (dt_ms == 0u)
    {
      return 0;
    }

    const std::int32_t delta_heading_urad = current_snapshot.heading_urad - previous_snapshot.heading_urad;
    const double delta_time_ms = static_cast<double>(dt_ms);
    const double measured_yaw_rate_mdeg_s = static_cast<double>(delta_heading_urad) * 180000.0 / (k_pi * 1000000.0 * delta_time_ms);
    return static_cast<std::int32_t>(std::lround(measured_yaw_rate_mdeg_s));
  }

  void update_outer_loop_state(wheel_drive_controller::state &controller_state, std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command, const local_positioning::snapshot &local_position_snapshot)
  {
    if (local_position_snapshot.has_pose)
    {
      if (!controller_state.has_previous_local_position_snapshot)
      {
        controller_state.previous_local_position_snapshot = local_position_snapshot;
        controller_state.has_previous_local_position_snapshot = true;
      }
      else if (local_position_snapshot.update_id != controller_state.previous_local_position_snapshot.update_id)
      {
        if (local_position_snapshot.branch_id != controller_state.previous_local_position_snapshot.branch_id)
        {
          controller_state.previous_local_position_snapshot = local_position_snapshot;
          controller_state.latched_measured_yaw_rate_mdeg_s = 0;
          controller_state.outer_correction_mdeg_s = 0;
          controller_state.heading_feedback_active = false;
          controller_state.latched_corrected_yaw_rate_mdeg_s = motion_command.yaw_rate_mdeg_s;
          return;
        }

        controller_state.latched_measured_yaw_rate_mdeg_s = compute_measured_yaw_rate_mdeg_s(controller_state.previous_local_position_snapshot, local_position_snapshot);
        controller_state.previous_local_position_snapshot = local_position_snapshot;

        if (motion_command.yaw_rate_mdeg_s != 0 && local_position_snapshot.confidence_heading >= wheel_drive_controller_tuning::k_outer_yaw_min_heading_confidence)
        {
          const std::int32_t yaw_error_mdeg_s = motion_command.yaw_rate_mdeg_s - controller_state.latched_measured_yaw_rate_mdeg_s;
          controller_state.outer_correction_mdeg_s = yaw_error_mdeg_s / wheel_drive_controller_tuning::k_outer_yaw_feedback_per_mdeg_s_divisor;
        }
        else
        {
          controller_state.outer_correction_mdeg_s = 0;
        }
      }
    }

    if (!controller_state.has_previous_local_position_snapshot)
    {
      controller_state.heading_feedback_active = false;
      controller_state.outer_correction_mdeg_s = 0;
      controller_state.latched_corrected_yaw_rate_mdeg_s = motion_command.yaw_rate_mdeg_s;
      return;
    }

    const local_positioning::snapshot &active_pose_snapshot = controller_state.previous_local_position_snapshot;
    const bool pose_is_fresh = is_pose_feedback_fresh(now_ms, controller_state);
    if (!pose_is_fresh)
    {
      controller_state.heading_feedback_active = false;
      controller_state.outer_correction_mdeg_s = 0;
      controller_state.latched_corrected_yaw_rate_mdeg_s = motion_command.yaw_rate_mdeg_s;
      return;
    }

    if (motion_command.yaw_rate_mdeg_s == 0 || active_pose_snapshot.confidence_heading < wheel_drive_controller_tuning::k_outer_yaw_min_heading_confidence)
    {
      controller_state.heading_feedback_active = false;
      controller_state.outer_correction_mdeg_s = 0;
      controller_state.latched_corrected_yaw_rate_mdeg_s = motion_command.yaw_rate_mdeg_s;
      return;
    }

    controller_state.heading_feedback_active = true;
    controller_state.latched_corrected_yaw_rate_mdeg_s = clamp_i32(motion_command.yaw_rate_mdeg_s + controller_state.outer_correction_mdeg_s,
                                                                    -wheel_drive_controller_tuning::k_outer_yaw_rate_limit_mdeg_s,
                                                                    wheel_drive_controller_tuning::k_outer_yaw_rate_limit_mdeg_s);
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

  void apply_motor_u(std::uint8_t wheel_id, wheel_drive_controller::wheel_runtime &wheel_state, std::int16_t drive_u)
  {
    wheel_state.applied_drive_u = drive_u;
    motor_api::set_u(wheel_id, drive_u);
  }

  void apply_open_loop_to_all_wheels(wheel_drive_controller::state &controller_state, const std::array<std::int32_t, 4u> &wheel_targets_mm_s, bool use_rotation_override)
  {
    std::array<std::int16_t, 4u> open_loop_outputs = {};
    compute_open_loop_outputs(wheel_targets_mm_s, use_rotation_override, open_loop_outputs);

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      reset_wheel_integrator_if_needed(controller_state.wheels[wheel_id], wheel_targets_mm_s[wheel_id]);
      apply_motor_u(wheel_id, controller_state.wheels[wheel_id], open_loop_outputs[wheel_id]);
    }
  }

  void hold_or_update_closed_loop_outputs(wheel_drive_controller::state &controller_state, const std::array<std::int32_t, 4u> &wheel_targets_mm_s, const std::array<bool, 4u> &wheel_has_new_sample, bool use_rotation_override)
  {
    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      wheel_drive_controller::wheel_runtime &wheel_state = controller_state.wheels[wheel_id];
      reset_wheel_integrator_if_needed(wheel_state, wheel_targets_mm_s[wheel_id]);

      if (absolute_i32(wheel_targets_mm_s[wheel_id]) <= wheel_drive_controller_tuning::k_speed_deadband_mm_s)
      {
        apply_motor_u(wheel_id, wheel_state, 0);
        continue;
      }

      if (wheel_has_new_sample[wheel_id])
      {
        const std::int16_t closed_loop_u = compute_closed_loop_u(wheel_state, wheel_targets_mm_s[wheel_id], use_rotation_override);
        apply_motor_u(wheel_id, wheel_state, closed_loop_u);
        continue;
      }

      motor_api::set_u(wheel_id, wheel_state.applied_drive_u);
    }
  }

  void stop_with_reset(wheel_drive_controller::state &controller_state)
  {
    stop_motion_session(controller_state);
    wheel_drive_controller::stop();
  }
}

namespace wheel_drive_controller
{
  void init(state &controller_state)
  {
    controller_state = {};
  }

  void tick(state &controller_state, std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command, const local_positioning::snapshot &local_position_snapshot)
  {
    std::array<bool, 4u> wheel_has_new_sample = {};
    bool has_not_ready_feedback = false;
    bool has_invalid_feedback = false;
    const bool use_rotation_override = is_rotation_only_command(motion_command);
    g_motion_debug_snapshot = {};
    g_motion_debug_snapshot.valid = true;
    g_motion_debug_snapshot.drive_enabled = motion_command.drive_enabled;
    g_motion_debug_snapshot.motion_session_active = controller_state.motion_session_active;
    g_motion_debug_snapshot.safe_guard_latched = safe_guard::is_latched();
    g_motion_debug_snapshot.motion_command_stale = is_motion_command_stale(now_ms, motion_command);
    g_motion_debug_snapshot.has_pose = local_position_snapshot.has_pose;
    g_motion_debug_snapshot.commanded_linear_velocity_mm_s = motion_command.linear_velocity_mm_s;
    g_motion_debug_snapshot.commanded_yaw_rate_mdeg_s = motion_command.yaw_rate_mdeg_s;
    g_motion_debug_snapshot.time_ms = now_ms;

    if (safe_guard::is_latched())
    {
      stop_with_reset(controller_state);
      return;
    }

    if (!motion_command.drive_enabled)
    {
      stop_with_reset(controller_state);
      return;
    }

    if (is_motion_command_stale(now_ms, motion_command))
    {
      stop_with_reset(controller_state);
      return;
    }

    start_motion_session_if_needed(controller_state, now_ms);

    update_outer_loop_state(controller_state, now_ms, motion_command, local_position_snapshot);
    compute_wheel_targets(motion_command, controller_state.latched_corrected_yaw_rate_mdeg_s, controller_state.latched_wheel_targets_mm_s);

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      bool wheel_invalid_feedback = false;
      wheel_has_new_sample[wheel_id] = update_wheel_runtime_from_encoder(controller_state.wheels[wheel_id], wheel_id, wheel_invalid_feedback);

      if (wheel_invalid_feedback)
      {
        has_invalid_feedback = true;
      }

      g_motion_debug_snapshot.wheel_targets_mm_s[wheel_id] = controller_state.latched_wheel_targets_mm_s[wheel_id];
      g_motion_debug_snapshot.wheel_speeds_mm_s[wheel_id] = controller_state.wheels[wheel_id].measured_speed_mm_s;
      g_motion_debug_snapshot.wheel_sample_ids[wheel_id] = controller_state.wheels[wheel_id].latest_encoder_sample.sample_id;
      g_motion_debug_snapshot.wheel_sample_age_ms[wheel_id] = compute_wheel_sample_age_ms(now_ms, controller_state.wheels[wheel_id]);
      g_motion_debug_snapshot.wheel_has_new_sample[wheel_id] = wheel_has_new_sample[wheel_id];
      g_motion_debug_snapshot.wheel_has_measured_speed[wheel_id] = controller_state.wheels[wheel_id].has_measured_speed;
    }

    g_motion_debug_snapshot.motion_session_active = controller_state.motion_session_active;
    g_motion_debug_snapshot.pose_is_fresh = is_pose_feedback_fresh(now_ms, controller_state);
    g_motion_debug_snapshot.pose_confidence_heading = controller_state.has_previous_local_position_snapshot ? controller_state.previous_local_position_snapshot.confidence_heading : 0u;
    g_motion_debug_snapshot.pose_age_ms = compute_pose_age_ms(now_ms, controller_state);
    g_motion_debug_snapshot.heading_feedback_active = controller_state.heading_feedback_active;
    g_motion_debug_snapshot.corrected_yaw_rate_mdeg_s = controller_state.latched_corrected_yaw_rate_mdeg_s;
    g_motion_debug_snapshot.measured_yaw_rate_mdeg_s = controller_state.latched_measured_yaw_rate_mdeg_s;
    g_motion_debug_snapshot.outer_correction_mdeg_s = controller_state.outer_correction_mdeg_s;
    g_motion_debug_snapshot.has_invalid_feedback = has_invalid_feedback;

    if (has_invalid_feedback)
    {
      stop_with_reset(controller_state);
      return;
    }

    if (!all_wheels_have_measured_speed(controller_state))
    {
      has_not_ready_feedback = true;
      g_motion_debug_snapshot.has_not_ready_feedback = true;
      apply_open_loop_to_all_wheels(controller_state, controller_state.latched_wheel_targets_mm_s, use_rotation_override);

      for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
      {
        g_motion_debug_snapshot.wheel_drive_u[wheel_id] = controller_state.wheels[wheel_id].applied_drive_u;
      }
      return;
    }

    if (!all_wheels_are_fresh(now_ms, controller_state))
    {
      has_not_ready_feedback = true;
      g_motion_debug_snapshot.has_not_ready_feedback = true;

      apply_open_loop_to_all_wheels(controller_state, controller_state.latched_wheel_targets_mm_s, use_rotation_override);

      for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
      {
        g_motion_debug_snapshot.wheel_drive_u[wheel_id] = controller_state.wheels[wheel_id].applied_drive_u;
      }
      return;
    }

    hold_or_update_closed_loop_outputs(controller_state, controller_state.latched_wheel_targets_mm_s, wheel_has_new_sample, use_rotation_override);

    for (std::uint8_t wheel_id = 0u; wheel_id < wheel_drive_controller_tuning::k_wheel_count; ++wheel_id)
    {
      g_motion_debug_snapshot.wheel_drive_u[wheel_id] = controller_state.wheels[wheel_id].applied_drive_u;
    }

    g_motion_debug_snapshot.has_not_ready_feedback = has_not_ready_feedback;
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

  void read_motion_debug_snapshot(motion_debug_snapshot &snapshot_out)
  {
    snapshot_out = g_motion_debug_snapshot;
  }
}
