#pragma once

#include <array>
#include <cstdint>

#include "core/api/encoder_api.hpp"
#include "core/control/local_positioning/local_positioning.hpp"
#include "core/middleware/incoming_payloads/runtime/motion_command_payload.hpp"

namespace wheel_drive_controller
{
  struct rotation_drive_tuning
  {
    bool has_override = false;
    std::int32_t min_drive_u = 0;
    std::int32_t startup_drive_u = 0;
  };

  struct motion_debug_snapshot
  {
    bool valid = false;
    bool drive_enabled = false;
    bool motion_session_active = false;
    bool safe_guard_latched = false;
    bool motion_command_stale = false;
    bool has_pose = false;
    bool pose_is_fresh = false;
    bool heading_feedback_active = false;
    bool has_not_ready_feedback = false;
    bool has_invalid_feedback = false;
    std::int32_t commanded_linear_velocity_mm_s = 0;
    std::int32_t commanded_yaw_rate_mdeg_s = 0;
    std::int32_t corrected_yaw_rate_mdeg_s = 0;
    std::int32_t measured_yaw_rate_mdeg_s = 0;
    std::int32_t outer_correction_mdeg_s = 0;
    std::uint16_t pose_confidence_heading = 0u;
    std::uint32_t pose_age_ms = 0u;
    std::array<std::int32_t, 4u> wheel_targets_mm_s = {};
    std::array<std::int32_t, 4u> wheel_speeds_mm_s = {};
    std::array<std::uint32_t, 4u> wheel_sample_ids = {};
    std::array<std::uint32_t, 4u> wheel_sample_age_ms = {};
    std::array<bool, 4u> wheel_has_new_sample = {};
    std::array<bool, 4u> wheel_has_measured_speed = {};
    std::array<std::int16_t, 4u> wheel_drive_u = {};
    std::uint32_t time_ms = 0u;
  };

  struct wheel_runtime
  {
    bool has_encoder_sample = false;
    bool has_measured_speed = false;
    encoder_api::encoder_sample latest_encoder_sample = {};
    std::int32_t measured_speed_mm_s = 0;
    std::uint32_t measured_dt_ms = 0u;
    std::int64_t integral_error_mm_s_ms = 0;
    std::int32_t previous_target_mm_s = 0;
    std::int16_t applied_drive_u = 0;
  };

  struct state
  {
    bool motion_session_active = false;
    std::uint32_t motion_activation_time_ms = 0u;
    std::array<wheel_runtime, 4u> wheels = {};
    bool has_previous_local_position_snapshot = false;
    local_positioning::snapshot previous_local_position_snapshot = {};
    std::int32_t outer_correction_mdeg_s = 0;
    std::int32_t latched_corrected_yaw_rate_mdeg_s = 0;
    std::int32_t latched_measured_yaw_rate_mdeg_s = 0;
    bool heading_feedback_active = false;
    std::array<std::int32_t, 4u> latched_wheel_targets_mm_s = {};
  };

  void init(state &controller_state);
  void tick(state &controller_state, std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command, const local_positioning::snapshot &local_position_snapshot);
  void stop(void);
  void set_rotation_drive_tuning_override(const rotation_drive_tuning &tuning);
  void clear_rotation_drive_tuning_override(void);
  void read_motion_debug_snapshot(motion_debug_snapshot &snapshot_out);
}
