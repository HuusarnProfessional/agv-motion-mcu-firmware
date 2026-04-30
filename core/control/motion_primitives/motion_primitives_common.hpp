#pragma once

#include <cstdint>

#include "core/control/motion_primitives/motion_primitives.hpp"
#include "core/middleware/incoming_payloads/runtime/motion_command_payload.hpp"

namespace motion_primitives_common
{
  enum class phase : std::uint8_t
  {
    idle = 0u,
    active,
    settling,
    done
  };

  enum class tick_result : std::uint8_t
  {
    keep_running = 0u,
    begin_settling,
    complete_success,
    complete_failure,
    complete_timeout
  };

  struct state
  {
    motion_primitives::snapshot snapshot = {};
    motion_primitives::request active_request = {};
    phase active_phase = phase::idle;
  };

  bool has_elapsed(std::uint32_t now_ms, std::uint32_t target_ms);
  std::int64_t abs_i64(std::int64_t value);
  void send_motion_command(bool drive_enabled, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s, std::uint32_t now_ms);
  void stop_motion(std::uint32_t now_ms);
  std::int64_t squared_distance_um(const local_positioning::snapshot &start_pose, const local_positioning::snapshot &current_pose);
  bool has_reached_forward_target(const local_positioning::snapshot &start_pose, const local_positioning::snapshot &current_pose, std::int64_t target_distance_um);
  bool has_reached_rotation_target(const local_positioning::snapshot &start_pose, const local_positioning::snapshot &current_pose, std::int64_t target_rotation_urad);
  bool is_settled(const encoder_motion::state &encoder_state, const local_positioning_imu::state &imu_state);
  void begin_settling(state &primitive_state, const local_positioning::snapshot &current_pose, std::uint32_t now_ms);
  void finish(state &primitive_state, bool success, bool timed_out, std::uint32_t now_ms);
  void begin_active_phase(state &primitive_state, const local_positioning::snapshot &current_pose);
}
