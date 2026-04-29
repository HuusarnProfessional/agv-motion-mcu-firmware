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

  struct wheel_feedback
  {
    bool is_ready = false;
    bool is_valid = false;
    std::int32_t speed_mm_s = 0;
  };

  struct state
  {
    std::array<bool, 4u> has_previous_encoder_sample = {};
    std::array<encoder_api::encoder_sample, 4u> previous_encoder_samples = {};
    std::array<std::int64_t, 4u> integral_error_mm_s_ms = {};
    std::array<std::int32_t, 4u> previous_targets_mm_s = {};
    bool has_previous_local_position_snapshot = false;
    local_positioning::snapshot previous_local_position_snapshot = {};
    std::uint32_t previous_local_position_time_ms = 0;
  };

  void init(state &controller_state);
  void tick(state &controller_state, std::uint32_t now_ms, const middleware_incoming_payloads::motion_command_payload_data &motion_command, const local_positioning::snapshot &local_position_snapshot);
  void stop(void);
  void set_rotation_drive_tuning_override(const rotation_drive_tuning &tuning);
  void clear_rotation_drive_tuning_override(void);
}
