#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/runtime/motion_command_payload.hpp"

namespace drive_control
{
  struct snapshot
  {
    bool has_motion_command = false;
    middleware_incoming_payloads::motion_command_payload_data latest_motion_command = {};
  };

  void init(void);
  void set_motion_command(const middleware_incoming_payloads::motion_command_payload_data &motion_command);
  void tick(std::uint32_t now_ms);
  void read_snapshot(snapshot &out);
  void set_rotation_drive_tuning_override(std::int32_t min_drive_u, std::int32_t startup_drive_u);
  void clear_rotation_drive_tuning_override(void);
}
