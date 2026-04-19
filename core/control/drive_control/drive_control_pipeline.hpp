#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/motion_command_payload.hpp"

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
}
