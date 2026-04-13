#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/motion_command_payload.hpp"

namespace drive_control
{
  void init(void);
  void set_motion_command(const middleware_incoming_payloads::motion_command_payload_data &motion_command);
  void tick(std::uint32_t now_ms);
}
