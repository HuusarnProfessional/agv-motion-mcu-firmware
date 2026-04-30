#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"

namespace middleware_incoming_payloads
{
  struct motion_command_payload_data
  {
    bool drive_enabled = false;
    std::int32_t linear_velocity_mm_s = 0;
    std::int32_t yaw_rate_mdeg_s = 0;
    std::uint32_t received_time_ms = 0U;
  };

  extern const incoming_payload_definition motion_command_payload_definition;
}
