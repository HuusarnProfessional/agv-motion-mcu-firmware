#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"

namespace middleware_incoming_payloads
{
  struct rotate_delta_payload_data
  {
    std::int32_t linear_velocity_mm_s = 0;
    std::int32_t yaw_rate_mdeg_s = 0;
    std::int64_t target_rotation_urad = 0;
    bool has_rotation_drive_tuning = false;
    std::int32_t rotation_min_drive_u = 0;
    std::int32_t rotation_startup_drive_u = 0;
  };

  extern const incoming_payload_definition rotate_delta_payload_definition;
}
