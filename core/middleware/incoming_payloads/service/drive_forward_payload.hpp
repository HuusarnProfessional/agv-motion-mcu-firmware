#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"

namespace middleware_incoming_payloads
{
  struct drive_forward_payload_data
  {
    std::int32_t velocity_mm_s = 0;
    std::int64_t target_distance_um = 0;
  };

  extern const incoming_payload_definition drive_forward_payload_definition;
}
