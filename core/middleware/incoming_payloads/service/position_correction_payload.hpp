#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"

namespace middleware_incoming_payloads
{
  struct position_correction_payload_data
  {
    std::uint8_t pose_id = 0U;
    std::uint8_t branch_id = 0U;
  };

  extern const incoming_payload_definition position_correction_payload_definition;
}
