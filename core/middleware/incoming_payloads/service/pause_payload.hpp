#pragma once

#include <cstdint>

#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"

namespace middleware_incoming_payloads
{
  struct pause_payload_data
  {
    std::uint32_t duration_ms = 0u;
  };

  extern const incoming_payload_definition pause_payload_definition;
}
