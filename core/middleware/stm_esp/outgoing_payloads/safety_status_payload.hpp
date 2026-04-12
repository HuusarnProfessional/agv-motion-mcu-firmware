#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct safety_status_payload_data
  {
    bool collision_blocked = false;
    std::uint32_t time_ms = 0U;
  };

  extern const outgoing_payload_definition safety_status_payload_definition;
}
