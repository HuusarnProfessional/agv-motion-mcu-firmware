#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/incoming_payloads/incoming_payload_definition.hpp"

namespace stm_esp_incoming_payloads
{
  struct debug_stream_control_payload_data
  {
    std::uint8_t target_payload_id = 0U;
    bool is_enabled = false;
  };

  extern const incoming_payload_definition debug_stream_control_payload_definition;
}
