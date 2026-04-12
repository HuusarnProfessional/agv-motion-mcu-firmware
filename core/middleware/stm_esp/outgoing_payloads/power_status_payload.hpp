#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct power_status_payload_data
  {
    std::uint32_t voltage_mv = 0U;
    std::uint32_t time_ms = 0U;
    std::uint8_t status = 0U;
  };

  extern const outgoing_payload_definition power_status_payload_definition;
}
