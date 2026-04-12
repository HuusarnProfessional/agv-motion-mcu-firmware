#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct voltage_debug_payload_data
  {
    std::uint16_t raw_adc = 0U;
    std::uint32_t voltage_mv = 0U;
    std::uint32_t time_ms = 0U;
    std::uint8_t status = 0U;
  };

  extern const outgoing_payload_definition voltage_debug_payload_definition;
}
