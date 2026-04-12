#pragma once

#include <array>
#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct encoder_debug_sample
  {
    std::uint16_t angle_raw_12bit = 0U;
    std::uint32_t angle_mdeg = 0U;
    std::uint32_t time_ms = 0U;
    std::uint8_t status = 0U;
  };

  struct encoder_debug_payload_data
  {
    std::array<encoder_debug_sample, 4U> encoders = {};
    std::uint8_t encoder_count = 4U;
  };

  extern const outgoing_payload_definition encoder_debug_payload_definition;
}
