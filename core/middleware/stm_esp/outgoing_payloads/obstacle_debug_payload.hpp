#pragma once

#include <array>
#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct obstacle_debug_sample
  {
    std::uint32_t distance_mm = 0U;
    std::uint32_t time_ms = 0U;
    std::uint8_t status = 0U;
  };

  struct obstacle_debug_payload_data
  {
    std::array<obstacle_debug_sample, 2U> sensors = {};
    std::uint8_t sensor_count = 2U;
  };

  extern const outgoing_payload_definition obstacle_debug_payload_definition;
}
