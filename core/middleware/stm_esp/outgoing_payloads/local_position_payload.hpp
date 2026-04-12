#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct local_position_payload_data
  {
    bool has_pose = false;
    std::int64_t x_um = 0;
    std::int64_t y_um = 0;
    std::int32_t heading_urad = 0;
    std::uint16_t confidence_position = 0;
    std::uint16_t confidence_heading = 0;
    std::uint8_t pose_id = 0;
    std::uint8_t branch_id = 0;
  };

  extern const outgoing_payload_definition local_position_payload_definition;
}
