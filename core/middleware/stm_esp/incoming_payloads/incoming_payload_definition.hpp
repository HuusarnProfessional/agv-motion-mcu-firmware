#pragma once

#include <cstddef>
#include <cstdint>

namespace stm_esp
{
  struct middleware_state;
}

namespace stm_esp_incoming_payloads
{
  using apply_payload_bytes_fn = bool (*)(stm_esp::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms);

  struct incoming_payload_definition
  {
    const char *name;
    std::uint8_t payload_id;
    apply_payload_bytes_fn apply_payload_bytes;
  };
}
