#pragma once

#include <cstddef>
#include <cstdint>

namespace stm_esp
{
  struct middleware_state;
}

namespace stm_esp_outgoing_payloads
{
  using build_payload_bytes_fn = bool (*)(const stm_esp::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out);

  struct outgoing_payload_definition
  {
    const char *name;
    std::uint8_t payload_id;
    build_payload_bytes_fn build_payload_bytes;
  };
}
