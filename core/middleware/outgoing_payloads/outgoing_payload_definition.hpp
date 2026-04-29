#pragma once

#include <cstddef>
#include <cstdint>

namespace middleware
{
  struct middleware_state;
}

namespace middleware_outgoing_payloads
{
  using build_payload_bytes_fn = bool (*)(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out);

  struct outgoing_payload_definition
  {
    const char *name;
    build_payload_bytes_fn build_payload_bytes;
  };
}
