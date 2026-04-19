#include "core/middleware/incoming_payloads/trailer_status_payload.hpp"

#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t)
  {
    middleware::binary_packing::reader reader(payload_data, payload_length);
    bool has_trailer = false;

    if (!reader.read_bool(has_trailer))
    {
      return false;
    }

    if (!reader.is_finished())
    {
      return false;
    }

    state.has_trailer = has_trailer;
    return true;
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition trailer_status_payload_definition = {
    "trailer_status",
    0x25u,
    apply_payload_bytes
  };
}
