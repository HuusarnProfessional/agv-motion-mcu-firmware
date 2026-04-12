#include "core/middleware/stm_esp/incoming_payloads/debug_stream_control_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool apply_payload_bytes(stm_esp::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t)
  {
    stm_esp::binary_packing::reader reader(payload_data, payload_length);
    stm_esp_incoming_payloads::debug_stream_control_payload_data payload = {};

    if (!reader.read_u8(payload.target_payload_id) ||
        !reader.read_bool(payload.is_enabled) ||
        !reader.is_finished())
    {
      return false;
    }

    state.pending_debug_stream_control = payload;
    state.has_pending_debug_stream_control = true;
    return true;
  }
}

namespace stm_esp_incoming_payloads
{
  const incoming_payload_definition debug_stream_control_payload_definition = {
    "debug_stream_control",
    0x24U,
    apply_payload_bytes
  };
}
