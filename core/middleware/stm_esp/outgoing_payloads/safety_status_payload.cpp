#include "core/middleware/stm_esp/outgoing_payloads/safety_status_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const stm_esp::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;

    if (!state.has_safety_status || payload_out == nullptr)
    {
      return false;
    }

    const stm_esp_outgoing_payloads::safety_status_payload_data &payload = state.safety_status;
    stm_esp::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.collision_blocked))
    {
      return false;
    }

    if (!writer.write_u32(payload.time_ms))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace stm_esp_outgoing_payloads
{
  const outgoing_payload_definition safety_status_payload_definition = {
    "safety_status",
    0x02U,
    build_payload_bytes
  };
}
