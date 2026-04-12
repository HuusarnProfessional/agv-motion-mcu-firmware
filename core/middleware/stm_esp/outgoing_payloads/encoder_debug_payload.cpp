#include "core/middleware/stm_esp/outgoing_payloads/encoder_debug_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const stm_esp::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;

    if (!state.has_encoder_debug || payload_out == nullptr)
    {
      return false;
    }

    const stm_esp_outgoing_payloads::encoder_debug_payload_data &payload = state.encoder_debug;
    stm_esp::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u8(payload.encoder_count))
    {
      return false;
    }

    for (std::size_t encoder_index = 0U; encoder_index < payload.encoder_count && encoder_index < payload.encoders.size(); ++encoder_index)
    {
      const stm_esp_outgoing_payloads::encoder_debug_sample &sample = payload.encoders[encoder_index];

      if (!writer.write_u16(sample.angle_raw_12bit) ||
          !writer.write_u32(sample.angle_mdeg) ||
          !writer.write_u32(sample.time_ms) ||
          !writer.write_u8(sample.status))
      {
        return false;
      }
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace stm_esp_outgoing_payloads
{
  const outgoing_payload_definition encoder_debug_payload_definition = {
    "encoder_debug",
    0x11U,
    build_payload_bytes
  };
}
