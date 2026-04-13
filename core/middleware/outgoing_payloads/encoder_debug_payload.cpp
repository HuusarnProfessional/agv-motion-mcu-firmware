#include "core/middleware/outgoing_payloads/encoder_debug_payload.hpp"

#include "core/api/encoder_api.hpp"
#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  constexpr std::uint8_t k_encoder_count = 4U;

  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u8(k_encoder_count))
    {
      return false;
    }

    for (std::uint8_t encoder_id = 0U; encoder_id < k_encoder_count; ++encoder_id)
    {
      encoder_api::encoder_sample sample = {};
      sample.status = encoder_api::encoder_status::no_signal;
      encoder_api::read_sample(encoder_id, sample);

      if (!writer.write_u16(sample.angle_raw_12bit))
      {
        return false;
      }

      if (!writer.write_u32(sample.angle_mdeg))
      {
        return false;
      }

      if (!writer.write_u32(sample.time_ms))
      {
        return false;
      }

      if (!writer.write_u8(static_cast<std::uint8_t>(sample.status)))
      {
        return false;
      }
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition encoder_debug_payload_definition = {
    "encoder_debug",
    0x11U,
    build_payload_bytes
  };
}
