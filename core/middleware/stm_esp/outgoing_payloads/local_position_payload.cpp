#include "core/middleware/stm_esp/outgoing_payloads/local_position_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const stm_esp::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;

    if (!state.has_local_position || payload_out == nullptr)
    {
      return false;
    }

    const stm_esp_outgoing_payloads::local_position_payload_data &payload = state.local_position;
    stm_esp::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.has_pose) ||
        !writer.write_i64(payload.x_um) ||
        !writer.write_i64(payload.y_um) ||
        !writer.write_i32(payload.heading_urad) ||
        !writer.write_u16(payload.confidence_position) ||
        !writer.write_u16(payload.confidence_heading) ||
        !writer.write_u8(payload.pose_id) ||
        !writer.write_u8(payload.branch_id))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace stm_esp_outgoing_payloads
{
  const outgoing_payload_definition local_position_payload_definition = {
    "local_position",
    0x01U,
    build_payload_bytes
  };
}
