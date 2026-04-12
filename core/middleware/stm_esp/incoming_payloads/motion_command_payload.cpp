#include "core/middleware/stm_esp/incoming_payloads/motion_command_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool apply_payload_bytes(stm_esp::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    stm_esp::binary_packing::reader reader(payload_data, payload_length);
    stm_esp_incoming_payloads::motion_command_payload_data payload = {};

    if (!reader.read_bool(payload.drive_enabled) ||
        !reader.read_i16(payload.linear_velocity_mm_s) ||
        !reader.read_i16(payload.yaw_rate_mdeg_s) ||
        !reader.is_finished())
    {
      return false;
    }

    payload.received_time_ms = received_time_ms;
    state.latest_motion_command = payload;
    state.has_latest_motion_command = true;
    return true;
  }
}

namespace stm_esp_incoming_payloads
{
  const incoming_payload_definition motion_command_payload_definition = {
    "motion_command",
    0x21U,
    apply_payload_bytes
  };
}
