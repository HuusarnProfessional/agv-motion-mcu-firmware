#include "core/middleware/stm_esp/outgoing_payloads/obstacle_debug_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const stm_esp::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;

    if (!state.has_obstacle_debug || payload_out == nullptr)
    {
      return false;
    }

    const stm_esp_outgoing_payloads::obstacle_debug_payload_data &payload = state.obstacle_debug;
    stm_esp::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u8(payload.sensor_count))
    {
      return false;
    }

    for (std::size_t sensor_index = 0U; sensor_index < payload.sensor_count && sensor_index < payload.sensors.size(); ++sensor_index)
    {
      const stm_esp_outgoing_payloads::obstacle_debug_sample &sample = payload.sensors[sensor_index];

      if (!writer.write_u32(sample.distance_mm))
      {
        return false;
      }

      if (!writer.write_u32(sample.time_ms))
      {
        return false;
      }

      if (!writer.write_u8(sample.status))
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
  const outgoing_payload_definition obstacle_debug_payload_definition = {
    "obstacle_debug",
    0x13U,
    build_payload_bytes
  };
}
