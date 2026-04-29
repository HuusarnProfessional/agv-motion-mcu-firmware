#include "core/middleware/outgoing_payloads/debug/obstacle_debug_payload.hpp"

#include "core/api/obstacle_api.hpp"
#include "core/middleware/payload_helper_functions.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  constexpr std::uint8_t k_obstacle_sensor_count = 2U;

  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u8(k_obstacle_sensor_count))
    {
      return false;
    }

    for (std::uint8_t sensor_id = 0U; sensor_id < k_obstacle_sensor_count; ++sensor_id)
    {
      obstacle_api::obstacle_sample sample = {};
      sample.status = obstacle_api::obstacle_status::no_signal;
      obstacle_api::read_sample(sensor_id, sample);

      if (!writer.write_u32(sample.distance_mm))
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
  const outgoing_payload_definition obstacle_debug_payload_definition = {
    "obstacle_debug",

    build_payload_bytes
  };
}
