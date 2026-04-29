#include "core/middleware/outgoing_payloads/debug/voltage_debug_payload.hpp"

#include "core/api/voltage_monitor_api.hpp"
#include "core/middleware/payload_helper_functions.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  constexpr std::uint8_t k_voltage_sensor_id = 0U;
}

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    voltage_monitor_api::voltage_sample payload = {};
    payload_length_out = 0U;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    payload.status = voltage_monitor_api::voltage_status::no_signal;
    voltage_monitor_api::read_sample(k_voltage_sensor_id, payload);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u16(payload.raw_adc))
    {
      return false;
    }

    if (!writer.write_u32(payload.voltage_mv))
    {
      return false;
    }

    if (!writer.write_u32(payload.time_ms))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.status)))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition voltage_debug_payload_definition = {
    "voltage_debug",

    build_payload_bytes
  };
}
