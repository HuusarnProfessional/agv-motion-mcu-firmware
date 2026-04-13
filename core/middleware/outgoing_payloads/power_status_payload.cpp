#include "core/middleware/outgoing_payloads/power_status_payload.hpp"

#include "core/api/voltage_monitor_api.hpp"
#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  constexpr std::uint8_t k_voltage_sensor_id = 0U;
}

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    voltage_monitor_api::voltage_sample sample = {};
    payload_length_out = 0U;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    sample.status = voltage_monitor_api::voltage_status::no_signal;
    voltage_monitor_api::read_sample(k_voltage_sensor_id, sample);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u32(sample.voltage_mv))
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

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition power_status_payload_definition = {
    "power_status",
    0x03U,
    build_payload_bytes
  };
}
