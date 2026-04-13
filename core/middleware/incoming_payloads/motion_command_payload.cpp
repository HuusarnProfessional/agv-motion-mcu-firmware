#include "core/middleware/incoming_payloads/motion_command_payload.hpp"

#include "core/control/drive_control/drive_control_pipeline.hpp"
#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    middleware::binary_packing::reader reader(payload_data, payload_length);
    middleware_incoming_payloads::motion_command_payload_data payload = {};

    if (!reader.read_bool(payload.drive_enabled))
    {
      return false;
    }

    if (!reader.read_i16(payload.linear_velocity_mm_s))
    {
      return false;
    }

    if (!reader.read_i16(payload.yaw_rate_mdeg_s))
    {
      return false;
    }

    if (!reader.is_finished())
    {
      return false;
    }

    payload.received_time_ms = received_time_ms;
    drive_control::set_motion_command(payload);
    (void)state;
    return true;
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition motion_command_payload_definition = {
    "motion_command",
    0x21U,
    apply_payload_bytes
  };
}
