#include "core/middleware/outgoing_payloads/runtime/motion_primitive_status_payload.hpp"

#include "core/control/motion_primitives/motion_primitives_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"
#include "core/middleware/payload_helper_functions.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    motion_primitives::snapshot payload = {};
    payload_length_out = 0u;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    motion_primitives_pipeline::read_snapshot(payload);

    if (payload.command_id == 0u)
    {
      return false;
    }

    if (!payload.running && !payload.complete)
    {
      return false;
    }

    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_u32(payload.command_id))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.active_primitive_id)))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.state)))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.failure_code)))
    {
      return false;
    }

    if (!writer.write_u8(0u))
    {
      return false;
    }

    if (!writer.write_u32(payload.status_time_ms))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition motion_primitive_status_payload_definition = {
    "motion_primitive_status",

    build_payload_bytes
  };
}
