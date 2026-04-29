#include "core/middleware/outgoing_payloads/runtime/safety_status_payload.hpp"

#include "core/control/safe_guard/safe_guard_pipeline.hpp"
#include "core/middleware/payload_helper_functions.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    safe_guard::snapshot payload = {};
    payload_length_out = 0u;

    if (payload_out == nullptr)
    {
      return false;
    }

    safe_guard::read_snapshot(payload);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.fault_latched))
    {
      return false;
    }

    if (!writer.write_u32(state.last_tick_time_ms))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition safety_status_payload_definition = {
    "safety_status",

    build_payload_bytes
  };
}
