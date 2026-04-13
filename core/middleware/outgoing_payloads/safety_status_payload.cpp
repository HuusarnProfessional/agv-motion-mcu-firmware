#include "core/middleware/outgoing_payloads/safety_status_payload.hpp"

#include "core/control/collision_prediction/collision_prediction_pipeline.hpp"
#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    collision_prediction::snapshot payload = {};
    payload_length_out = 0U;

    if (payload_out == nullptr)
    {
      return false;
    }

    collision_prediction::read_snapshot(payload);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.collision_blocked))
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
    0x02U,
    build_payload_bytes
  };
}
