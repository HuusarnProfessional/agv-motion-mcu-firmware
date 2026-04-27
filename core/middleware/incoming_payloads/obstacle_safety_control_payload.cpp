#include "core/middleware/incoming_payloads/obstacle_safety_control_payload.hpp"

#include "core/control/collision_prediction/collision_prediction_pipeline.hpp"
#include "core/middleware/binary_packing.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t)
  {
    middleware::binary_packing::reader reader(payload_data, payload_length);
    bool enabled = true;

    if (!reader.read_bool(enabled))
    {
      return false;
    }

    if (!reader.is_finished())
    {
      return false;
    }

    collision_prediction::set_obstacle_safety_enabled(enabled);
    (void)state;
    return true;
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition obstacle_safety_control_payload_definition = {
    "obstacle_safety_control",
    0x28u,
    apply_payload_bytes
  };
}
