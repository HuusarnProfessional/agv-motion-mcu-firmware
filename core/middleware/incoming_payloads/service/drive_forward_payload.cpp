#include "core/middleware/incoming_payloads/service/drive_forward_payload.hpp"

#include "core/control/motion_primitives/motion_primitives_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"
#include "core/middleware/payload_helper_functions.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    middleware::binary_packing::reader reader(payload_data, payload_length);
    middleware_incoming_payloads::drive_forward_payload_data payload = {};

    if (!reader.read_i32(payload.velocity_mm_s))
    {
      return false;
    }

    if (!reader.read_i64(payload.target_distance_um))
    {
      return false;
    }

    if (!reader.is_finished())
    {
      return false;
    }

    motion_primitives::request primitive_request = {};
    primitive_request.primitive_id_value = motion_primitives::primitive_id::drive_forward;
    primitive_request.drive_forward.velocity_mm_s = payload.velocity_mm_s;
    primitive_request.drive_forward.target_distance_um = payload.target_distance_um;

    (void)state;
    return motion_primitives_pipeline::request_start(primitive_request, received_time_ms);
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition drive_forward_payload_definition = {
    "drive_forward",

    apply_payload_bytes
  };
}
