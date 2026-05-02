#include "core/middleware/incoming_payloads/service/rotate_delta_payload.hpp"

#include "core/control/motion_primitives/motion_primitives_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"
#include "core/middleware/payload_helper_functions.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    middleware::binary_packing::reader reader(payload_data, payload_length);
    middleware_incoming_payloads::rotate_delta_payload_data payload = {};

    if (!reader.read_i32(payload.linear_velocity_mm_s))
    {
      return false;
    }

    if (!reader.read_i32(payload.yaw_rate_mdeg_s))
    {
      return false;
    }

    if (!reader.read_i64(payload.target_rotation_urad))
    {
      return false;
    }

    if (!reader.read_bool(payload.has_rotation_drive_tuning))
    {
      return false;
    }

    if (!reader.read_i32(payload.rotation_min_drive_u))
    {
      return false;
    }

    if (!reader.read_i32(payload.rotation_startup_drive_u))
    {
      return false;
    }

    if (!reader.is_finished())
    {
      return false;
    }

    motion_primitives::request primitive_request = {};
    primitive_request.primitive_id_value = motion_primitives::primitive_id::rotate_delta;
    primitive_request.rotate_delta.linear_velocity_mm_s = payload.linear_velocity_mm_s;
    primitive_request.rotate_delta.yaw_rate_mdeg_s = payload.yaw_rate_mdeg_s;
    primitive_request.rotate_delta.target_rotation_urad = payload.target_rotation_urad;
    primitive_request.rotate_delta.has_rotation_drive_tuning = payload.has_rotation_drive_tuning;
    primitive_request.rotate_delta.rotation_min_drive_u = payload.rotation_min_drive_u;
    primitive_request.rotate_delta.rotation_startup_drive_u = payload.rotation_startup_drive_u;

    (void)state;
    return motion_primitives_pipeline::request_start(primitive_request, received_time_ms);
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition rotate_delta_payload_definition = {
    "rotate_delta",

    apply_payload_bytes
  };
}
