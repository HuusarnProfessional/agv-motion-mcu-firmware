#include "core/middleware/incoming_payloads/service/position_correction_payload.hpp"

#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"
#include "core/middleware/payload_helper_functions.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t)
  {
    middleware::binary_packing::reader reader(payload_data, payload_length);
    middleware_incoming_payloads::position_correction_payload_data payload = {};

    if (!reader.read_u16(payload.pose_id))
    {
      return false;
    }

    if (!reader.read_u8(payload.branch_id))
    {
      return false;
    }

    if (!reader.is_finished())
    {
      return false;
    }

    local_positioning::external_correction_request request = {};
    request.has_request = true;
    request.pose_id = payload.pose_id;
    request.branch_id = payload.branch_id;

    (void)state;
    return local_positioning_pipeline::request_position_correction(request);
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition position_correction_payload_definition = {
    "position_correction",

    apply_payload_bytes
  };
}
