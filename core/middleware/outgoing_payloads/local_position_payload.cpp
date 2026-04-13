#include "core/middleware/outgoing_payloads/local_position_payload.hpp"

#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    local_positioning::snapshot payload = {};
    payload_length_out = 0U;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    local_positioning_pipeline::read_snapshot(payload);

    if (!payload.has_pose)
    {
      return false;
    }

    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.has_pose))
    {
      return false;
    }

    if (!writer.write_i64(payload.x_um))
    {
      return false;
    }

    if (!writer.write_i64(payload.y_um))
    {
      return false;
    }

    if (!writer.write_i32(payload.heading_urad))
    {
      return false;
    }

    if (!writer.write_u16(payload.confidence_position))
    {
      return false;
    }

    if (!writer.write_u16(payload.confidence_heading))
    {
      return false;
    }

    if (!writer.write_u8(payload.pose_id))
    {
      return false;
    }

    if (!writer.write_u8(payload.branch_id))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition local_position_payload_definition = {
    "local_position",
    0x01U,
    build_payload_bytes
  };
}
