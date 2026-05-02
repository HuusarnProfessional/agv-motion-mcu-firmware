#include "core/middleware/outgoing_payloads/debug/local_position_model_debug_payload.hpp"

#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/control/local_positioning/test/local_position_model_debug.hpp"
#include "core/middleware/middleware_runtime.hpp"
#include "core/middleware/payload_helper_functions.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    local_position_model_debug::snapshot payload = {};
    payload_length_out = 0u;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    local_positioning_pipeline::read_model_debug_snapshot(payload);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.has_encoder_motion))
    {
      return false;
    }
    if (!writer.write_bool(payload.has_imu_motion))
    {
      return false;
    }
    if (!writer.write_bool(payload.has_fused_translation))
    {
      return false;
    }
    if (!writer.write_bool(payload.has_fused_rotation))
    {
      return false;
    }
    if (!writer.write_i64(payload.encoder_translation_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.encoder_rotation_urad))
    {
      return false;
    }
    if (!writer.write_i64(payload.encoder_translation_sum_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.encoder_rotation_sum_urad))
    {
      return false;
    }
    if (!writer.write_i64(payload.imu_translation_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.imu_rotation_urad))
    {
      return false;
    }
    if (!writer.write_i64(payload.imu_translation_sum_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.imu_rotation_sum_urad))
    {
      return false;
    }
    if (!writer.write_i64(payload.fused_translation_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.fused_rotation_urad))
    {
      return false;
    }
    if (!writer.write_i64(payload.fused_translation_sum_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.fused_rotation_sum_urad))
    {
      return false;
    }
    if (!writer.write_i64(payload.local_x_um))
    {
      return false;
    }
    if (!writer.write_i64(payload.local_y_um))
    {
      return false;
    }
    if (!writer.write_i32(payload.local_heading_urad))
    {
      return false;
    }
    if (!writer.write_u32(payload.local_update_id))
    {
      return false;
    }
    if (!writer.write_u32(payload.time_ms))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition local_position_model_debug_payload_definition = {
    "local_position_model_debug",
    build_payload_bytes
  };
}
