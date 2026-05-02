#include "core/middleware/outgoing_payloads/debug/motion_debug_payload.hpp"

#include "core/control/drive_control/wheel_drive_controller.hpp"
#include "core/middleware/payload_helper_functions.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    wheel_drive_controller::motion_debug_snapshot payload = {};
    payload_length_out = 0u;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    wheel_drive_controller::read_motion_debug_snapshot(payload);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_bool(payload.drive_enabled))
    {
      return false;
    }
    if (!writer.write_bool(payload.motion_session_active))
    {
      return false;
    }
    if (!writer.write_bool(payload.safe_guard_latched))
    {
      return false;
    }
    if (!writer.write_bool(payload.motion_command_stale))
    {
      return false;
    }
    if (!writer.write_bool(payload.has_pose))
    {
      return false;
    }
    if (!writer.write_bool(payload.pose_is_fresh))
    {
      return false;
    }
    if (!writer.write_bool(payload.heading_feedback_active))
    {
      return false;
    }
    if (!writer.write_bool(payload.has_not_ready_feedback))
    {
      return false;
    }
    if (!writer.write_bool(payload.has_invalid_feedback))
    {
      return false;
    }
    if (!writer.write_i32(payload.commanded_linear_velocity_mm_s))
    {
      return false;
    }
    if (!writer.write_i32(payload.commanded_yaw_rate_mdeg_s))
    {
      return false;
    }
    if (!writer.write_i32(payload.corrected_yaw_rate_mdeg_s))
    {
      return false;
    }
    if (!writer.write_i32(payload.measured_yaw_rate_mdeg_s))
    {
      return false;
    }
    if (!writer.write_i32(payload.outer_correction_mdeg_s))
    {
      return false;
    }
    if (!writer.write_u16(payload.pose_confidence_heading))
    {
      return false;
    }
    if (!writer.write_u32(payload.pose_age_ms))
    {
      return false;
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_i32(payload.wheel_targets_mm_s[wheel_id]))
      {
        return false;
      }
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_i32(payload.wheel_speeds_mm_s[wheel_id]))
      {
        return false;
      }
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_u32(payload.wheel_sample_ids[wheel_id]))
      {
        return false;
      }
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_u32(payload.wheel_sample_age_ms[wheel_id]))
      {
        return false;
      }
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_bool(payload.wheel_has_new_sample[wheel_id]))
      {
        return false;
      }
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_bool(payload.wheel_has_measured_speed[wheel_id]))
      {
        return false;
      }
    }

    for (std::uint8_t wheel_id = 0u; wheel_id < 4u; ++wheel_id)
    {
      if (!writer.write_i16(payload.wheel_drive_u[wheel_id]))
      {
        return false;
      }
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
  const outgoing_payload_definition motion_debug_payload_definition = {
    "motion_debug",

    build_payload_bytes
  };
}
