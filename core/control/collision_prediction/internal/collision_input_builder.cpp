#include "core/control/collision_prediction/internal/collision_input_builder.hpp"

#include "core/control/drive_control/drive_control_pipeline.hpp"
#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/middleware/middleware_shared_state.hpp"

namespace collision_input_builder
{
  void build(std::uint32_t now_ms, vehicle_motion_estimator::state &motion_state, const collision_prediction::runtime_config &config, collision_prediction_input &out)
  {
    local_positioning::snapshot local_positioning_snapshot = {};
    drive_control::snapshot drive_control_snapshot = {};
    vehicle_motion_estimator::input motion_input = {};

    out = {};
    out.now_ms = now_ms;
    out.has_trailer = middleware::g_middleware_state.has_trailer;
    out.obstacle_safety_enabled = config.obstacle_safety_enabled;

    local_positioning_pipeline::read_snapshot(local_positioning_snapshot);
    drive_control::read_snapshot(drive_control_snapshot);

    motion_input.now_ms = now_ms;
    motion_input.local_positioning_snapshot = local_positioning_snapshot;
    motion_input.has_motion_command = drive_control_snapshot.has_motion_command;
    motion_input.motion_command = drive_control_snapshot.latest_motion_command;

    vehicle_motion_estimator::estimate(motion_state, motion_input, out.motion);

    for (std::size_t sensor_index = 0u; sensor_index < collision_tuning::k_sensor_count; ++sensor_index)
    {
      out.sensor_inputs[sensor_index].config = collision_tuning::k_sensor_configs[sensor_index];

      if (!out.obstacle_safety_enabled)
      {
        out.sensor_inputs[sensor_index].config.is_enabled = false;
      }

      if (out.has_trailer)
      {
        if (out.sensor_inputs[sensor_index].config.role == collision_tuning::obstacle_sensor_role::rear)
        {
          out.sensor_inputs[sensor_index].config.is_masked = true;
        }
      }

      obstacle_api::read_sample(collision_tuning::k_sensor_configs[sensor_index].sensor_id, out.sensor_inputs[sensor_index].sample);
    }
  }
}
