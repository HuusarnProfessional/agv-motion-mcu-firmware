#include "core/control/controller_robot_future.hpp"

#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include "core/control/local_positioning/imu_model/imu_model_pipeline.hpp"
#include "core/control/local_positioning/local_positioning.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_pipeline.hpp"

namespace controller_robot_future
{
  namespace
  {
    encoder_motion::state g_encoder_motion_state = {};
    local_positioning_imu::state g_local_positioning_imu_state = {};
    local_positioning_sensorfusion::state g_local_positioning_sensorfusion_state = {};
    local_positioning::state g_local_positioning_state = {};
    std::uint32_t g_tick_id = 0U;

    constexpr std::uint8_t k_local_positioning_imu_id = 0U;
  }

  void init(void)
  {
    encoder_motion::reset(g_encoder_motion_state);
    local_positioning_imu::reset(g_local_positioning_imu_state);
    local_positioning_sensorfusion::reset(g_local_positioning_sensorfusion_state);
    local_positioning::reset(g_local_positioning_state);
    g_tick_id = 0U;
  }

  void tick(std::uint32_t now_ms)
  {
    g_tick_id += 1U;

    encoder_motion::tick(g_encoder_motion_state, g_tick_id);
    local_positioning_imu::tick(g_local_positioning_imu_state, k_local_positioning_imu_id, g_tick_id, now_ms);
    local_positioning_sensorfusion::tick(
        g_local_positioning_sensorfusion_state,
        g_encoder_motion_state.encoder_motion_snapshot,
        g_local_positioning_imu_state.motion_snapshot);
    local_positioning::tick(g_local_positioning_state, g_local_positioning_sensorfusion_state);
  }

  bool read_local_positioning_snapshot(local_positioning::snapshot &out)
  {
    out = g_local_positioning_state.output_snapshot;
    return out.has_pose;
  }
}
