#include "core/control/local_positioning/local_positioning_pipeline.hpp"

namespace
{
  struct pipeline_state
  {
    encoder_motion::state encoder_motion_state = {};
    local_positioning_imu::state local_positioning_imu_state = {};
    local_positioning_sensorfusion::state local_positioning_sensorfusion_state = {};
    local_positioning::state local_positioning_state = {};
    std::uint32_t tick_id = 0U;
    std::uint8_t imu_id = 0U;
  };

  pipeline_state g_pipeline_state = {};
}

namespace local_positioning_pipeline
{
  void init(void)
  {
    g_pipeline_state = {};
    encoder_motion::reset(g_pipeline_state.encoder_motion_state);
    local_positioning_imu::reset(g_pipeline_state.local_positioning_imu_state);
    local_positioning_sensorfusion::reset(g_pipeline_state.local_positioning_sensorfusion_state);
    local_positioning::reset(g_pipeline_state.local_positioning_state);
  }

  void tick(std::uint32_t now_ms)
  {
    g_pipeline_state.tick_id += 1U;

    encoder_motion::tick(g_pipeline_state.encoder_motion_state, g_pipeline_state.tick_id);
    local_positioning_imu::tick(g_pipeline_state.local_positioning_imu_state, g_pipeline_state.imu_id, g_pipeline_state.tick_id, now_ms);
    local_positioning_sensorfusion::tick(
      g_pipeline_state.local_positioning_sensorfusion_state,
      g_pipeline_state.encoder_motion_state.encoder_motion_snapshot,
      g_pipeline_state.local_positioning_imu_state.motion_snapshot);
    local_positioning::tick(g_pipeline_state.local_positioning_state, g_pipeline_state.local_positioning_sensorfusion_state);
  }

  void read_snapshot(local_positioning::snapshot &out)
  {
    out = g_pipeline_state.local_positioning_state.output_snapshot;
  }

  void read_encoder_motion_state(encoder_motion::state &out)
  {
    out = g_pipeline_state.encoder_motion_state;
  }

  std::uint8_t read_imu_id(void)
  {
    return g_pipeline_state.imu_id;
  }
}
