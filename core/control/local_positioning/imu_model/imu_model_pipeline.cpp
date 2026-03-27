#include "core/control/local_positioning/imu_model/imu_model_pipeline.hpp"

namespace local_positioning_imu
{
  void reset(state &local_positioning_imu_state)
  {
    imu_input_storage::reset(local_positioning_imu_state.input_snapshot);
    stationary_detection_imu::reset(local_positioning_imu_state.stationary_snapshot);
    delta_estimation_imu::reset(local_positioning_imu_state.delta_snapshot);
    motion_model_imu::reset(local_positioning_imu_state.motion_state);
    motion_model_imu::reset(local_positioning_imu_state.motion_snapshot);
  }

  void tick(state &local_positioning_imu_state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms)
  {
    delta_estimation_imu::reset(local_positioning_imu_state.delta_snapshot);
    motion_model_imu::reset(local_positioning_imu_state.motion_snapshot);
    local_positioning_imu_state.stationary_snapshot.is_stationary = false;
    local_positioning_imu_state.stationary_snapshot.has_stationary_detection = false;

    if (!imu_input_storage::sample_from_imu_api(local_positioning_imu_state.input_snapshot, imu_id, tick_id, now_ms))
    {
      return;
    }

    if (!stationary_detection_imu::estimate_from_imu_snapshot(local_positioning_imu_state.input_snapshot, local_positioning_imu_state.stationary_snapshot))
    {
      return;
    }

    if (!delta_estimation_imu::estimate_from_imu_snapshot(
            local_positioning_imu_state.input_snapshot,
            local_positioning_imu_state.stationary_snapshot,
            local_positioning_imu_state.delta_snapshot))
    {
      return;
    }

    motion_model_imu::estimate_from_imu_delta(
        local_positioning_imu_state.delta_snapshot,
        local_positioning_imu_state.motion_state,
        local_positioning_imu_state.motion_snapshot);
  }
}
