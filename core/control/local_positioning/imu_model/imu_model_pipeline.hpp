#pragma once

#include <cstdint>

#include "core/control/local_positioning/imu_model/delta_estimation_imu.hpp"
#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"
#include "core/control/local_positioning/imu_model/motion_model_imu.hpp"
#include "core/control/local_positioning/imu_model/stationary_detection_imu.hpp"

namespace local_positioning_imu
{
  struct state
  {
    imu_input_storage::imu_sample_snapshot input_snapshot = {};
    stationary_detection_imu::stationary_snapshot stationary_snapshot = {};
    delta_estimation_imu::delta_snapshot delta_snapshot = {};
    motion_model_imu::motion_model_state motion_state = {};
    motion_model_imu::motion_model_snapshot motion_snapshot = {};
  };

  void reset(state &local_positioning_imu_state);
  void tick(state &local_positioning_imu_state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms);
}
