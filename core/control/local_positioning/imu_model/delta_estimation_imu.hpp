#pragma once

#include <cstdint>

#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"
#include "core/control/local_positioning/imu_model/stationary_detection_imu.hpp"

namespace delta_estimation_imu
{
  struct delta_snapshot
  {
    std::uint8_t imu_id = 0u;
    std::uint32_t previous_tick_id = 0u;
    std::uint32_t current_tick_id = 0u;
    std::uint32_t dt_ms = 0u;
    std::int32_t gyroscope_z_calibrated_mdps = 0;
    std::int32_t accelerometer_x_calibrated_mg = 0;
    std::int32_t accelerometer_y_calibrated_mg = 0;
    std::int32_t accelerometer_z_calibrated_mg = 0;
    std::int64_t delta_rotation_urad = 0;
    bool is_stationary = false;
    bool has_delta = false;
  };

  void reset(delta_snapshot &state);
  bool estimate_from_imu_snapshot(
      const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot,
      const stationary_detection_imu::stationary_snapshot &stationary_snapshot,
      delta_snapshot &out);
}
