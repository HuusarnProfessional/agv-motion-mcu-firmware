#pragma once

#include <cstdint>

#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"

namespace stationary_detection_imu
{
  struct stationary_snapshot
  {
    std::int32_t filtered_gyroscope_z_mdps = 0;
    std::int32_t filtered_accelerometer_x_mg = 0;
    std::int32_t filtered_accelerometer_y_mg = 0;
    bool has_filter_state = false;
    bool is_stationary = false;
    bool has_stationary_detection = false;
  };

  void reset(stationary_snapshot &state);
  bool estimate_from_imu_snapshot(const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot, stationary_snapshot &out);
}
