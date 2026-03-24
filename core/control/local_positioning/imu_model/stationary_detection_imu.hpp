#pragma once

#include <cstdint>

#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"

namespace stationary_detection_imu
{
  struct stationary_snapshot
  {
    bool is_stationary = false;
    bool has_stationary_detection = false;
  };

  void reset(stationary_snapshot &state);
  bool estimate_from_imu_snapshot(const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot, stationary_snapshot &out);
}
