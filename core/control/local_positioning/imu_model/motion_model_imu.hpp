#pragma once

#include <cstdint>

#include "core/control/local_positioning/imu_model/delta_estimation_imu.hpp"

namespace motion_model_imu
{
  struct motion_model_snapshot
  {
    bool has_motion_model = false;
    std::int32_t gyroscope_z_calibrated_mdps = 0;
    std::int64_t translation = 0;
    std::int64_t rotation = 0;
    std::int64_t confidence_translation = 0;
    std::int64_t confidence_rotation = 0;
    bool is_stationary = false;
  };

  struct motion_model_state
  {
    std::int64_t forward_velocity_um_per_s = 0;
  };

  void reset(motion_model_state &state);
  void reset(motion_model_snapshot &state);
  bool estimate_from_imu_delta(const delta_estimation_imu::delta_snapshot &delta_snapshot, motion_model_state &state, motion_model_snapshot &out);
}
