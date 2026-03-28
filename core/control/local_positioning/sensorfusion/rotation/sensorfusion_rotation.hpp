#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/motion_model_encoders.hpp"
#include "core/control/local_positioning/imu_model/motion_model_imu.hpp"

namespace sensorfusion_rotation
{
  struct rotation_snapshot
  {
    bool has_fused_rotation = false;
    bool has_encoder_rotation = false;
    bool has_gyro_rotation = false;

    std::int64_t rotation = 0;
    std::int64_t confidence_rotation = 0;

    std::int64_t encoder_confidence_rotation_raw = 0;
    std::int64_t encoder_confidence_rotation_math_model = 0;
    std::int64_t encoder_confidence_rotation_final = 0;

    std::int64_t gyro_confidence_rotation_raw = 0;
    std::int64_t gyro_confidence_rotation_math_model = 0;
    std::int64_t gyro_confidence_rotation_final = 0;

    std::int64_t encoder_weight_rotation = 0;
    std::int64_t gyro_weight_rotation = 0;
  };

  void reset(rotation_snapshot &state);
  bool fuse_rotation(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, rotation_snapshot &out);
}
