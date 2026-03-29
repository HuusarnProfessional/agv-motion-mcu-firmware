#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/motion_model_encoders.hpp"
#include "core/control/local_positioning/imu_model/motion_model_imu.hpp"

namespace sensorfusion_translation
{
  struct translation_state
  {
    bool is_initialized = false;
    double s_estimate_um = 0.0;
    double s_encoder_accumulated_um = 0.0;
    double p_translation_um2 = 0.0;
  };

  struct translation_snapshot
  {
    bool has_fused_translation = false;
    bool has_encoder_translation = false;
    bool has_imu_translation = false;

    std::int64_t translation = 0;
    std::int64_t confidence_translation = 0;

    std::int64_t encoder_confidence_translation_raw = 0;
    std::int64_t encoder_confidence_translation_math_model = 0;
    std::int64_t encoder_confidence_translation_final = 0;

    std::int64_t imu_confidence_translation_raw = 0;
    std::int64_t imu_confidence_translation_math_model = 0;
    std::int64_t imu_confidence_translation_final = 0;
  };

  void reset(translation_state &state);
  void reset(translation_snapshot &state);
  bool fuse_translation(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, translation_state &state, translation_snapshot &out);
}
