#include "core/control/local_positioning/sensorfusion/translation/imu/sensorfusion_translation_imu.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_tuning.hpp"

namespace sensorfusion_translation_imu
{
  void update_translation_confidence(const motion_model_imu::motion_model_snapshot &imu_motion, sensorfusion_translation::translation_snapshot &out)
  {
    out.imu_confidence_translation_raw = imu_motion.confidence_translation;
    out.imu_confidence_translation_math_model = sensorfusion_tuning::k_confidence_max;

    if (out.imu_confidence_translation_raw < out.imu_confidence_translation_math_model)
    {
      out.imu_confidence_translation_final = out.imu_confidence_translation_raw;
    }
    else
    {
      out.imu_confidence_translation_final = out.imu_confidence_translation_math_model;
    }
  }
}
