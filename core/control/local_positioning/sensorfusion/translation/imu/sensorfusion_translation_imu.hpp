#pragma once

#include "core/control/local_positioning/sensorfusion/translation/sensorfusion_translation.hpp"

namespace sensorfusion_translation_imu
{
  void update_translation_confidence(const motion_model_imu::motion_model_snapshot &imu_motion, sensorfusion_translation::translation_snapshot &out);
}
