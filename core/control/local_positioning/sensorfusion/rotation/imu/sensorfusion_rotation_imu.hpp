#pragma once

#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.hpp"

namespace sensorfusion_rotation_imu
{
  void update_rotation_confidence(const motion_model_imu::motion_model_snapshot &imu_motion, sensorfusion_rotation::rotation_snapshot &out);
}
