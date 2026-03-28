#pragma once

#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.hpp"

namespace sensorfusion_rotation_encoder
{
  void update_rotation_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion, sensorfusion_rotation::rotation_snapshot &out);
}
