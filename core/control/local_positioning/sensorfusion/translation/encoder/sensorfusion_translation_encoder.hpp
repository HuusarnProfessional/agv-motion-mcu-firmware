#pragma once

#include "core/control/local_positioning/sensorfusion/translation/sensorfusion_translation.hpp"

namespace sensorfusion_translation_encoder
{
  void update_translation_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion, sensorfusion_translation::translation_snapshot &out);
}
