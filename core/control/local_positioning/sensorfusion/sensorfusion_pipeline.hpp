#pragma once

#include "core/control/local_positioning/encoder_model/motion_model_encoders.hpp"
#include "core/control/local_positioning/imu_model/motion_model_imu.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.hpp"
#include "core/control/local_positioning/sensorfusion/translation/sensorfusion_translation.hpp"

namespace local_positioning_sensorfusion
{
  struct state
  {
    sensorfusion_rotation::rotation_state rotation_state = {};
    sensorfusion_rotation::rotation_snapshot rotation_snapshot = {};
    sensorfusion_translation::translation_state translation_state = {};
    sensorfusion_translation::translation_snapshot translation_snapshot = {};
  };

  void reset(state &local_positioning_sensorfusion_state);
  void tick(state &local_positioning_sensorfusion_state, const motion_model_encoders::motion_model_snapshot &encoder_motion_snapshot, const motion_model_imu::motion_model_snapshot &imu_motion_snapshot);
}
