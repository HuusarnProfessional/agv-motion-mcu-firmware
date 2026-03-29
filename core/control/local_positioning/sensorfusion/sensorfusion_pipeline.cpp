#include "core/control/local_positioning/sensorfusion/sensorfusion_pipeline.hpp"

namespace local_positioning_sensorfusion
{
  void reset(state &local_positioning_sensorfusion_state)
  {
    sensorfusion_rotation::reset(local_positioning_sensorfusion_state.rotation_state);
    sensorfusion_rotation::reset(local_positioning_sensorfusion_state.rotation_snapshot);
    sensorfusion_translation::reset(local_positioning_sensorfusion_state.translation_state);
    sensorfusion_translation::reset(local_positioning_sensorfusion_state.translation_snapshot);
  }

  void tick(state &local_positioning_sensorfusion_state, const motion_model_encoders::motion_model_snapshot &encoder_motion_snapshot, const motion_model_imu::motion_model_snapshot &imu_motion_snapshot)
  {
    sensorfusion_rotation::reset(local_positioning_sensorfusion_state.rotation_snapshot);
    sensorfusion_translation::reset(local_positioning_sensorfusion_state.translation_snapshot);
    sensorfusion_rotation::fuse_rotation(encoder_motion_snapshot, imu_motion_snapshot, local_positioning_sensorfusion_state.rotation_state, local_positioning_sensorfusion_state.rotation_snapshot);
    sensorfusion_translation::fuse_translation(encoder_motion_snapshot, imu_motion_snapshot, local_positioning_sensorfusion_state.translation_state, local_positioning_sensorfusion_state.translation_snapshot);
  }
}
