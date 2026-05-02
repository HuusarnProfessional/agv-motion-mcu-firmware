#include "core/control/local_positioning/test/local_position_model_debug.hpp"

namespace local_position_model_debug
{
  void reset(state &debug_state)
  {
    debug_state = {};
  }

  void tick(state &debug_state,
            const motion_model_encoders::motion_model_snapshot &encoder_motion_snapshot,
            const motion_model_imu::motion_model_snapshot &imu_motion_snapshot,
            const local_positioning_sensorfusion::state &sensorfusion_state,
            const local_positioning::state &local_positioning_state,
            std::uint32_t now_ms)
  {
    debug_state.output_snapshot.has_encoder_motion = encoder_motion_snapshot.has_motion_model;
    debug_state.output_snapshot.has_imu_motion = imu_motion_snapshot.has_motion_model;
    debug_state.output_snapshot.has_fused_translation = sensorfusion_state.translation_snapshot.has_fused_translation;
    debug_state.output_snapshot.has_fused_rotation = sensorfusion_state.rotation_snapshot.has_fused_rotation;
    debug_state.output_snapshot.encoder_translation_um = 0;
    debug_state.output_snapshot.encoder_rotation_urad = 0;
    debug_state.output_snapshot.imu_translation_um = 0;
    debug_state.output_snapshot.imu_rotation_urad = 0;
    debug_state.output_snapshot.fused_translation_um = 0;
    debug_state.output_snapshot.fused_rotation_urad = 0;

    if (encoder_motion_snapshot.has_motion_model)
    {
      debug_state.output_snapshot.encoder_translation_um = encoder_motion_snapshot.translation;
      debug_state.output_snapshot.encoder_rotation_urad = encoder_motion_snapshot.rotation;
      debug_state.output_snapshot.encoder_translation_sum_um += encoder_motion_snapshot.translation;
      debug_state.output_snapshot.encoder_rotation_sum_urad += encoder_motion_snapshot.rotation;
    }

    if (imu_motion_snapshot.has_motion_model)
    {
      debug_state.output_snapshot.imu_translation_um = imu_motion_snapshot.translation;
      debug_state.output_snapshot.imu_rotation_urad = imu_motion_snapshot.rotation;
      debug_state.output_snapshot.imu_translation_sum_um += imu_motion_snapshot.translation;
      debug_state.output_snapshot.imu_rotation_sum_urad += imu_motion_snapshot.rotation;
    }

    if (sensorfusion_state.translation_snapshot.has_fused_translation)
    {
      debug_state.output_snapshot.fused_translation_um = sensorfusion_state.translation_snapshot.translation;
      debug_state.output_snapshot.fused_translation_sum_um += sensorfusion_state.translation_snapshot.translation;
    }

    if (sensorfusion_state.rotation_snapshot.has_fused_rotation)
    {
      debug_state.output_snapshot.fused_rotation_urad = sensorfusion_state.rotation_snapshot.rotation;
      debug_state.output_snapshot.fused_rotation_sum_urad += sensorfusion_state.rotation_snapshot.rotation;
    }

    debug_state.output_snapshot.local_x_um = local_positioning_state.live_pose.x_um;
    debug_state.output_snapshot.local_y_um = local_positioning_state.live_pose.y_um;
    debug_state.output_snapshot.local_heading_urad = local_positioning_state.live_pose.heading_urad;
    debug_state.output_snapshot.local_update_id = local_positioning_state.live_pose.update_id;
    debug_state.output_snapshot.time_ms = now_ms;
  }
}
