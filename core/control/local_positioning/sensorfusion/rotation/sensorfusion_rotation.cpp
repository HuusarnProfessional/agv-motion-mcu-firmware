#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/imu/sensorfusion_rotation_imu.hpp"

namespace sensorfusion_rotation
{
  void reset(rotation_snapshot &state)
  {
    state = {};
  }

  bool fuse_rotation(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, rotation_snapshot &out)
  {
    out = {};

    if (encoder_motion.has_motion_model && encoder_motion.confidence_rotation > 0)
    {
      out.has_encoder_rotation = true;
    }

    if (imu_motion.has_motion_model && imu_motion.confidence_rotation > 0)
    {
      out.has_gyro_rotation = true;
    }

    if (!out.has_encoder_rotation && !out.has_gyro_rotation)
    {
      return false;
    }

    if (out.has_encoder_rotation)
    {
      sensorfusion_rotation_encoder::update_rotation_confidence(encoder_motion, out);
    }

    if (out.has_gyro_rotation)
    {
      sensorfusion_rotation_imu::update_rotation_confidence(imu_motion, out);
    }

    if (out.has_encoder_rotation && out.encoder_confidence_rotation_final == 0)
    {
      out.has_encoder_rotation = false;
    }

    if (out.has_gyro_rotation && out.gyro_confidence_rotation_final == 0)
    {
      out.has_gyro_rotation = false;
    }

    if (!out.has_encoder_rotation && !out.has_gyro_rotation)
    {
      return false;
    }

    if (out.has_encoder_rotation && !out.has_gyro_rotation)
    {
      out.rotation = encoder_motion.rotation;
      out.confidence_rotation = out.encoder_confidence_rotation_final;
      out.has_fused_rotation = true;
      return true;
    }

    if (!out.has_encoder_rotation && out.has_gyro_rotation)
    {
      out.rotation = imu_motion.rotation;
      out.confidence_rotation = out.gyro_confidence_rotation_final;
      out.has_fused_rotation = true;
      return true;
    }

    return false;
  }
}
