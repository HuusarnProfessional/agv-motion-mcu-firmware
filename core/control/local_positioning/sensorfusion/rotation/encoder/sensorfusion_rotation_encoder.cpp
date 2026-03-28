#include "core/control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_tuning.hpp"

#include <cmath>

namespace
{
  std::int32_t compute_encoder_rotation_math_model_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion)
  {
    if (encoder_motion.rotation == 0)
    {
      return sensorfusion_tuning::k_confidence_max;
    }

    const double translation_um = std::abs(static_cast<double>(encoder_motion.translation));
    const double rotation_urad = std::abs(static_cast<double>(encoder_motion.rotation));
    const double radius_cm = 100.0 * translation_um / rotation_urad;
    const double denominator = radius_cm + sensorfusion_tuning::k_encoder_rotation_radius_offset_cm;
    const double exponent_argument = -sensorfusion_tuning::k_encoder_rotation_radius_gain / denominator;
    const double confidence_percent = 100.0 * std::exp(exponent_argument);
    const double confidence_scaled = confidence_percent * sensorfusion_tuning::k_confidence_max / 100.0;

    return static_cast<std::int32_t>(confidence_scaled);
  }
}

namespace sensorfusion_rotation_encoder
{
  void update_rotation_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion, sensorfusion_rotation::rotation_snapshot &out)
  {
    out.encoder_confidence_rotation_raw = encoder_motion.confidence_rotation;
    out.encoder_confidence_rotation_math_model = compute_encoder_rotation_math_model_confidence(encoder_motion);

    if (out.encoder_confidence_rotation_raw < out.encoder_confidence_rotation_math_model)
    {
      out.encoder_confidence_rotation_final = out.encoder_confidence_rotation_raw;
    }
    else
    {
      out.encoder_confidence_rotation_final = out.encoder_confidence_rotation_math_model;
    }
  }
}
