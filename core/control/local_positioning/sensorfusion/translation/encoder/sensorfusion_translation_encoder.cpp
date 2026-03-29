#include "core/control/local_positioning/sensorfusion/translation/encoder/sensorfusion_translation_encoder.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_tuning.hpp"

#include <cmath>

namespace
{
  std::int32_t compute_encoder_translation_math_model_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion)
  {
    if (encoder_motion.rotation == 0)
    {
      return sensorfusion_tuning::k_confidence_max;
    }

    const double translation_um = std::abs(static_cast<double>(encoder_motion.translation));
    const double rotation_urad = std::abs(static_cast<double>(encoder_motion.rotation));
    const double radius_cm = 100.0 * translation_um / rotation_urad;
    const double denominator = radius_cm + sensorfusion_tuning::k_encoder_translation_radius_offset_cm;
    const double exponent_argument = -sensorfusion_tuning::k_encoder_translation_radius_gain / denominator;
    const double confidence_percent = 100.0 * std::exp(exponent_argument);
    const double confidence_scaled = confidence_percent * sensorfusion_tuning::k_confidence_max / 100.0;

    return static_cast<std::int32_t>(confidence_scaled);
  }
}

namespace sensorfusion_translation_encoder
{
  void update_translation_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion, sensorfusion_translation::translation_snapshot &out)
  {
    out.encoder_confidence_translation_raw = encoder_motion.confidence_translation;
    out.encoder_confidence_translation_math_model = compute_encoder_translation_math_model_confidence(encoder_motion);

    if (out.encoder_confidence_translation_raw < out.encoder_confidence_translation_math_model)
    {
      out.encoder_confidence_translation_final = out.encoder_confidence_translation_raw;
    }
    else
    {
      out.encoder_confidence_translation_final = out.encoder_confidence_translation_math_model;
    }
  }
}
