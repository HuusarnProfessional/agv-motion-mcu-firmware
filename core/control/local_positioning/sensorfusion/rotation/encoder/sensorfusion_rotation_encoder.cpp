#include "core/control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_tuning.hpp"

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

    if (radius_cm <= sensorfusion_tuning::k_encoder_rotation_zero_confidence_radius_cm)
    {
      return static_cast<std::int32_t>(sensorfusion_tuning::k_min_active_sensor_confidence);
    }

    const double denominator = radius_cm + sensorfusion_tuning::k_encoder_rotation_radius_offset_cm;
    const double exponent_argument = -sensorfusion_tuning::k_encoder_rotation_radius_gain / denominator;
    const double confidence_percent = 100.0 * std::exp(exponent_argument);
    const double confidence_scaled = confidence_percent * sensorfusion_tuning::k_confidence_max / 100.0;

    return static_cast<std::int32_t>(confidence_scaled);
  }

  std::int32_t combine_radius_and_slip_confidence(std::int32_t radius_confidence, std::int64_t slip_confidence)
  {
    if (radius_confidence <= 0 || slip_confidence <= 0)
    {
      return 0;
    }

    const std::int64_t confidence_product = static_cast<std::int64_t>(radius_confidence) * slip_confidence;
    const std::int64_t combined_confidence = confidence_product / sensorfusion_tuning::k_confidence_max;

    return static_cast<std::int32_t>(combined_confidence);
  }
}

namespace sensorfusion_rotation_encoder
{
  void update_rotation_confidence(const motion_model_encoders::motion_model_snapshot &encoder_motion, sensorfusion_rotation::rotation_snapshot &out)
  {
    out.encoder_confidence_rotation_raw = encoder_motion.confidence_rotation;
    const std::int32_t radius_confidence = compute_encoder_rotation_math_model_confidence(encoder_motion);
    out.encoder_confidence_rotation_math_model = combine_radius_and_slip_confidence(radius_confidence, encoder_motion.confidence_slip);

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
