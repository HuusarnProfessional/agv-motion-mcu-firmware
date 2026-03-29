#include "core/control/local_positioning/sensorfusion/translation/sensorfusion_translation.hpp"
#include "core/control/local_positioning/sensorfusion/translation/encoder/sensorfusion_translation_encoder.hpp"
#include "core/control/local_positioning/sensorfusion/translation/imu/sensorfusion_translation_imu.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_tuning.hpp"

namespace
{
  struct translation_estimate
  {
    double s_um = 0.0;
    double p_um2 = 0.0;
  };

  double map_imu_confidence_to_process_variance(std::int64_t confidence_translation)
  {
    double confidence_normalized = static_cast<double>(confidence_translation);
    const double confidence_max = static_cast<double>(sensorfusion_tuning::k_confidence_max);

    confidence_normalized = confidence_normalized / confidence_max;

    if (confidence_normalized < 0.0)
    {
      confidence_normalized = 0.0;
    }

    if (confidence_normalized > 1.0)
    {
      confidence_normalized = 1.0;
    }

    const double max_variance = sensorfusion_tuning::k_translation_process_variance_max_um2;
    const double min_variance = sensorfusion_tuning::k_translation_process_variance_min_um2;
    const double variance_range = max_variance - min_variance;
    const double variance = max_variance - confidence_normalized * variance_range;

    return variance;
  }

  double map_encoder_confidence_to_measurement_variance(std::int64_t confidence_translation)
  {
    double confidence_normalized = static_cast<double>(confidence_translation);
    const double confidence_max = static_cast<double>(sensorfusion_tuning::k_confidence_max);

    confidence_normalized = confidence_normalized / confidence_max;

    if (confidence_normalized < 0.0)
    {
      confidence_normalized = 0.0;
    }

    if (confidence_normalized > 1.0)
    {
      confidence_normalized = 1.0;
    }

    const double max_variance = sensorfusion_tuning::k_translation_measurement_variance_max_um2;
    const double min_variance = sensorfusion_tuning::k_translation_measurement_variance_min_um2;
    const double variance_range = max_variance - min_variance;
    const double variance = max_variance - confidence_normalized * variance_range;

    return variance;
  }

  std::int64_t map_translation_variance_to_fused_confidence(double translation_variance_um2)
  {
    double variance_normalized = 0.0;
    const double min_variance = sensorfusion_tuning::k_translation_fused_variance_confidence_min_um2;
    const double max_variance = sensorfusion_tuning::k_translation_fused_variance_confidence_max_um2;
    const double variance_range = max_variance - min_variance;

    if (variance_range > 0.0)
    {
      variance_normalized = (translation_variance_um2 - min_variance) / variance_range;
    }

    if (variance_normalized < 0.0)
    {
      variance_normalized = 0.0;
    }

    if (variance_normalized > 1.0)
    {
      variance_normalized = 1.0;
    }

    const double confidence_normalized = 1.0 - variance_normalized;
    const double confidence_scaled = confidence_normalized * sensorfusion_tuning::k_confidence_max;

    return static_cast<std::int64_t>(confidence_scaled);
  }

  void initialize_state_if_needed(sensorfusion_translation::translation_state &state)
  {
    if (!state.is_initialized)
    {
      state.is_initialized = true;
      state.s_estimate_um = 0.0;
      state.s_encoder_accumulated_um = 0.0;
      state.p_translation_um2 = 0.0;
    }
  }

  void update_sensor_confidences(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, sensorfusion_translation::translation_snapshot &out)
  {
    if (out.has_encoder_translation)
    {
      sensorfusion_translation_encoder::update_translation_confidence(encoder_motion, out);
    }

    if (out.has_imu_translation)
    {
      sensorfusion_translation_imu::update_translation_confidence(imu_motion, out);
    }

    if (out.has_encoder_translation && out.encoder_confidence_translation_final == 0)
    {
      out.has_encoder_translation = false;
    }

    if (out.has_imu_translation && out.imu_confidence_translation_final == 0)
    {
      out.has_imu_translation = false;
    }
  }

  translation_estimate predict_with_imu(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, const sensorfusion_translation::translation_snapshot &out, const sensorfusion_translation::translation_state &state)
  {
    const double process_variance = map_imu_confidence_to_process_variance(out.imu_confidence_translation_final);
    translation_estimate predicted_estimate;

    predicted_estimate.s_um = state.s_estimate_um;
    predicted_estimate.p_um2 = state.p_translation_um2;

    if (out.has_imu_translation)
    {
      predicted_estimate.s_um =
          state.s_estimate_um +
          static_cast<double>(imu_motion.translation);
    }

    if (imu_motion.is_stationary)
    {
      const std::int64_t encoder_translation_abs =
          encoder_motion.translation >= 0 ? encoder_motion.translation : -encoder_motion.translation;

      if (encoder_translation_abs <= 1)
      {
        predicted_estimate.s_um = state.s_estimate_um;
      }
    }

    predicted_estimate.p_um2 =
        state.p_translation_um2 +
        process_variance;

    return predicted_estimate;
  }

  translation_estimate correct_predicted_translation_with_encoder(const motion_model_encoders::motion_model_snapshot &encoder_motion, const sensorfusion_translation::translation_snapshot &out, sensorfusion_translation::translation_state &state, const translation_estimate &predicted_estimate)
  {
    translation_estimate corrected_estimate = predicted_estimate;

    if (out.has_encoder_translation)
    {
      state.s_encoder_accumulated_um =
          state.s_encoder_accumulated_um +
          static_cast<double>(encoder_motion.translation);

      const double measurement_variance =
          map_encoder_confidence_to_measurement_variance(out.encoder_confidence_translation_final);
      const double measurement_residual =
          state.s_encoder_accumulated_um -
          predicted_estimate.s_um;
      const double innovation_variance =
          predicted_estimate.p_um2 +
          measurement_variance;
      const double kalman_gain =
          predicted_estimate.p_um2 /
          innovation_variance;

      corrected_estimate.s_um =
          predicted_estimate.s_um +
          kalman_gain * measurement_residual;
      corrected_estimate.p_um2 =
          (1.0 - kalman_gain) *
          predicted_estimate.p_um2;
    }

    return corrected_estimate;
  }

  void write_translation_output(const translation_estimate &corrected_estimate, double previous_s_estimate_um, sensorfusion_translation::translation_state &state, sensorfusion_translation::translation_snapshot &out)
  {
    state.s_estimate_um = corrected_estimate.s_um;
    state.p_translation_um2 = corrected_estimate.p_um2;

    const double fused_translation_um =
        corrected_estimate.s_um -
        previous_s_estimate_um;

    out.translation = static_cast<std::int64_t>(fused_translation_um);
    out.confidence_translation =
        map_translation_variance_to_fused_confidence(corrected_estimate.p_um2);

    out.has_fused_translation = true;
  }
}

namespace sensorfusion_translation
{
  void reset(translation_state &state)
  {
    state = {};
  }

  void reset(translation_snapshot &state)
  {
    state = {};
  }

  bool fuse_translation(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, translation_state &state, translation_snapshot &out)
  {
    out = {};

    if (encoder_motion.has_motion_model && encoder_motion.confidence_translation > 0)
    {
      out.has_encoder_translation = true;
    }

    if (imu_motion.has_motion_model && imu_motion.confidence_translation > 0)
    {
      out.has_imu_translation = true;
    }

    if (!out.has_encoder_translation && !out.has_imu_translation)
    {
      return false;
    }

    initialize_state_if_needed(state);
    update_sensor_confidences(encoder_motion, imu_motion, out);

    if (!out.has_encoder_translation && !out.has_imu_translation)
    {
      return false;
    }

    const double previous_s_estimate_um = state.s_estimate_um;
    const translation_estimate predicted_estimate =
        predict_with_imu(encoder_motion, imu_motion, out, state);
    const translation_estimate corrected_estimate =
        correct_predicted_translation_with_encoder(encoder_motion, out, state, predicted_estimate);

    write_translation_output(corrected_estimate, previous_s_estimate_um, state, out);
    return true;
  }
}
