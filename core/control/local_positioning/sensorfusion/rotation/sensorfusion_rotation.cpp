#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/imu/sensorfusion_rotation_imu.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_tuning.hpp"

namespace
{
  constexpr std::int64_t k_zero_encoder_rotation_with_active_gyro_threshold_urad = 200;

  struct heading_state_estimate
  {
    double theta_urad = 0.0;
    double p_urad2 = 0.0;
  };

  double map_gyro_confidence_to_process_variance(std::int64_t confidence_rotation)
  {
    double confidence_normalized = static_cast<double>(confidence_rotation);
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

    const double max_variance = sensorfusion_tuning::k_rotation_process_variance_max_urad2;
    const double min_variance = sensorfusion_tuning::k_rotation_process_variance_min_urad2;
    const double variance_range = max_variance - min_variance;
    const double variance = max_variance - confidence_normalized * variance_range;

    return variance;
  }

  double map_encoder_confidence_to_measurement_variance(std::int64_t confidence_rotation)
  {
    double confidence_normalized = static_cast<double>(confidence_rotation);
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

    const double max_variance = sensorfusion_tuning::k_rotation_measurement_variance_max_urad2;
    const double min_variance = sensorfusion_tuning::k_rotation_measurement_variance_min_urad2;
    const double variance_range = max_variance - min_variance;
    const double variance = max_variance - confidence_normalized * variance_range;

    return variance;
  }

  std::int64_t map_heading_variance_to_fused_confidence(double heading_variance_urad2)
  {
    double variance_normalized = 0.0;
    const double min_variance = sensorfusion_tuning::k_rotation_fused_variance_confidence_min_urad2;
    const double max_variance = sensorfusion_tuning::k_rotation_fused_variance_confidence_max_urad2;
    const double variance_range = max_variance - min_variance;

    if (variance_range > 0.0)
    {
      variance_normalized = (heading_variance_urad2 - min_variance) / variance_range;
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

  void initialize_heading_state_if_needed(sensorfusion_rotation::rotation_state &state)
  {
    if (!state.is_initialized)
    {
      state.is_initialized = true;
      state.theta_estimate_urad = 0.0;
      state.theta_encoder_accumulated_urad = 0.0;
      state.p_heading_urad2 = 0.0;
    }
  }

  void update_rotation_sensor_confidences(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, sensorfusion_rotation::rotation_snapshot &out)
  {
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

    if (out.has_encoder_rotation &&
        out.has_gyro_rotation &&
        encoder_motion.rotate_in_spot_flag)
    {
      out.encoder_confidence_rotation_final = 0;
      out.has_encoder_rotation = false;
    }

    if (out.has_encoder_rotation &&
        out.has_gyro_rotation &&
        encoder_motion.rotation == 0 &&
        ((imu_motion.rotation >= 0) ? imu_motion.rotation : -imu_motion.rotation) >= k_zero_encoder_rotation_with_active_gyro_threshold_urad)
    {
      out.encoder_confidence_rotation_final = 0;
      out.has_encoder_rotation = false;
    }
  }

  heading_state_estimate build_gyro_advanced_heading(const motion_model_imu::motion_model_snapshot &imu_motion, const sensorfusion_rotation::rotation_snapshot &out, const sensorfusion_rotation::rotation_state &state)
  {
    const double process_variance = map_gyro_confidence_to_process_variance(out.gyro_confidence_rotation_final);
    heading_state_estimate gyro_advanced_heading;

    gyro_advanced_heading.theta_urad = state.theta_estimate_urad;
    gyro_advanced_heading.p_urad2 = state.p_heading_urad2;

    if (out.has_gyro_rotation)
    {
      gyro_advanced_heading.theta_urad = state.theta_estimate_urad + static_cast<double>(imu_motion.rotation);
    }

    gyro_advanced_heading.p_urad2 = state.p_heading_urad2 + process_variance;

    return gyro_advanced_heading;
  }

  heading_state_estimate apply_encoder_heading_correction(const motion_model_encoders::motion_model_snapshot &encoder_motion, const sensorfusion_rotation::rotation_snapshot &out, sensorfusion_rotation::rotation_state &state, double previous_fused_heading_urad, const heading_state_estimate &gyro_advanced_heading)
  {
    heading_state_estimate encoder_corrected_heading = gyro_advanced_heading;

    if (out.has_encoder_rotation)
    {
      const double encoder_measured_theta_urad = previous_fused_heading_urad + static_cast<double>(encoder_motion.rotation);
      const double measurement_variance = map_encoder_confidence_to_measurement_variance(out.encoder_confidence_rotation_final);
      const double measurement_residual = encoder_measured_theta_urad - gyro_advanced_heading.theta_urad;
      const double innovation_variance = gyro_advanced_heading.p_urad2 + measurement_variance;
      const double kalman_gain = gyro_advanced_heading.p_urad2 / innovation_variance;

      encoder_corrected_heading.theta_urad = gyro_advanced_heading.theta_urad + kalman_gain * measurement_residual;
      encoder_corrected_heading.p_urad2 = (1.0 - kalman_gain) * gyro_advanced_heading.p_urad2;
      state.theta_encoder_accumulated_urad = encoder_measured_theta_urad;
    }

    return encoder_corrected_heading;
  }

  void write_fused_rotation_output(const heading_state_estimate &encoder_corrected_heading, double previous_fused_heading_urad, sensorfusion_rotation::rotation_state &state, sensorfusion_rotation::rotation_snapshot &out)
  {
    state.theta_estimate_urad = encoder_corrected_heading.theta_urad;
    state.p_heading_urad2 = encoder_corrected_heading.p_urad2;

    const double fused_rotation_urad = encoder_corrected_heading.theta_urad - previous_fused_heading_urad;

    out.rotation = static_cast<std::int64_t>(fused_rotation_urad);
    out.confidence_rotation = map_heading_variance_to_fused_confidence(encoder_corrected_heading.p_urad2);

    out.has_fused_rotation = true;
  }
}

namespace sensorfusion_rotation
{
  void reset(rotation_state &state)
  {
    state = {};
  }

  void reset(rotation_snapshot &state)
  {
    state = {};
  }

  bool fuse_rotation(const motion_model_encoders::motion_model_snapshot &encoder_motion, const motion_model_imu::motion_model_snapshot &imu_motion, rotation_state &state, rotation_snapshot &out)
  {
    out = {};

    if (encoder_motion.has_motion_model && encoder_motion.confidence_rotation > 0)
    {
      out.has_encoder_rotation = true;
      out.encoder_rotation = encoder_motion.rotation;
    }

    if (imu_motion.has_motion_model && imu_motion.confidence_rotation > 0)
    {
      out.has_gyro_rotation = true;
      out.gyro_rotation = imu_motion.rotation;
    }

    if (!out.has_encoder_rotation && !out.has_gyro_rotation)
    {
      return false;
    }

    initialize_heading_state_if_needed(state);
    update_rotation_sensor_confidences(encoder_motion, imu_motion, out);

    if (!out.has_encoder_rotation && !out.has_gyro_rotation)
    {
      return false;
    }

    const double previous_fused_heading_urad = state.theta_estimate_urad;
    const heading_state_estimate gyro_advanced_heading = build_gyro_advanced_heading(imu_motion, out, state);
    const heading_state_estimate encoder_corrected_heading = apply_encoder_heading_correction(encoder_motion, out, state, previous_fused_heading_urad, gyro_advanced_heading);

    write_fused_rotation_output(encoder_corrected_heading, previous_fused_heading_urad, state, out);
    return true;
  }
}
