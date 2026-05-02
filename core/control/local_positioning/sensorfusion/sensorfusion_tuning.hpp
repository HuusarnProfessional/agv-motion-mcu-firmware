#pragma once

#include <cstdint>

namespace sensorfusion_tuning
{
  inline constexpr std::int64_t k_confidence_max = 1000;

  inline constexpr double k_encoder_rotation_radius_gain = 40.0;
  inline constexpr double k_encoder_rotation_radius_offset_cm = 1.65;
  inline constexpr double k_encoder_rotation_zero_confidence_radius_cm = 22.0;

  inline constexpr double k_gyro_rotation_observability_degps = 3.0;

  //value below is for  1D kalman filter.

  // maps gyro confidence to kalman process variance q
  inline constexpr double k_rotation_process_variance_min_urad2 = 100.0;
  inline constexpr double k_rotation_process_variance_max_urad2 = 1000000.0;

  // maps encoder confidence to kalman measurement variance r
  inline constexpr double k_rotation_measurement_variance_min_urad2 = 100.0;
  inline constexpr double k_rotation_measurement_variance_max_urad2 = 1000000.0;

  // maps kalman heading variance to fused output confidence
  inline constexpr double k_rotation_fused_variance_confidence_min_urad2 = 100.0;
  inline constexpr double k_rotation_fused_variance_confidence_max_urad2 = 1000000.0;

  // maps imu translation confidence to kalman process variance q
  inline constexpr double k_translation_process_variance_min_um2 = 100.0;
  inline constexpr double k_translation_process_variance_max_um2 = 1000000.0;

  // maps encoder translation confidence to kalman measurement variance r
  inline constexpr double k_translation_measurement_variance_min_um2 = 100.0;
  inline constexpr double k_translation_measurement_variance_max_um2 = 1000000.0;

  // maps kalman translation variance to fused output confidence
  inline constexpr double k_translation_fused_variance_confidence_min_um2 = 100.0;
  inline constexpr double k_translation_fused_variance_confidence_max_um2 = 1000000.0;

  // maps encoder translation confidence from turn radius
  inline constexpr double k_encoder_translation_radius_gain = 8.04;
  inline constexpr double k_encoder_translation_radius_offset_cm = 1.65;

}
