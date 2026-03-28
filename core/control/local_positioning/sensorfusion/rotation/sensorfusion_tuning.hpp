#pragma once

#include <cstdint>

namespace sensorfusion_tuning
{
  inline constexpr std::int64_t k_confidence_max = 1000;

  inline constexpr double k_encoder_rotation_radius_gain = 8.04;
  inline constexpr double k_encoder_rotation_radius_offset_cm = 1.65;

  inline constexpr double k_gyro_rotation_observability_degps = 6.0;

  //value below is for  1D kalman filter.
  inline constexpr double k_rotation_process_variance_min_urad2 = 100.0;
  inline constexpr double k_rotation_process_variance_max_urad2 = 1000000.0;

  inline constexpr double k_rotation_measurement_variance_min_urad2 = 100.0;
  inline constexpr double k_rotation_measurement_variance_max_urad2 = 1000000.0;

}
