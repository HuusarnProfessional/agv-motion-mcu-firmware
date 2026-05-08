#pragma once

#include <cstdint>

namespace imu_model_tuning
{
  struct input_storage_tuning
  {
  };

  struct stationary_detection_tuning
  {
  };

  struct delta_estimation_tuning
  {
  };

  struct confidence_estimation_tuning
  {
  };

  struct motion_model_tuning
  {
  };

  inline constexpr std::uint32_t k_stationary_noise_scale_percent = 120u;
  inline constexpr std::uint32_t k_stationary_low_pass_alpha_percent = 20u;
  inline constexpr std::int32_t k_stationary_gyro_z_min_limit_mdps = 3000;
  inline constexpr std::int32_t k_stationary_acc_x_min_limit_mg = 50;
  inline constexpr std::int32_t k_stationary_acc_y_min_limit_mg = 50;
  inline constexpr std::int64_t k_motion_confidence_max = 1000;
  inline constexpr std::int32_t k_motion_gyro_z_noise_floor_mdps = 1;
  inline constexpr std::int32_t k_motion_acc_x_noise_floor_mg = 1;
  inline constexpr std::int64_t k_stationary_motion_confidence = 900;
  inline constexpr std::int64_t k_min_active_motion_confidence = 1;
  inline constexpr std::int64_t k_um_per_mg_per_s2 = 9807;
}
