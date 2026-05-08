#pragma once

#include <cstdint>

namespace motion_primitives_tuning
{
  constexpr std::uint32_t k_start_pose_wait_timeout_ms = 1000u;
  constexpr std::uint32_t k_default_drive_forward_timeout_ms = 30000u;
  constexpr std::uint32_t k_default_rotate_delta_timeout_ms = 30000u;
  constexpr std::uint32_t k_pose_freshness_timeout_ms = 1000u;
  constexpr std::int64_t k_drive_forward_target_tolerance_um = 5000;
  constexpr std::int64_t k_drive_forward_final_window_um = 100000;
  constexpr std::int64_t k_drive_forward_settle_translation_threshold_um = 500;
  constexpr std::int32_t k_drive_forward_min_velocity_mm_s = 20;
  constexpr std::int32_t k_drive_forward_final_max_velocity_mm_s = 40;
  constexpr std::int32_t k_drive_forward_p_um_per_s_divisor = 4000;
  constexpr std::int64_t k_rotate_delta_target_tolerance_urad = 35000;
  constexpr std::int64_t k_rotate_delta_final_window_urad = 261799;
  constexpr std::int32_t k_rotate_delta_min_request_yaw_rate_mdeg_s = 45000;
  constexpr std::int32_t k_rotate_delta_min_yaw_rate_mdeg_s = 45000;
  constexpr std::int32_t k_rotate_delta_final_max_yaw_rate_mdeg_s = 30000;
  constexpr std::int32_t k_rotate_delta_settle_yaw_rate_mdps = 30000;
  constexpr std::int32_t k_rotate_delta_p_divisor = 1;
  constexpr std::int32_t k_rotate_delta_d_divisor = 1;
  constexpr std::int32_t k_rotate_delta_default_rotation_min_drive_u = 350;
  constexpr std::int32_t k_rotate_delta_default_rotation_startup_drive_u = 500;
  constexpr std::uint32_t k_settling_max_duration_ms = 2000u;
  constexpr std::uint32_t k_settling_still_duration_ms = 300u;
  constexpr std::int64_t k_settling_encoder_translation_threshold_um = 100;
  constexpr std::int64_t k_settling_imu_translation_threshold_um = 2000;
  constexpr std::int64_t k_settling_imu_yaw_threshold_urad = 2000;
}
