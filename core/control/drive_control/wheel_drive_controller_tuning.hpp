#pragma once

#include <cstdint>

namespace wheel_drive_controller_tuning
{
  inline constexpr std::uint8_t k_wheel_count = 4u;
  inline constexpr std::uint32_t k_motion_command_timeout_ms = 250u;
  inline constexpr std::uint32_t k_feedback_not_ready_grace_ms = 300u;
  inline constexpr std::uint32_t k_encoder_feedback_stale_timeout_ms = 40u;
  inline constexpr std::uint32_t k_pose_feedback_stale_timeout_ms = 100u;
  inline constexpr std::int32_t k_drive_pwm_scale_percent = 105;
  inline constexpr std::int32_t k_min_drive_u = 225;
  inline constexpr std::int32_t k_startup_drive_u = 325;
  inline constexpr std::int32_t k_max_drive_u = 1000;
  inline constexpr std::int32_t k_feedforward_per_mm_s = 2;
  inline constexpr std::int32_t k_feedback_per_mm_s = 1;
  inline constexpr std::int32_t k_integral_divisor = 200;
  inline constexpr std::int64_t k_integral_limit_mm_s_ms = 160000;
  inline constexpr std::int32_t k_speed_deadband_mm_s = 10;
  inline constexpr std::int32_t k_outer_yaw_feedback_per_mdeg_s_divisor = 2;
  inline constexpr std::int32_t k_outer_yaw_rate_limit_mdeg_s = 60000;
  inline constexpr std::uint16_t k_outer_yaw_min_heading_confidence = 300u;
}
