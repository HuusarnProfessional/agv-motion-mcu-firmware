#pragma once

#include <cstdint>

namespace motion_primitives_tuning
{
  constexpr std::uint32_t k_default_drive_forward_timeout_ms = 30000u;
  constexpr std::uint32_t k_default_rotate_delta_timeout_ms = 30000u;
  constexpr std::uint32_t k_settling_max_duration_ms = 2000u;
  constexpr std::uint32_t k_settling_still_duration_ms = 300u;
  constexpr std::int64_t k_settling_encoder_translation_threshold_um = 100;
  constexpr std::int64_t k_settling_imu_translation_threshold_um = 2000;
  constexpr std::int64_t k_settling_imu_yaw_threshold_urad = 2000;
}
