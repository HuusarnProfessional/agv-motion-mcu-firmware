#pragma once

#include <cstdint>

namespace imu_calibration_tuning
{
  inline constexpr std::int64_t k_still_translation_limit_um = 500;
  inline constexpr std::int64_t k_still_rotation_limit_urad = 5000;
  inline constexpr std::int16_t k_drive_u = 600;
  inline constexpr std::int64_t k_max_translation_um = 500000;
  inline constexpr std::int64_t k_min_translation_before_stop_um = 50000;
  inline constexpr std::int64_t k_block_sample_count = 100;
  inline constexpr std::uint32_t k_max_drive_tick_count = 200000u;
  inline constexpr std::uint32_t k_noise_p_proc_percent = 95u;
}
