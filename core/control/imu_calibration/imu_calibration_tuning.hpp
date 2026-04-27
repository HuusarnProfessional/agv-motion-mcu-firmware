#pragma once

#include <cstdint>

namespace imu_calibration_tuning
{
  inline constexpr std::int64_t k_still_translation_limit_um = 500;
  inline constexpr std::int64_t k_still_rotation_limit_urad = 5000;
  inline constexpr std::int16_t k_drive_u = 600;
  inline constexpr std::int64_t k_max_translation_um = 500000;
  inline constexpr std::int64_t k_min_translation_before_stop_um = 50000;
  inline constexpr std::uint32_t k_max_drive_tick_count = 200000U;
  inline constexpr std::uint32_t k_tare_target_sample_count = 500U;
  inline constexpr std::uint32_t k_tare_max_tick_count = 5000U;
  inline constexpr std::uint32_t k_drive_min_sample_count = 100U;
  inline constexpr std::int32_t k_min_alignment_axis_delta_raw = 100;
  inline constexpr std::uint32_t k_noise_p_proc_percent = 95U;
}
