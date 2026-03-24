#pragma once

#include <cstdint>

namespace imu_calibration_tuning
{
  // Max translation to still count as stationary.
  inline constexpr std::int64_t k_still_translation_limit_um = 500;

  // Max rotation to still count as stationary.
  inline constexpr std::int64_t k_still_rotation_limit_urad = 5000;

  // Motor command used during forward/backward alignment sampling.
  inline constexpr std::int16_t k_drive_u = 600;

  // Max drive distance during one alignment run.
  inline constexpr std::int64_t k_max_translation_um = 500000;

  // Min drive distance before acceleration drop may stop alignment.
  inline constexpr std::int64_t k_min_translation_before_stop_um = 50000;

  // IMU samples collected in one acceleration block.
  inline constexpr std::int64_t k_block_sample_count = 100;

  // Safety limit for drive loop iterations.
  inline constexpr std::uint32_t k_max_drive_tick_count = 200000u;
}
