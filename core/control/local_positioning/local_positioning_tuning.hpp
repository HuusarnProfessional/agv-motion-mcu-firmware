#pragma once

#include <cstdint>

namespace local_positioning_tuning
{
  inline constexpr std::uint32_t k_position_uncertainty_update_divider = 100u;
  inline constexpr std::uint32_t k_heading_uncertainty_update_divider = 100u;

  inline constexpr std::int32_t k_position_no_motion_translation_um = 5;
  inline constexpr std::int32_t k_heading_no_motion_rotation_urad = 20;

  inline constexpr std::int32_t k_position_reference_translation_um = 1000;
  inline constexpr std::int32_t k_heading_reference_rotation_urad = 10000;

  inline constexpr std::uint32_t k_uncertainty_scale_max = 1000u;
}
