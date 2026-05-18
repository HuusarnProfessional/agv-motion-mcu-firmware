#pragma once

#include <array>
#include <cstdint>

namespace collision_tuning
{
  enum class obstacle_sensor_role : std::uint8_t
  {
    front = 0u,
    rear
  };

  struct obstacle_sensor_config
  {
    std::uint8_t sensor_id = 0u;
    obstacle_sensor_role role = obstacle_sensor_role::front;
    bool is_enabled = true;
    bool is_masked = false;
  };

  inline constexpr std::uint8_t k_sensor_count = 2u;

  inline constexpr std::array<obstacle_sensor_config, k_sensor_count> k_sensor_configs = 
  {{
    obstacle_sensor_config{ 0u, obstacle_sensor_role::front, true, false },
    obstacle_sensor_config{ 1u, obstacle_sensor_role::rear, false, false }
  }};

  inline constexpr bool k_require_sensor_coverage = false;
  inline constexpr std::int32_t k_min_approach_speed_for_obstacle_check_mm_s = 100;
  inline constexpr std::uint32_t k_sample_stale_timeout_ms = 250u;
  inline constexpr std::uint32_t k_required_consecutive_hazard_samples = 3u;
  inline constexpr std::uint32_t k_static_margin_mm = 50u;
  inline constexpr std::uint32_t k_total_system_latency_ms = 10u;
  inline constexpr std::uint32_t k_min_brake_mm_s2 = 4000u;

}
