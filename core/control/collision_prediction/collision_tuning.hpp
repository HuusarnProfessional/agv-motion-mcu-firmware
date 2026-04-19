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

  inline constexpr std::array<obstacle_sensor_config, k_sensor_count> k_sensor_configs = {{
    obstacle_sensor_config{ 0u, obstacle_sensor_role::front, true, false },
    obstacle_sensor_config{ 1u, obstacle_sensor_role::rear, true, false }
  }};

  inline constexpr std::int32_t k_velocity_arm_mm_s = 25;
  inline constexpr std::uint32_t k_sample_stale_timeout_ms = 250u;
  inline constexpr std::uint32_t k_static_margin_mm = 150u;
  inline constexpr std::uint32_t k_total_latency_ms = 250u;
  inline constexpr std::uint32_t k_min_brake_mm_s2 = 1000u;

}
