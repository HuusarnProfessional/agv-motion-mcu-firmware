#pragma once

#include <cstdint>

#include "core/api/obstacle_api.hpp"
#include "core/control/collision_prediction/collision_tuning.hpp"

namespace obstacle_sensor_logic
{
  struct input
  {
    std::uint32_t now_ms = 0u;
    collision_tuning::obstacle_sensor_config config = {};
    obstacle_api::obstacle_sample sample = {};
    std::uint32_t approach_speed_mm_s = 0u;
  };

  struct state
  {
    bool has_filtered_distance = false;
    bool has_seen_valid_sample = false;
    std::uint32_t filtered_distance_mm = 0u;
    std::uint32_t last_valid_sample_time_ms = 0u;
    std::uint32_t consecutive_hazard_sample_count = 0u;
  };

  struct output
  {
    bool is_relevant = false;
    bool has_valid_sample = false;
    bool is_sample_fresh = false;
    bool hazard_detected = false;
    std::uint32_t filtered_distance_mm = 0u;
    std::uint32_t required_distance_mm = 0u;
  };

  void reset(state &sensor_state);
  void evaluate(state &sensor_state, const input &evaluation_input, output &out);
}
