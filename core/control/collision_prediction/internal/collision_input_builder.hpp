#pragma once

#include <array>
#include <cstdint>

#include "core/api/obstacle_api.hpp"
#include "core/control/collision_prediction/collision_tuning.hpp"
#include "core/control/collision_prediction/internal/vehicle_motion_estimator.hpp"

namespace collision_input_builder
{
  struct sensor_input
  {
    collision_tuning::obstacle_sensor_config config = {};
    obstacle_api::obstacle_sample sample = {};
  };

  struct collision_prediction_input
  {
    std::uint32_t now_ms = 0u;
    bool has_trailer = false;
    vehicle_motion_estimator::output motion = {};
    std::array<sensor_input, collision_tuning::k_sensor_count> sensor_inputs = {};
  };

  void build(std::uint32_t now_ms, vehicle_motion_estimator::state &motion_state, collision_prediction_input &out);
}
