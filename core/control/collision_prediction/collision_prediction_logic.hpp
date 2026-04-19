#pragma once

#include <array>
#include <cstdint>

#include "core/control/collision_prediction/collision_tuning.hpp"
#include "core/control/collision_prediction/internal/obstacle_sensor_logic.hpp"
#include "core/control/collision_prediction/internal/vehicle_motion_estimator.hpp"

namespace collision_prediction_logic
{
  struct result
  {
    bool collision_blocked = false;
  };

  struct state
  {
    vehicle_motion_estimator::state vehicle_motion_state = {};
    std::array<obstacle_sensor_logic::state, collision_tuning::k_sensor_count> sensor_states = {};
    result latest_result = {};
  };

  void reset(state &logic_state);
  void tick(state &logic_state, std::uint32_t now_ms, result &out);
}
