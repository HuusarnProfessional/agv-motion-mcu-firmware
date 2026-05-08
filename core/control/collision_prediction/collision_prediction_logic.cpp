#include "core/control/collision_prediction/collision_prediction_logic.hpp"

#include "core/control/collision_prediction/internal/collision_input_builder.hpp"

namespace
{
  std::uint32_t select_approach_speed_mm_s(const collision_input_builder::collision_prediction_input &input, collision_tuning::obstacle_sensor_role role)
  {
    if (role == collision_tuning::obstacle_sensor_role::front)
    {
      return input.motion.front_approach_mm_s;
    }

    return input.motion.rear_approach_mm_s;
  }

  bool role_requires_coverage(const collision_input_builder::collision_prediction_input &input, collision_tuning::obstacle_sensor_role role)
  {
    if (!collision_tuning::k_require_sensor_coverage)
    {
      return false;
    }

    if (!input.obstacle_safety_enabled)
    {
      return false;
    }

    if (input.has_trailer)
    {
      if (role == collision_tuning::obstacle_sensor_role::rear)
      {
        return false;
      }
    }

    return select_approach_speed_mm_s(input, role) > static_cast<std::uint32_t>(collision_tuning::k_min_approach_speed_for_obstacle_check_mm_s);
  }
}

namespace collision_prediction_logic
{
  void reset(state &logic_state)
  {
    logic_state = {};
    vehicle_motion_estimator::reset(logic_state.vehicle_motion_state);

    for (obstacle_sensor_logic::state &sensor_state : logic_state.sensor_states)
    {
      obstacle_sensor_logic::reset(sensor_state);
    }
  }

  void tick(state &logic_state, std::uint32_t now_ms, const collision_prediction::runtime_config &config, result &out)
  {
    collision_input_builder::collision_prediction_input input = {};
    bool front_has_valid_coverage = false;
    bool rear_has_valid_coverage = false;
    collision_input_builder::build(now_ms, logic_state.vehicle_motion_state, config, input);
    out = {};

    for (std::size_t sensor_index = 0u; sensor_index < collision_tuning::k_sensor_count; ++sensor_index)
    {
      const collision_input_builder::sensor_input &sensor_input = input.sensor_inputs[sensor_index];
      obstacle_sensor_logic::input evaluation_input = {};
      obstacle_sensor_logic::output evaluation_output = {};
      evaluation_input.now_ms = input.now_ms;
      evaluation_input.config = sensor_input.config;
      evaluation_input.sample = sensor_input.sample;
      evaluation_input.approach_speed_mm_s = select_approach_speed_mm_s(input, sensor_input.config.role);
      obstacle_sensor_logic::evaluate(logic_state.sensor_states[sensor_index], evaluation_input, evaluation_output);

      if (sensor_input.config.role == collision_tuning::obstacle_sensor_role::front)
      {
        if (sensor_input.config.is_enabled)
        {
          if (!sensor_input.config.is_masked)
          {
            if (evaluation_output.has_valid_sample)
            {
              if (evaluation_output.is_sample_fresh)
              {
                front_has_valid_coverage = true;
              }
            }
          }
        }
      }

      if (sensor_input.config.role == collision_tuning::obstacle_sensor_role::rear)
      {
        if (sensor_input.config.is_enabled)
        {
          if (!sensor_input.config.is_masked)
          {
            if (evaluation_output.has_valid_sample)
            {
              if (evaluation_output.is_sample_fresh)
              {
                rear_has_valid_coverage = true;
              }
            }
          }
        }
      }

      if (evaluation_output.hazard_detected)
      {
        out.collision_blocked = true;
      }
    }

    if (role_requires_coverage(input, collision_tuning::obstacle_sensor_role::front) && !front_has_valid_coverage)
    {
      out.collision_blocked = true;
    }

    if (role_requires_coverage(input, collision_tuning::obstacle_sensor_role::rear) && !rear_has_valid_coverage)
    {
      out.collision_blocked = true;
    }

    logic_state.latest_result = out;
  }
}
