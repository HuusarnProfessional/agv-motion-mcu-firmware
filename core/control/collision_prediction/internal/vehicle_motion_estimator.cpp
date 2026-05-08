#include "core/control/collision_prediction/internal/vehicle_motion_estimator.hpp"
#include "core/control/collision_prediction/collision_tuning.hpp"
#include "core/control/drive_control/wheel_drive_controller_tuning.hpp"

#include <cmath>
#include <limits>

namespace
{
  std::int32_t clamp_to_i32(double value)
  {
    if (value > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
    {
      return std::numeric_limits<std::int32_t>::max();
    }

    if (value < static_cast<double>(std::numeric_limits<std::int32_t>::min()))
    {
      return std::numeric_limits<std::int32_t>::min();
    }

    return static_cast<std::int32_t>(std::lround(value));
  }

  bool is_motion_command_fresh(const vehicle_motion_estimator::input &estimation_input)
  {
    if (!estimation_input.has_motion_command)
    {
      return false;
    }

    if (estimation_input.now_ms < estimation_input.motion_command.received_time_ms)
    {
      return false;
    }

    if (estimation_input.now_ms - estimation_input.motion_command.received_time_ms > wheel_drive_controller_tuning::k_motion_command_timeout_ms)
    {
      return false;
    }

    return true;
  }

  std::int32_t compute_commanded_forward_mm_s(const vehicle_motion_estimator::input &estimation_input)
  {
    if (!is_motion_command_fresh(estimation_input))
    {
      return 0;
    }

    if (!estimation_input.motion_command.drive_enabled)
    {
      return 0;
    }

    return estimation_input.motion_command.linear_velocity_mm_s;
  }

  bool can_estimate_measured_velocity(const vehicle_motion_estimator::state &estimator_state, const vehicle_motion_estimator::input &estimation_input)
  {
    return estimator_state.has_previous_pose && estimator_state.previous_local_positioning_snapshot.has_pose && estimation_input.local_positioning_snapshot.has_pose && estimation_input.now_ms > estimator_state.previous_time_ms;
  }

  std::int32_t compute_measured_forward_mm_s(const vehicle_motion_estimator::state &estimator_state, const vehicle_motion_estimator::input &estimation_input)
  {
    const std::int64_t dx_um = estimation_input.local_positioning_snapshot.x_um - estimator_state.previous_local_positioning_snapshot.x_um;
    const std::int64_t dy_um = estimation_input.local_positioning_snapshot.y_um - estimator_state.previous_local_positioning_snapshot.y_um;
    const double heading_rad = static_cast<double>(estimator_state.previous_local_positioning_snapshot.heading_urad) / 1000000.0;
    const double forward_axis_x = std::cos(heading_rad);
    const double forward_axis_y = std::sin(heading_rad);
    const double forward_delta_um = static_cast<double>(dx_um) * forward_axis_x + static_cast<double>(dy_um) * forward_axis_y;
    const std::uint32_t dt_ms = estimation_input.now_ms - estimator_state.previous_time_ms;

    if (dt_ms == 0u)
    {
      return 0;
    }

    return clamp_to_i32(forward_delta_um / static_cast<double>(dt_ms));
  }

  std::uint32_t positive_component(std::int32_t value)
  {
    if (value <= 0)
    {
      return 0u;
    }

    return static_cast<std::uint32_t>(value);
  }

  std::uint32_t negative_component(std::int32_t value)
  {
    if (value >= 0)
    {
      return 0u;
    }

    return static_cast<std::uint32_t>(-value);
  }
}

namespace vehicle_motion_estimator
{
  void reset(state &estimator_state)
  {
    estimator_state = {};
  }

  void estimate(state &estimator_state, const input &estimation_input, output &out)
  {
    out = {};
    out.commanded_forward_mm_s = compute_commanded_forward_mm_s(estimation_input);

    if (can_estimate_measured_velocity(estimator_state, estimation_input))
    {
      out.measured_forward_mm_s = compute_measured_forward_mm_s(estimator_state, estimation_input);
      out.has_measured_forward_velocity = true;
    }

    out.front_approach_mm_s = positive_component(out.commanded_forward_mm_s);
    out.rear_approach_mm_s = negative_component(out.commanded_forward_mm_s);

    if (positive_component(out.measured_forward_mm_s) > out.front_approach_mm_s)
    {
      out.front_approach_mm_s = positive_component(out.measured_forward_mm_s);
    }

    if (negative_component(out.measured_forward_mm_s) > out.rear_approach_mm_s)
    {
      out.rear_approach_mm_s = negative_component(out.measured_forward_mm_s);
    }

    if (positive_component(out.measured_forward_mm_s) <= static_cast<std::uint32_t>(collision_tuning::k_min_approach_speed_for_obstacle_check_mm_s))
    {
      out.front_approach_mm_s = 0u;
    }

    if (negative_component(out.measured_forward_mm_s) <= static_cast<std::uint32_t>(collision_tuning::k_min_approach_speed_for_obstacle_check_mm_s))
    {
      out.rear_approach_mm_s = 0u;
    }

    estimator_state.previous_local_positioning_snapshot = estimation_input.local_positioning_snapshot;
    estimator_state.previous_time_ms = estimation_input.now_ms;
    estimator_state.has_previous_pose = estimation_input.local_positioning_snapshot.has_pose;
  }
}
