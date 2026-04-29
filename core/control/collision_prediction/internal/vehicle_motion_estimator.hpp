#pragma once

#include <cstdint>

#include "core/control/local_positioning/local_positioning.hpp"
#include "core/middleware/incoming_payloads/runtime/motion_command_payload.hpp"

namespace vehicle_motion_estimator
{
  struct input
  {
    std::uint32_t now_ms = 0u;
    local_positioning::snapshot local_positioning_snapshot = {};
    bool has_motion_command = false;
    middleware_incoming_payloads::motion_command_payload_data motion_command = {};
  };

  struct state
  {
    bool has_previous_pose = false;
    local_positioning::snapshot previous_local_positioning_snapshot = {};
    std::uint32_t previous_time_ms = 0u;
  };

  struct output
  {
    bool has_measured_forward_velocity = false;
    std::int32_t measured_forward_mm_s = 0;
    std::int32_t commanded_forward_mm_s = 0;
    std::uint32_t front_approach_mm_s = 0u;
    std::uint32_t rear_approach_mm_s = 0u;
  };

  void reset(state &estimator_state);
  void estimate(state &estimator_state, const input &estimation_input, output &out);
}
