#pragma once

#include <cstddef>
#include <cstdint>

#include "core/middleware/incoming_payloads/service/clear_imu_calibration_payload.hpp"
#include "core/middleware/incoming_payloads/debug/debug_stream_control_payload.hpp"
#include "core/middleware/incoming_payloads/service/drive_forward_payload.hpp"
#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"
#include "core/middleware/incoming_payloads/service/lock_safe_guard_payload.hpp"
#include "core/middleware/incoming_payloads/runtime/motion_command_payload.hpp"
#include "core/middleware/incoming_payloads/service/obstacle_safety_control_payload.hpp"
#include "core/middleware/incoming_payloads/service/pause_payload.hpp"
#include "core/middleware/incoming_payloads/service/position_correction_payload.hpp"
#include "core/middleware/incoming_payloads/service/rotate_delta_payload.hpp"
#include "core/middleware/incoming_payloads/service/start_imu_calibration_payload.hpp"
#include "core/middleware/incoming_payloads/service/trailer_status_payload.hpp"
#include "core/middleware/incoming_payloads/service/unlock_safe_guard_payload.hpp"
#include "core/middleware/outgoing_payloads/debug/encoder_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/debug/imu_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/debug/local_position_model_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/debug/motion_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/runtime/local_position_payload.hpp"
#include "core/middleware/outgoing_payloads/debug/obstacle_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/outgoing_payload_definition.hpp"
#include "core/middleware/outgoing_payloads/runtime/power_status_payload.hpp"
#include "core/middleware/outgoing_payloads/runtime/safety_status_payload.hpp"
#include "core/middleware/outgoing_payloads/debug/voltage_debug_payload.hpp"

namespace middleware_routes
{
  struct outgoing_route_definition
  {
    const char *name;
    std::uint8_t payload_id;
    const middleware_outgoing_payloads::outgoing_payload_definition *payload;
    std::uint32_t period_ms;
    std::uint32_t phase_offset_ms;
    bool enabled_by_default;
  };

  inline const outgoing_route_definition outgoing_routes[] = {
    { "local_position", 0x01U, &middleware_outgoing_payloads::local_position_payload_definition, 10u, 0u, true },
    { "safety_status", 0x02U, &middleware_outgoing_payloads::safety_status_payload_definition, 10u, 5u, true },
    { "power_status", 0x03U, &middleware_outgoing_payloads::power_status_payload_definition, 5000u, 0u, true },
    { "encoder_debug", 0x11U, &middleware_outgoing_payloads::encoder_debug_payload_definition, 100u, 0u, false },
    { "imu_debug", 0x12U, &middleware_outgoing_payloads::imu_debug_payload_definition, 100u, 25u, false },
    { "obstacle_debug", 0x13U, &middleware_outgoing_payloads::obstacle_debug_payload_definition, 100u, 50u, false },
    { "voltage_debug", 0x14U, &middleware_outgoing_payloads::voltage_debug_payload_definition, 100u, 75u, false },
    { "local_position_model_debug", 0x1EU, &middleware_outgoing_payloads::local_position_model_debug_payload_definition, 100u, 85u, false },
    { "motion_debug", 0x1DU, &middleware_outgoing_payloads::motion_debug_payload_definition, 100u, 80u, false },
  };

  inline constexpr std::size_t outgoing_route_count = sizeof(outgoing_routes) / sizeof(outgoing_routes[0]);

  struct incoming_route_registration
  {
    const char *name;
    std::uint8_t payload_id;
    const middleware_incoming_payloads::incoming_payload_definition *payload;
  };

  inline const incoming_route_registration incoming_routes[] = {
    { "motion_command", 0x21U, &middleware_incoming_payloads::motion_command_payload_definition },
    { "start_imu_calibration", 0x22U, &middleware_incoming_payloads::start_imu_calibration_payload_definition },
    { "clear_imu_calibration", 0x23U, &middleware_incoming_payloads::clear_imu_calibration_payload_definition },
    { "debug_stream_control", 0x24U, &middleware_incoming_payloads::debug_stream_control_payload_definition },
    { "trailer_status", 0x25U, &middleware_incoming_payloads::trailer_status_payload_definition },
    { "unlock_safe_guard", 0x26U, &middleware_incoming_payloads::unlock_safe_guard_payload_definition },
    { "lock_safe_guard", 0x27U, &middleware_incoming_payloads::lock_safe_guard_payload_definition },
    { "obstacle_safety_control", 0x28U, &middleware_incoming_payloads::obstacle_safety_control_payload_definition },
    { "rotate_delta", 0x29U, &middleware_incoming_payloads::rotate_delta_payload_definition },
    { "drive_forward", 0x2AU, &middleware_incoming_payloads::drive_forward_payload_definition },
    { "pause", 0x2BU, &middleware_incoming_payloads::pause_payload_definition },
    { "position_correction", 0x30U, &middleware_incoming_payloads::position_correction_payload_definition },
  };

  inline constexpr std::size_t incoming_route_count = sizeof(incoming_routes) / sizeof(incoming_routes[0]);
}
