#pragma once

#include <cstddef>
#include <cstdint>

#include "core/middleware/incoming_payloads/clear_imu_calibration_payload.hpp"
#include "core/middleware/incoming_payloads/debug_stream_control_payload.hpp"
#include "core/middleware/incoming_payloads/incoming_payload_definition.hpp"
#include "core/middleware/incoming_payloads/motion_command_payload.hpp"
#include "core/middleware/incoming_payloads/start_imu_calibration_payload.hpp"
#include "core/middleware/outgoing_payloads/encoder_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/imu_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/local_position_payload.hpp"
#include "core/middleware/outgoing_payloads/obstacle_debug_payload.hpp"
#include "core/middleware/outgoing_payloads/outgoing_payload_definition.hpp"
#include "core/middleware/outgoing_payloads/power_status_payload.hpp"
#include "core/middleware/outgoing_payloads/safety_status_payload.hpp"
#include "core/middleware/outgoing_payloads/voltage_debug_payload.hpp"

namespace middleware_streams
{
  struct outgoing_stream_definition
  {
    const char *name;
    const middleware_outgoing_payloads::outgoing_payload_definition *payload;
    std::uint32_t period_ms;
    std::uint32_t phase_offset_ms;
    bool enabled_by_default;
  };

  inline const outgoing_stream_definition outgoing_streams[] = {
    { "local_position", &middleware_outgoing_payloads::local_position_payload_definition, 10U, 0U, true },
    { "safety_status", &middleware_outgoing_payloads::safety_status_payload_definition, 10U, 5U, true },
    { "power_status", &middleware_outgoing_payloads::power_status_payload_definition, 5000U, 0U, true },
    { "encoder_debug", &middleware_outgoing_payloads::encoder_debug_payload_definition, 100U, 0U, false },
    { "imu_debug", &middleware_outgoing_payloads::imu_debug_payload_definition, 100U, 25U, false },
    { "obstacle_debug", &middleware_outgoing_payloads::obstacle_debug_payload_definition, 100U, 50U, false },
    { "voltage_debug", &middleware_outgoing_payloads::voltage_debug_payload_definition, 100U, 75U, false },
  };

  inline constexpr std::size_t outgoing_stream_count = sizeof(outgoing_streams) / sizeof(outgoing_streams[0]);

  inline const middleware_incoming_payloads::incoming_payload_definition *incoming_payloads[] = {
    &middleware_incoming_payloads::motion_command_payload_definition,
    &middleware_incoming_payloads::start_imu_calibration_payload_definition,
    &middleware_incoming_payloads::clear_imu_calibration_payload_definition,
    &middleware_incoming_payloads::debug_stream_control_payload_definition,
  };

  inline constexpr std::size_t incoming_payload_count = sizeof(incoming_payloads) / sizeof(incoming_payloads[0]);
}
