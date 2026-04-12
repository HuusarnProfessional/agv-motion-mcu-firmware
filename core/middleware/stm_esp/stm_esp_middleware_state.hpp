#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/middleware/stm_esp/incoming_payloads/debug_stream_control_payload.hpp"
#include "core/middleware/stm_esp/incoming_payloads/motion_command_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/encoder_debug_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/imu_debug_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/local_position_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/obstacle_debug_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/power_status_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/safety_status_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/voltage_debug_payload.hpp"

namespace stm_esp
{
  enum class incoming_packet_step : std::uint8_t
  {
    wait_for_sync = 0,
    wait_for_payload_id,
    wait_for_payload_length,
    read_payload_bytes
  };

  struct incoming_packet_parser
  {
    incoming_packet_step step = incoming_packet_step::wait_for_sync;
    std::uint8_t payload_id = 0U;
    std::uint8_t payload_length = 0U;
    std::size_t payload_bytes_read = 0U;
    std::array<std::uint8_t, 255U> payload_bytes = {};
  };

  struct middleware_state
  {
    bool is_initialized = false;
    std::uint8_t comm_uart_id = 0U;
    std::uint32_t last_tick_time_ms = 0U;

    incoming_packet_parser parser = {};

    stm_esp_outgoing_payloads::local_position_payload_data local_position = {};
    stm_esp_outgoing_payloads::safety_status_payload_data safety_status = {};
    stm_esp_outgoing_payloads::power_status_payload_data power_status = {};
    stm_esp_outgoing_payloads::encoder_debug_payload_data encoder_debug = {};
    stm_esp_outgoing_payloads::imu_debug_payload_data imu_debug = {};
    stm_esp_outgoing_payloads::obstacle_debug_payload_data obstacle_debug = {};
    stm_esp_outgoing_payloads::voltage_debug_payload_data voltage_debug = {};

    bool has_local_position = false;
    bool has_safety_status = false;
    bool has_power_status = false;
    bool has_encoder_debug = false;
    bool has_imu_debug = false;
    bool has_obstacle_debug = false;
    bool has_voltage_debug = false;

    stm_esp_incoming_payloads::motion_command_payload_data latest_motion_command = {};
    bool has_latest_motion_command = false;

    bool start_imu_calibration_requested = false;
    bool clear_imu_calibration_requested = false;

    stm_esp_incoming_payloads::debug_stream_control_payload_data pending_debug_stream_control = {};
    bool has_pending_debug_stream_control = false;
  };
}
