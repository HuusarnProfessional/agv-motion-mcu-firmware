#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/incoming_payloads/motion_command_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/encoder_debug_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/imu_debug_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/local_position_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/obstacle_debug_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/power_status_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/safety_status_payload.hpp"
#include "core/middleware/stm_esp/outgoing_payloads/voltage_debug_payload.hpp"

namespace stm_esp_middleware_pipeline
{
  void init(std::uint8_t comm_uart_id);
  void tick(std::uint32_t now_ms);

  void set_local_position(const stm_esp_outgoing_payloads::local_position_payload_data &local_position);
  void set_safety_status(const stm_esp_outgoing_payloads::safety_status_payload_data &safety_status);
  void set_power_status(const stm_esp_outgoing_payloads::power_status_payload_data &power_status);
  void set_encoder_debug(const stm_esp_outgoing_payloads::encoder_debug_payload_data &encoder_debug);
  void set_imu_debug(const stm_esp_outgoing_payloads::imu_debug_payload_data &imu_debug);
  void set_obstacle_debug(const stm_esp_outgoing_payloads::obstacle_debug_payload_data &obstacle_debug);
  void set_voltage_debug(const stm_esp_outgoing_payloads::voltage_debug_payload_data &voltage_debug);

  bool read_latest_motion_command(stm_esp_incoming_payloads::motion_command_payload_data &out);
  bool consume_start_imu_calibration_request(void);
  bool consume_clear_imu_calibration_request(void);

  bool set_stream_enabled(std::uint8_t payload_id, bool is_enabled);
  bool read_stream_enabled(std::uint8_t payload_id, bool &is_enabled_out);
}
