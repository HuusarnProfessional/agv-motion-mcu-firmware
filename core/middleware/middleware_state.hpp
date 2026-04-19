#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/middleware/incoming_payloads/debug_stream_control_payload.hpp"

namespace middleware
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
    std::uint8_t comm_uart_id = 0u;
    std::uint32_t last_tick_time_ms = 0u;
    bool has_trailer = false;

    incoming_packet_parser parser = {};

    middleware_incoming_payloads::debug_stream_control_payload_data pending_debug_stream_control = {};
    bool has_pending_debug_stream_control = false;
  };
}
