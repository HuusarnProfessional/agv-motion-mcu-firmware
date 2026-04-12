#include "core/middleware/stm_esp_middleware_pipeline.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/api/comm_uart_api.hpp"
#include "core/middleware/stm_esp_middleware_streams.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  constexpr std::uint8_t k_sync_byte = 0xA5U;
  constexpr std::size_t k_max_payload_length = 255U;
  constexpr std::size_t k_max_packet_length = k_max_payload_length + 3U;
  constexpr std::size_t k_read_chunk_length = 64U;

  struct outgoing_stream_runtime
  {
    bool enabled = false;
    std::uint32_t next_send_time_ms = 0U;
  };

  enum class send_result : std::uint8_t
  {
    sent = 0,
    skipped,
    failed
  };

  stm_esp::middleware_state g_state = {};
  std::array<outgoing_stream_runtime, stm_esp_middleware_streams::outgoing_stream_count> g_outgoing_stream_runtime = {};

  bool has_time_elapsed(std::uint32_t now_ms, std::uint32_t target_time_ms)
  {
    return static_cast<std::int32_t>(now_ms - target_time_ms) >= 0;
  }

  void reset_parser(stm_esp::incoming_packet_parser &parser)
  {
    parser.step = stm_esp::incoming_packet_step::wait_for_sync;
    parser.payload_id = 0U;
    parser.payload_length = 0U;
    parser.payload_bytes_read = 0U;
  }

  void advance_stream_deadline(outgoing_stream_runtime &runtime, std::uint32_t period_ms, std::uint32_t now_ms)
  {
    if (period_ms == 0U)
    {
      runtime.next_send_time_ms = now_ms;
      return;
    }

    do
    {
      runtime.next_send_time_ms += period_ms;
    }
    while (has_time_elapsed(now_ms, runtime.next_send_time_ms));
  }

  bool apply_incoming_payload(std::uint8_t payload_id, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    for (std::size_t payload_index = 0U; payload_index < stm_esp_middleware_streams::incoming_payload_count; ++payload_index)
    {
      const stm_esp_incoming_payloads::incoming_payload_definition *payload = stm_esp_middleware_streams::incoming_payloads[payload_index];

      if (payload == nullptr || payload->payload_id != payload_id || payload->apply_payload_bytes == nullptr)
      {
        continue;
      }

      return payload->apply_payload_bytes(g_state, payload_data, payload_length, received_time_ms);
    }

    return false;
  }

  void process_incoming_byte(std::uint8_t incoming_byte, std::uint32_t now_ms)
  {
    switch (g_state.parser.step)
    {
    case stm_esp::incoming_packet_step::wait_for_sync:
      if (incoming_byte == k_sync_byte)
      {
        g_state.parser.step = stm_esp::incoming_packet_step::wait_for_payload_id;
      }
      break;

    case stm_esp::incoming_packet_step::wait_for_payload_id:
      g_state.parser.payload_id = incoming_byte;
      g_state.parser.step = stm_esp::incoming_packet_step::wait_for_payload_length;
      break;

    case stm_esp::incoming_packet_step::wait_for_payload_length:
      g_state.parser.payload_length = incoming_byte;
      g_state.parser.payload_bytes_read = 0U;

      if (g_state.parser.payload_length == 0U)
      {
        apply_incoming_payload(g_state.parser.payload_id, nullptr, 0U, now_ms);
        reset_parser(g_state.parser);
      }
      else
      {
        g_state.parser.step = stm_esp::incoming_packet_step::read_payload_bytes;
      }
      break;

    case stm_esp::incoming_packet_step::read_payload_bytes:
      if (g_state.parser.payload_bytes_read >= g_state.parser.payload_bytes.size())
      {
        reset_parser(g_state.parser);
        break;
      }

      g_state.parser.payload_bytes[g_state.parser.payload_bytes_read++] = incoming_byte;

      if (g_state.parser.payload_bytes_read == g_state.parser.payload_length)
      {
        apply_incoming_payload(g_state.parser.payload_id, g_state.parser.payload_bytes.data(), g_state.parser.payload_length, now_ms);
        reset_parser(g_state.parser);
      }
      break;
    }
  }

  void read_incoming_packets(std::uint32_t now_ms)
  {
    std::array<std::uint8_t, k_read_chunk_length> incoming_bytes = {};

    while (true)
    {
      const std::size_t bytes_read = comm_uart_api::read_bytes(g_state.comm_uart_id, incoming_bytes.data(), incoming_bytes.size());

      if (bytes_read == 0U)
      {
        return;
      }

      for (std::size_t byte_index = 0U; byte_index < bytes_read; ++byte_index)
      {
        process_incoming_byte(incoming_bytes[byte_index], now_ms);
      }
    }
  }

  send_result send_stream_payload(const stm_esp_middleware_streams::outgoing_stream_definition &stream)
  {
    std::array<std::uint8_t, k_max_payload_length> payload_bytes = {};
    std::array<std::uint8_t, k_max_packet_length> packet_bytes = {};
    std::size_t payload_length = 0U;

    if (stream.payload == nullptr || stream.payload->build_payload_bytes == nullptr)
    {
      return send_result::failed;
    }

    if (!stream.payload->build_payload_bytes(g_state, payload_bytes.data(), payload_bytes.size(), payload_length))
    {
      return send_result::skipped;
    }

    if (payload_length > k_max_payload_length)
    {
      return send_result::failed;
    }

    packet_bytes[0] = k_sync_byte;
    packet_bytes[1] = stream.payload->payload_id;
    packet_bytes[2] = static_cast<std::uint8_t>(payload_length);

    for (std::size_t byte_index = 0U; byte_index < payload_length; ++byte_index)
    {
      packet_bytes[3U + byte_index] = payload_bytes[byte_index];
    }

    const comm_uart_api::comm_uart_status status = comm_uart_api::write_bytes(g_state.comm_uart_id, packet_bytes.data(), payload_length + 3U);
    return (status == comm_uart_api::comm_uart_status::ok) ? send_result::sent : send_result::failed;
  }
}

namespace stm_esp_middleware_pipeline
{
  void init(std::uint8_t comm_uart_id)
  {
    g_state = {};
    g_state.is_initialized = true;
    g_state.comm_uart_id = comm_uart_id;

    reset_parser(g_state.parser);

    for (std::size_t stream_index = 0U; stream_index < stm_esp_middleware_streams::outgoing_stream_count; ++stream_index)
    {
      g_outgoing_stream_runtime[stream_index].enabled = stm_esp_middleware_streams::outgoing_streams[stream_index].enabled_by_default;
      g_outgoing_stream_runtime[stream_index].next_send_time_ms = stm_esp_middleware_streams::outgoing_streams[stream_index].phase_offset_ms;
    }
  }

  void tick(std::uint32_t now_ms)
  {
    if (!g_state.is_initialized)
    {
      return;
    }

    g_state.last_tick_time_ms = now_ms;

    read_incoming_packets(now_ms);

    if (g_state.has_pending_debug_stream_control)
    {
      set_stream_enabled(g_state.pending_debug_stream_control.target_payload_id, g_state.pending_debug_stream_control.is_enabled);
      g_state.has_pending_debug_stream_control = false;
      g_state.pending_debug_stream_control = {};
    }

    for (std::size_t stream_index = 0U; stream_index < stm_esp_middleware_streams::outgoing_stream_count; ++stream_index)
    {
      outgoing_stream_runtime &runtime = g_outgoing_stream_runtime[stream_index];
      const stm_esp_middleware_streams::outgoing_stream_definition &stream = stm_esp_middleware_streams::outgoing_streams[stream_index];

      if (!runtime.enabled || !has_time_elapsed(now_ms, runtime.next_send_time_ms))
      {
        continue;
      }

      const send_result result = send_stream_payload(stream);

      if (result == send_result::failed)
      {
        break;
      }

      advance_stream_deadline(runtime, stream.period_ms, now_ms);
    }
  }

  void set_local_position(const stm_esp_outgoing_payloads::local_position_payload_data &local_position)
  {
    g_state.local_position = local_position;
    g_state.has_local_position = true;
  }

  void set_safety_status(const stm_esp_outgoing_payloads::safety_status_payload_data &safety_status)
  {
    g_state.safety_status = safety_status;
    g_state.has_safety_status = true;
  }

  void set_power_status(const stm_esp_outgoing_payloads::power_status_payload_data &power_status)
  {
    g_state.power_status = power_status;
    g_state.has_power_status = true;
  }

  void set_encoder_debug(const stm_esp_outgoing_payloads::encoder_debug_payload_data &encoder_debug)
  {
    g_state.encoder_debug = encoder_debug;
    g_state.has_encoder_debug = true;
  }

  void set_imu_debug(const stm_esp_outgoing_payloads::imu_debug_payload_data &imu_debug)
  {
    g_state.imu_debug = imu_debug;
    g_state.has_imu_debug = true;
  }

  void set_obstacle_debug(const stm_esp_outgoing_payloads::obstacle_debug_payload_data &obstacle_debug)
  {
    g_state.obstacle_debug = obstacle_debug;
    g_state.has_obstacle_debug = true;
  }

  void set_voltage_debug(const stm_esp_outgoing_payloads::voltage_debug_payload_data &voltage_debug)
  {
    g_state.voltage_debug = voltage_debug;
    g_state.has_voltage_debug = true;
  }

  bool read_latest_motion_command(stm_esp_incoming_payloads::motion_command_payload_data &out)
  {
    if (!g_state.has_latest_motion_command)
    {
      return false;
    }

    out = g_state.latest_motion_command;
    return true;
  }

  bool consume_start_imu_calibration_request(void)
  {
    const bool has_request = g_state.start_imu_calibration_requested;
    g_state.start_imu_calibration_requested = false;
    return has_request;
  }

  bool consume_clear_imu_calibration_request(void)
  {
    const bool has_request = g_state.clear_imu_calibration_requested;
    g_state.clear_imu_calibration_requested = false;
    return has_request;
  }

  bool set_stream_enabled(std::uint8_t payload_id, bool is_enabled)
  {
    bool found_stream = false;

    for (std::size_t stream_index = 0U; stream_index < stm_esp_middleware_streams::outgoing_stream_count; ++stream_index)
    {
      if (stm_esp_middleware_streams::outgoing_streams[stream_index].payload == nullptr ||
          stm_esp_middleware_streams::outgoing_streams[stream_index].payload->payload_id != payload_id)
      {
        continue;
      }

      g_outgoing_stream_runtime[stream_index].enabled = is_enabled;

      if (is_enabled)
      {
        g_outgoing_stream_runtime[stream_index].next_send_time_ms = g_state.last_tick_time_ms;
      }

      found_stream = true;
    }

    return found_stream;
  }

  bool read_stream_enabled(std::uint8_t payload_id, bool &is_enabled_out)
  {
    for (std::size_t stream_index = 0U; stream_index < stm_esp_middleware_streams::outgoing_stream_count; ++stream_index)
    {
      if (stm_esp_middleware_streams::outgoing_streams[stream_index].payload == nullptr ||
          stm_esp_middleware_streams::outgoing_streams[stream_index].payload->payload_id != payload_id)
      {
        continue;
      }

      is_enabled_out = g_outgoing_stream_runtime[stream_index].enabled;
      return true;
    }

    return false;
  }
}
