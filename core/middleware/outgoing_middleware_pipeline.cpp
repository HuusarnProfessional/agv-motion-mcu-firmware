#include "core/middleware/outgoing_middleware_pipeline.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/api/comm_uart_api.hpp"
#include "core/middleware/middleware_streams.hpp"
#include "core/middleware/middleware_shared_state.hpp"

namespace
{
  constexpr std::uint8_t k_sync_byte = 0xA5U;
  constexpr std::size_t k_max_payload_length = 255U;
  constexpr std::size_t k_max_packet_length = k_max_payload_length + 3U;

  enum class send_result : std::uint8_t
  {
    sent = 0,
    skipped,
    failed
  };

  bool has_time_elapsed(std::uint32_t now_ms, std::uint32_t target_time_ms)
  {
    return static_cast<std::int32_t>(now_ms - target_time_ms) >= 0;
  }

  void advance_stream_deadline(middleware::outgoing_stream_runtime &runtime, std::uint32_t period_ms, std::uint32_t now_ms)
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

  send_result send_stream_payload(const middleware_streams::outgoing_stream_definition &stream)
  {
    std::array<std::uint8_t, k_max_payload_length> payload_bytes = {};
    std::array<std::uint8_t, k_max_packet_length> packet_bytes = {};
    std::size_t payload_length = 0U;

    if (stream.payload == nullptr || stream.payload->build_payload_bytes == nullptr)
    {
      return send_result::failed;
    }

    if (!stream.payload->build_payload_bytes(middleware::g_middleware_state, payload_bytes.data(), payload_bytes.size(), payload_length))
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

    const comm_uart_api::comm_uart_status status = comm_uart_api::write_bytes(middleware::g_middleware_state.comm_uart_id, packet_bytes.data(), payload_length + 3U);

    if (status == comm_uart_api::comm_uart_status::ok)
    {
      return send_result::sent;
    }

    return send_result::failed;
  }
}

namespace outgoing_middleware_pipeline
{
  void init(void)
  {
    for (std::size_t stream_index = 0U; stream_index < middleware_streams::outgoing_stream_count; ++stream_index)
    {
      middleware::g_outgoing_stream_runtime[stream_index].enabled = middleware_streams::outgoing_streams[stream_index].enabled_by_default;
      middleware::g_outgoing_stream_runtime[stream_index].next_send_time_ms = middleware_streams::outgoing_streams[stream_index].phase_offset_ms;
    }
  }

  void tick(std::uint32_t now_ms)
  {
    middleware::g_middleware_state.last_tick_time_ms = now_ms;

    if (middleware::g_middleware_state.has_pending_debug_stream_control)
    {
      set_stream_enabled(middleware::g_middleware_state.pending_debug_stream_control.target_payload_id, middleware::g_middleware_state.pending_debug_stream_control.is_enabled);
      middleware::g_middleware_state.has_pending_debug_stream_control = false;
      middleware::g_middleware_state.pending_debug_stream_control = {};
    }

    for (std::size_t stream_index = 0U; stream_index < middleware_streams::outgoing_stream_count; ++stream_index)
    {
      middleware::outgoing_stream_runtime &runtime = middleware::g_outgoing_stream_runtime[stream_index];
      const middleware_streams::outgoing_stream_definition &stream = middleware_streams::outgoing_streams[stream_index];

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

  bool set_stream_enabled(std::uint8_t payload_id, bool is_enabled)
  {
    bool found_stream = false;

    for (std::size_t stream_index = 0U; stream_index < middleware_streams::outgoing_stream_count; ++stream_index)
    {
      if (middleware_streams::outgoing_streams[stream_index].payload == nullptr || middleware_streams::outgoing_streams[stream_index].payload->payload_id != payload_id)
      {
        continue;
      }

      middleware::g_outgoing_stream_runtime[stream_index].enabled = is_enabled;

      if (is_enabled)
      {
        middleware::g_outgoing_stream_runtime[stream_index].next_send_time_ms = middleware::g_middleware_state.last_tick_time_ms;
      }

      found_stream = true;
    }

    return found_stream;
  }

  bool read_stream_enabled(std::uint8_t payload_id, bool &is_enabled_out)
  {
    for (std::size_t stream_index = 0U; stream_index < middleware_streams::outgoing_stream_count; ++stream_index)
    {
      if (middleware_streams::outgoing_streams[stream_index].payload == nullptr || middleware_streams::outgoing_streams[stream_index].payload->payload_id != payload_id)
      {
        continue;
      }

      is_enabled_out = middleware::g_outgoing_stream_runtime[stream_index].enabled;
      return true;
    }

    return false;
  }
}
