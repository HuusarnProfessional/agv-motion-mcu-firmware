#include "core/middleware/incoming_middleware_pipeline.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/api/comm_uart_api.hpp"
#include "core/middleware/middleware_streams.hpp"
#include "core/middleware/middleware_shared_state.hpp"

namespace
{
  constexpr std::uint8_t k_sync_byte = 0xA5U;
  constexpr std::size_t k_read_chunk_length = 64U;

  void reset_parser(middleware::incoming_packet_parser &parser)
  {
    parser.step = middleware::incoming_packet_step::wait_for_sync;
    parser.payload_id = 0U;
    parser.payload_length = 0U;
    parser.payload_bytes_read = 0U;
  }

  bool apply_incoming_payload(std::uint8_t payload_id, const std::uint8_t *payload_data, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    for (std::size_t payload_index = 0U; payload_index < middleware_streams::incoming_payload_count; ++payload_index)
    {
      const middleware_incoming_payloads::incoming_payload_definition *payload = middleware_streams::incoming_payloads[payload_index];

      if (payload == nullptr || payload->payload_id != payload_id || payload->apply_payload_bytes == nullptr)
      {
        continue;
      }

      return payload->apply_payload_bytes(middleware::g_middleware_state, payload_data, payload_length, received_time_ms);
    }

    return false;
  }

  void process_incoming_byte(std::uint8_t incoming_byte, std::uint32_t now_ms)
  {
    switch (middleware::g_middleware_state.parser.step)
    {
    case middleware::incoming_packet_step::wait_for_sync:
      if (incoming_byte == k_sync_byte)
      {
        middleware::g_middleware_state.parser.step = middleware::incoming_packet_step::wait_for_payload_id;
      }
      break;

    case middleware::incoming_packet_step::wait_for_payload_id:
      middleware::g_middleware_state.parser.payload_id = incoming_byte;
      middleware::g_middleware_state.parser.step = middleware::incoming_packet_step::wait_for_payload_length;
      break;

    case middleware::incoming_packet_step::wait_for_payload_length:
      middleware::g_middleware_state.parser.payload_length = incoming_byte;
      middleware::g_middleware_state.parser.payload_bytes_read = 0U;

      if (middleware::g_middleware_state.parser.payload_length == 0U)
      {
        apply_incoming_payload(middleware::g_middleware_state.parser.payload_id, nullptr, 0U, now_ms);
        reset_parser(middleware::g_middleware_state.parser);
      }
      else
      {
        middleware::g_middleware_state.parser.step = middleware::incoming_packet_step::read_payload_bytes;
      }
      break;

    case middleware::incoming_packet_step::read_payload_bytes:
      if (middleware::g_middleware_state.parser.payload_bytes_read >= middleware::g_middleware_state.parser.payload_bytes.size())
      {
        reset_parser(middleware::g_middleware_state.parser);
        break;
      }

      middleware::g_middleware_state.parser.payload_bytes[middleware::g_middleware_state.parser.payload_bytes_read++] = incoming_byte;

      if (middleware::g_middleware_state.parser.payload_bytes_read == middleware::g_middleware_state.parser.payload_length)
      {
        apply_incoming_payload(middleware::g_middleware_state.parser.payload_id, middleware::g_middleware_state.parser.payload_bytes.data(), middleware::g_middleware_state.parser.payload_length, now_ms);
        reset_parser(middleware::g_middleware_state.parser);
      }
      break;
    }
  }

  void read_incoming_packets(std::uint32_t now_ms)
  {
    std::array<std::uint8_t, k_read_chunk_length> incoming_bytes = {};

    while (true)
    {
      const std::size_t bytes_read = comm_uart_api::read_bytes(middleware::g_middleware_state.comm_uart_id, incoming_bytes.data(), incoming_bytes.size());

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
}

namespace incoming_middleware_pipeline
{
  void init(std::uint8_t comm_uart_id)
  {
    middleware::g_middleware_state = {};
    middleware::g_middleware_state.comm_uart_id = comm_uart_id;
    reset_parser(middleware::g_middleware_state.parser);
  }

  void tick(std::uint32_t now_ms)
  {
    read_incoming_packets(now_ms);
  }
}
