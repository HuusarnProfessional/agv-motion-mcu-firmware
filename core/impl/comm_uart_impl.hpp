#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/comm_uart_api.hpp"

namespace comm_uart_impl
{
  void init(const comm_uart_api::comm_uart_input *inputs, std::size_t count);
  comm_uart_api::comm_uart_status write_bytes(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length);
  std::size_t read_bytes(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity);
}

