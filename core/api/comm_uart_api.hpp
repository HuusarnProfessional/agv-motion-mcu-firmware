#pragma once

#include <cstddef>
#include <cstdint>

namespace comm_uart_api
{
  enum class comm_uart_status : std::uint8_t
  {
    ok = 0,
    not_initialized,
    invalid_arg,
    invalid_id,
    io_error
  };

  using uart_configure_fn = bool (*)(void *platform_handle, std::uint16_t tx_pin_id, std::uint16_t rx_pin_id, std::uint32_t baud_rate);
  using uart_write_bytes_fn = bool (*)(void *platform_handle, const std::uint8_t *data, std::size_t length);
  using uart_read_bytes_fn = std::size_t (*)(void *platform_handle, std::uint8_t *data_out, std::size_t capacity);

  struct uart_operations
  {
    uart_configure_fn configure;
    uart_write_bytes_fn write_bytes;
    uart_read_bytes_fn read_bytes;
  };

  struct comm_uart_input
  {
    void *platform_handle;
    std::uint16_t tx_pin_id;
    std::uint16_t rx_pin_id;
    std::uint32_t baud_rate;
    const uart_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const comm_uart_input *inputs, std::size_t count);
  using backend_write_bytes_fn = comm_uart_status (*)(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length);
  using backend_read_bytes_fn = std::size_t (*)(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_write_bytes_fn write_bytes_fn;
    backend_read_bytes_fn read_bytes_fn;
  };

  void init(const comm_uart_input *inputs, std::size_t count);
  comm_uart_status write_bytes(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length);
  std::size_t read_bytes(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity);

}

