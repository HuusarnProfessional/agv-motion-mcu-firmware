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
    buffer_full,
    io_error
  };

  enum class uart_platform_event_type : std::uint8_t
  {
    received_byte = 0,
    transmitted_byte
  };

  struct uart_platform_event
  {
    uart_platform_event_type type = uart_platform_event_type::received_byte;
    std::uint8_t byte = 0U;
  };

  using platform_event_callback_fn = void (*)(void *event_context, const uart_platform_event *event);
  using uart_configure_fn = bool (*)(void *platform_handle, std::uint16_t tx_pin_id, std::uint16_t rx_pin_id, std::uint32_t baud_rate);
  using uart_register_event_callback_fn = bool (*)(void *platform_handle, platform_event_callback_fn event_callback, void *event_context);
  using uart_enable_receive_fn = bool (*)(void *platform_handle);
  using uart_transmit_byte_fn = bool (*)(void *platform_handle, std::uint8_t byte);

  struct uart_operations
  {
    uart_configure_fn configure;
    uart_register_event_callback_fn register_event_callback;
    uart_enable_receive_fn enable_receive;
    uart_transmit_byte_fn transmit_byte;
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

