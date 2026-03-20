#include "core/impl/comm_uart_impl.hpp"
namespace
{
  const comm_uart_api::comm_uart_input *g_inputs = nullptr;
  std::size_t g_input_count = 0u;

  const comm_uart_api::comm_uart_input *get_input(std::uint8_t comm_uart_id)
  {
    if (g_inputs == nullptr || g_input_count == 0u)
    {
      return nullptr;
    }

    if (comm_uart_id >= g_input_count)
    {
      return nullptr;
    }

    return &g_inputs[comm_uart_id];
  }
}

namespace comm_uart_impl
{
  void init(const comm_uart_api::comm_uart_input *inputs, std::size_t count)
  {
    g_inputs = nullptr;
    g_input_count = 0u;

    if (inputs == nullptr || count == 0u)
    {
      return;
    }

    for (std::size_t i = 0u; i < count; ++i)
    {
      const comm_uart_api::comm_uart_input &selected_input = inputs[i];

      if (selected_input.platform_operations == nullptr)
      {
        return;
      }

      if (selected_input.platform_operations->configure == nullptr ||
          selected_input.platform_operations->write_bytes == nullptr ||
          selected_input.platform_operations->read_bytes == nullptr)
      {
        return;
      }

      if (!selected_input.platform_operations->configure(selected_input.platform_handle, selected_input.tx_pin_id, selected_input.rx_pin_id, selected_input.baud_rate))
      {
        return;
      }
    }

    g_inputs = inputs;
    g_input_count = count;
  }

  comm_uart_api::comm_uart_status write_bytes(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length)
  {
    const comm_uart_api::comm_uart_input *selected_input = get_input(comm_uart_id);
    if (selected_input == nullptr)
    {
      if (g_inputs == nullptr || g_input_count == 0u)
      {
        return comm_uart_api::comm_uart_status::not_initialized;
      }

      return comm_uart_api::comm_uart_status::invalid_id;
    }

    if (data == nullptr && length > 0u)
    {
      return comm_uart_api::comm_uart_status::invalid_arg;
    }

    const bool ok = selected_input->platform_operations->write_bytes(selected_input->platform_handle, data, length);
    if (!ok)
    {
      return comm_uart_api::comm_uart_status::io_error;
    }

    return comm_uart_api::comm_uart_status::ok;
  }

  std::size_t read_bytes(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity)
  {
    const comm_uart_api::comm_uart_input *selected_input = get_input(comm_uart_id);
    if (selected_input == nullptr)
    {
      return 0u;
    }

    if (data_out == nullptr && capacity > 0u)
    {
      return 0u;
    }

    return selected_input->platform_operations->read_bytes(selected_input->platform_handle, data_out, capacity);
  }
}

