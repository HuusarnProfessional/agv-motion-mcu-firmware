#include "core/api/comm_uart_api.hpp"
#include "core/system_select/system_select.hpp"

namespace
{
  comm_uart_api::backend_operation g_backend = {};
  bool g_backend_ready = false;
}

namespace comm_uart_api
{
  void init(const comm_uart_input *inputs, std::size_t count)
  {
    g_backend = {};
    g_backend_ready = false;

    if (inputs == nullptr || count == 0u)
    {
      return;
    }

    system_select::select_comm_uart_backend(g_backend);

    if (g_backend.init_fn == nullptr || g_backend.write_bytes_fn == nullptr || g_backend.read_bytes_fn == nullptr)
    {
      return;
    }

    g_backend.init_fn(inputs, count);
    g_backend_ready = true;
  }

  comm_uart_status write_bytes(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length)
  {
    if (!g_backend_ready || g_backend.write_bytes_fn == nullptr)
    {
      return comm_uart_status::not_initialized;
    }

    return g_backend.write_bytes_fn(comm_uart_id, data, length);
  }

  std::size_t read_bytes(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity)
  {
    if (!g_backend_ready || g_backend.read_bytes_fn == nullptr)
    {
      return 0u;
    }

    return g_backend.read_bytes_fn(comm_uart_id, data_out, capacity);
  }
}

