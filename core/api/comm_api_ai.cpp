#include "core/api/comm_api_ai.hpp"

#include "core/impl/comm_uart_impl_ai.hpp"

namespace comm_api_ai
{
  namespace
  {
    impl_kind g_impl = impl_kind::uart;
    bool g_initialized = false;
  }

  void init(const init_args &args)
  {
    g_impl = args.impl;
    g_initialized = false;

    if (g_impl == impl_kind::uart)
    {
      g_initialized = comm_uart_impl_ai::init(args.transport,
                                              args.transport_ctx,
                                              args.rx_line_buf,
                                              args.rx_line_buf_cap);
      return;
    }

    // Future impls: spi/i2c
    g_initialized = false;
  }

  bool is_ready(void)
  {
    if (!g_initialized)
    {
      return false;
    }

    if (g_impl == impl_kind::uart)
    {
      return comm_uart_impl_ai::is_ready();
    }

    return false;
  }

  status send_line(const char *line)
  {
    if (!is_ready())
    {
      return status::not_initialized;
    }

    if (g_impl == impl_kind::uart)
    {
      return comm_uart_impl_ai::send_line(line);
    }

    return status::not_initialized;
  }

  status poll_line(char *out_line, std::size_t out_line_cap, bool &out_has_line)
  {
    if (!is_ready())
    {
      out_has_line = false;
      return status::not_initialized;
    }

    if (g_impl == impl_kind::uart)
    {
      return comm_uart_impl_ai::poll_line(out_line, out_line_cap, out_has_line);
    }

    out_has_line = false;
    return status::not_initialized;
  }
}
