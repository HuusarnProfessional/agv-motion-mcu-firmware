#include "core/api/middleware_api_ai.hpp"

namespace middleware_api_ai
{
  namespace
  {
    const impl_ops *g_impl = nullptr;
    const dispatch_ops *g_dispatch = nullptr;
    const io_buffers *g_buffers = nullptr;
    void *g_app_ctx = nullptr;

    bool tx_via_comm(void *tx_ctx, const char *line)
    {
      (void)tx_ctx;
      return comm_api_ai::send_line(line) == comm_api_ai::status::ok;
    }

    const tx_ops k_tx =
    {
      tx_via_comm,
      nullptr
    };

    handle_status map_comm_status(comm_api_ai::status st)
    {
      switch (st)
      {
        case comm_api_ai::status::ok:
          return handle_status::ok;
        case comm_api_ai::status::not_initialized:
          return handle_status::not_initialized;
        case comm_api_ai::status::invalid_arg:
          return handle_status::invalid_arg;
        case comm_api_ai::status::overflow:
          return handle_status::err_bad_format;
        case comm_api_ai::status::io_error:
        default:
          return handle_status::err_handler;
      }
    }
  }

  void init(const impl_ops *impl,
            const dispatch_ops *dispatch,
            void *app_ctx,
            const comm_link *link,
            const io_buffers *buffers)
  {
    g_impl = impl;
    g_dispatch = dispatch;
    g_buffers = buffers;
    g_app_ctx = app_ctx;

    if (!link || !buffers)
    {
      return;
    }

    comm_api_ai::init_args args{};
    args.impl = link->impl;
    args.transport = link->transport;
    args.transport_ctx = link->transport_ctx;
    args.rx_line_buf = buffers->rx_accum_buf;
    args.rx_line_buf_cap = buffers->rx_accum_cap;
    comm_api_ai::init(args);
  }

  bool is_ready(void)
  {
    if (!g_impl || !g_dispatch || !g_buffers)
    {
      return false;
    }

    if (!g_impl->handle_line)
    {
      return false;
    }

    if (!g_buffers->parsed_line_buf || g_buffers->parsed_line_cap == 0U)
    {
      return false;
    }

    return comm_api_ai::is_ready();
  }

  handle_status handle_line(const char *raw_line)
  {
    if (!raw_line)
    {
      return handle_status::invalid_arg;
    }

    if (!is_ready())
    {
      return handle_status::not_initialized;
    }

    return g_impl->handle_line(raw_line, *g_dispatch, g_app_ctx, k_tx);
  }

  handle_status poll_once(bool &out_had_line)
  {
    out_had_line = false;

    if (!is_ready())
    {
      return handle_status::not_initialized;
    }

    bool has_line = false;
    const comm_api_ai::status st = comm_api_ai::poll_line(g_buffers->parsed_line_buf,
                                                           g_buffers->parsed_line_cap,
                                                           has_line);
    if (st != comm_api_ai::status::ok)
    {
      return map_comm_status(st);
    }

    if (!has_line)
    {
      return handle_status::ok;
    }

    out_had_line = true;
    return handle_line(g_buffers->parsed_line_buf);
  }

  handle_status send_line(const char *line)
  {
    if (!line)
    {
      return handle_status::invalid_arg;
    }

    if (!comm_api_ai::is_ready())
    {
      return handle_status::not_initialized;
    }

    return map_comm_status(comm_api_ai::send_line(line));
  }
}
