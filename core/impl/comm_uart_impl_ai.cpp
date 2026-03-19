#include "core/impl/comm_uart_impl_ai.hpp"

namespace comm_uart_impl_ai
{
  namespace
  {
    const comm_api_ai::transport_ops *g_ops = nullptr;
    void *g_ctx = nullptr;

    char *g_line_buf = nullptr;
    std::size_t g_line_buf_cap = 0U;
    std::size_t g_line_len = 0U;
    bool g_drop_until_newline = false;

    bool g_ready = false;

    static comm_api_ai::status parse_rx_bytes(const std::uint8_t *data,
                                              std::size_t len,
                                              char *out_line,
                                              std::size_t out_line_cap,
                                              bool &out_has_line)
    {
      out_has_line = false;
      if (!data || len == 0U)
      {
        return comm_api_ai::status::ok;
      }

      for (std::size_t i = 0; i < len; ++i)
      {
        const std::uint8_t b = data[i];
        const char c = static_cast<char>(b);

        if (g_drop_until_newline)
        {
          if (c == '\n')
          {
            g_drop_until_newline = false;
            g_line_len = 0U;
          }
          continue;
        }

        if (c == '\r')
        {
          continue;
        }

        if (c == '\n')
        {
          if (g_line_len == 0U)
          {
            continue;
          }

          if (g_line_len + 1U > out_line_cap)
          {
            g_line_len = 0U;
            return comm_api_ai::status::overflow;
          }

          for (std::size_t j = 0; j < g_line_len; ++j)
          {
            out_line[j] = g_line_buf[j];
          }
          out_line[g_line_len] = '\0';
          g_line_len = 0U;
          out_has_line = true;
          return comm_api_ai::status::ok;
        }

        // Protocol is ASCII text lines. If a non-printable byte appears,
        // do not keep a partially corrupted command; drop the whole line
        // and resync on next '\n'.
        if (b < 32U || b > 126U)
        {
          g_drop_until_newline = true;
          g_line_len = 0U;
          return comm_api_ai::status::overflow;
        }

        if (g_line_len + 1U >= g_line_buf_cap)
        {
          g_drop_until_newline = true;
          g_line_len = 0U;
          return comm_api_ai::status::overflow;
        }

        g_line_buf[g_line_len] = c;
        ++g_line_len;
      }

      return comm_api_ai::status::ok;
    }
  }

  bool init(const comm_api_ai::transport_ops *transport_ops,
            void *transport_ctx,
            char *rx_line_buf,
            std::size_t rx_line_buf_cap)
  {
    g_ready = false;

    if (!transport_ops || !transport_ops->tx_bytes || !transport_ops->rx_bytes)
    {
      return false;
    }
    if (!rx_line_buf || rx_line_buf_cap == 0U)
    {
      return false;
    }

    g_ops = transport_ops;
    g_ctx = transport_ctx;

    g_line_buf = rx_line_buf;
    g_line_buf_cap = rx_line_buf_cap;
    g_line_len = 0U;
    g_drop_until_newline = false;

    g_ready = true;
    return true;
  }

  bool is_ready(void)
  {
    if (!g_ready)
    {
      return false;
    }

    if (!g_ops || !g_ops->tx_bytes || !g_ops->rx_bytes)
    {
      return false;
    }

    if (!g_line_buf || g_line_buf_cap == 0U)
    {
      return false;
    }

    return true;
  }

  comm_api_ai::status send_line(const char *line)
  {
    if (!line)
    {
      return comm_api_ai::status::invalid_arg;
    }

    if (!is_ready())
    {
      return comm_api_ai::status::not_initialized;
    }

    std::size_t len = 0U;
    while (line[len] != '\0')
    {
      ++len;
    }

    const comm_api_ai::status tx_main = g_ops->tx_bytes(g_ctx,
                                                         reinterpret_cast<const std::uint8_t *>(line),
                                                         len);
    if (tx_main != comm_api_ai::status::ok)
    {
      return tx_main;
    }

    const std::uint8_t nl = static_cast<std::uint8_t>('\n');
    return g_ops->tx_bytes(g_ctx, &nl, 1U);
  }

  comm_api_ai::status poll_line(char *out_line, std::size_t out_line_cap, bool &out_has_line)
  {
    out_has_line = false;

    if (!out_line || out_line_cap == 0U)
    {
      return comm_api_ai::status::invalid_arg;
    }

    if (!is_ready())
    {
      return comm_api_ai::status::not_initialized;
    }

    // Consume ingress one byte at a time until we either produce one full line
    // or run out of bytes. This avoids dropping bytes when transport returns
    // multiple lines in one burst.
    for (std::size_t i = 0U; i < 64U; ++i)
    {
      std::uint8_t b = 0U;
      const std::size_t rx_n = g_ops->rx_bytes(g_ctx, &b, 1U);
      if (rx_n == 0U)
      {
        return comm_api_ai::status::ok;
      }

      const comm_api_ai::status st = parse_rx_bytes(&b, 1U, out_line, out_line_cap, out_has_line);
      if (st != comm_api_ai::status::ok)
      {
        return st;
      }

      if (out_has_line)
      {
        return comm_api_ai::status::ok;
      }
    }

    return comm_api_ai::status::ok;
  }

  void reset_rx_state(void)
  {
    g_line_len = 0U;
    g_drop_until_newline = false;
  }
}
