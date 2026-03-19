#pragma once

#include <cstddef>
#include <cstdint>

namespace comm_api_ai
{
  enum class status : std::uint8_t
  {
    ok = 0,
    not_initialized,
    invalid_arg,
    io_error,
    overflow
  };

  enum class impl_kind : std::uint8_t
  {
    uart = 0,
    spi = 1,
    i2c = 2
  };

  using tx_bytes_fn = status (*)(void *ctx, const std::uint8_t *data, std::size_t len);
  using rx_bytes_fn = std::size_t (*)(void *ctx, std::uint8_t *dst, std::size_t cap);

  struct transport_ops
  {
    tx_bytes_fn tx_bytes;
    rx_bytes_fn rx_bytes;
  };

  struct init_args
  {
    impl_kind impl;
    const transport_ops *transport;
    void *transport_ctx;
    char *rx_line_buf;
    std::size_t rx_line_buf_cap;
  };

  void init(const init_args &args);
  bool is_ready(void);

  // Sends a line and appends '\n'.
  status send_line(const char *line);

  // Non-blocking poll that returns at most one full line.
  // out_has_line == true means out_line contains one full line (without '\r'/'\n').
  status poll_line(char *out_line, std::size_t out_line_cap, bool &out_has_line);
}
