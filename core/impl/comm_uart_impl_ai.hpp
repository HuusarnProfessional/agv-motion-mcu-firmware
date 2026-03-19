#pragma once

#include <cstddef>

#include "core/api/comm_api_ai.hpp"

namespace comm_uart_impl_ai
{
  bool init(const comm_api_ai::transport_ops *transport_ops,
            void *transport_ctx,
            char *rx_line_buf,
            std::size_t rx_line_buf_cap);

  bool is_ready(void);

  comm_api_ai::status send_line(const char *line);
  comm_api_ai::status poll_line(char *out_line, std::size_t out_line_cap, bool &out_has_line);

  void reset_rx_state(void);
}
