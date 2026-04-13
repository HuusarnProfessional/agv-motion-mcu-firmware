#pragma once

#include <cstdint>

namespace incoming_middleware_pipeline
{
  void init(std::uint8_t comm_uart_id);
  void tick(std::uint32_t now_ms);
}
