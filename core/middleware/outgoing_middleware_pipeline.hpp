#pragma once

#include <cstdint>

namespace outgoing_middleware_pipeline
{
  void init(void);
  void tick(std::uint32_t now_ms);

  bool set_stream_enabled(std::uint8_t payload_id, bool is_enabled);
  bool read_stream_enabled(std::uint8_t payload_id, bool &is_enabled_out);
}
