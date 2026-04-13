#pragma once

#include <array>
#include <cstdint>

#include "core/middleware/middleware_streams.hpp"
#include "core/middleware/middleware_state.hpp"

namespace middleware
{
  struct outgoing_stream_runtime
  {
    bool enabled = false;
    std::uint32_t next_send_time_ms = 0U;
  };

  extern middleware_state g_middleware_state;
  extern std::array<outgoing_stream_runtime, middleware_streams::outgoing_stream_count> g_outgoing_stream_runtime;
}
