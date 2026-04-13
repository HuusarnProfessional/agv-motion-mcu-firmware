#include "core/middleware/middleware_shared_state.hpp"

namespace middleware
{
  middleware_state g_middleware_state = {};
  std::array<outgoing_stream_runtime, middleware_streams::outgoing_stream_count> g_outgoing_stream_runtime = {};
}
