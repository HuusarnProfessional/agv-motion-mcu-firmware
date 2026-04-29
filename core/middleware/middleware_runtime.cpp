#include "core/middleware/middleware_runtime.hpp"

namespace middleware
{
  middleware_state g_middleware_state = {};
  std::array<outgoing_stream_runtime, middleware_routes::outgoing_route_count> g_outgoing_stream_runtime = {};
}
