#include "core/middleware/incoming_payloads/service/heartbeat_payload.hpp"

#include "core/control/heartbeat/heartbeat_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *, std::size_t payload_length, std::uint32_t received_time_ms)
  {
    if (payload_length != 0u)
    {
      return false;
    }

    heartbeat::notify_heartbeat(received_time_ms);
    (void)state;
    return true;
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition heartbeat_payload_definition =
  {
    "heartbeat",

    apply_payload_bytes
  };
}
