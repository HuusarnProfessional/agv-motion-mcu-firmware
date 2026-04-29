#include "core/middleware/incoming_payloads/service/lock_safe_guard_payload.hpp"

#include "core/control/safe_guard/safe_guard_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *, std::size_t payload_length, std::uint32_t)
  {
    if (payload_length != 0u)
    {
      return false;
    }

    safe_guard::trip();
    (void)state;
    return true;
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition lock_safe_guard_payload_definition = {
    "lock_safe_guard",

    apply_payload_bytes
  };
}
