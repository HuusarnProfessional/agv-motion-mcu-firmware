#include "core/middleware/stm_esp/incoming_payloads/clear_imu_calibration_payload.hpp"

#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool apply_payload_bytes(stm_esp::middleware_state &state, const std::uint8_t *, std::size_t payload_length, std::uint32_t)
  {
    if (payload_length != 0U)
    {
      return false;
    }

    state.clear_imu_calibration_requested = true;
    return true;
  }
}

namespace stm_esp_incoming_payloads
{
  const incoming_payload_definition clear_imu_calibration_payload_definition = {
    "clear_imu_calibration",
    0x23U,
    apply_payload_bytes
  };
}
