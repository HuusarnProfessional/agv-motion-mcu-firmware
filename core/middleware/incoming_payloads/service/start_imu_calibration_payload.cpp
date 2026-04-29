#include "core/middleware/incoming_payloads/service/start_imu_calibration_payload.hpp"

#include "core/control/imu_calibration/imu_calibration_pipeline.hpp"
#include "core/middleware/middleware_runtime.hpp"

namespace
{
  bool apply_payload_bytes(middleware::middleware_state &state, const std::uint8_t *, std::size_t payload_length, std::uint32_t)
  {
    if (payload_length != 0u)
    {
      return false;
    }

    imu_calibration::request_start();
    (void)state;
    return true;
  }
}

namespace middleware_incoming_payloads
{
  const incoming_payload_definition start_imu_calibration_payload_definition = 
  {
    "start_imu_calibration",

    apply_payload_bytes
  };
}
