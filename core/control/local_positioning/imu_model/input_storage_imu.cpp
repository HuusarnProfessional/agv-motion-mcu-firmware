#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"

namespace imu_input_storage
{
  void reset(imu_sample_snapshot &state)
  {
    state = {};
  }

  bool sample_from_imu_api(imu_sample_snapshot &state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms)
  {
    (void)state;
    (void)imu_id;
    (void)tick_id;
    (void)now_ms;
    return false;
  }
}
