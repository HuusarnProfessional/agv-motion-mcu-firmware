#include "core/control/local_positioning/imu_model/stationary_detection_imu.hpp"

namespace stationary_detection_imu
{
  void reset(stationary_snapshot &state)
  {
    state = {};
  }

  bool estimate_from_imu_snapshot(const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot, stationary_snapshot &out)
  {
    (void)imu_sample_snapshot;
    out = {};
    return false;
  }
}
