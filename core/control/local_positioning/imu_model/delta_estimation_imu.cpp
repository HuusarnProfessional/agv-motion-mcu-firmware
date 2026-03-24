#include "core/control/local_positioning/imu_model/delta_estimation_imu.hpp"

namespace delta_estimation_imu
{
  void reset(delta_snapshot &state)
  {
    state = {};
  }

  bool sample_from_imu_api(std::uint8_t imu_id, std::uint32_t tick_id, delta_snapshot &out)
  {
    out = {};

    imu_api::imu_sample sample = {};
    const bool read_ok = imu_api::read_sample(imu_id, sample);

    out.tick_id = tick_id;
    out.time_ms = sample.time_ms;
    out.gyroscope_z_mdps = sample.gyroscope_z_mdps;
    out.gyroscope_state = sample.gyroscope_state;
    out.accelerometer_x_mg = sample.accelerometer_x_mg;
    out.accelerometer_y_mg = sample.accelerometer_y_mg;
    out.accelerometer_z_mg = sample.accelerometer_z_mg;
    out.accelerometer_state = sample.accelerometer_state;

    if (!read_ok)
    {
      return false;
    }

    out.has_delta = true;
    return true;
  }
}
