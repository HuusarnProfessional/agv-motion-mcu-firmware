#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"

namespace imu_input_storage
{
  void reset(imu_sample_snapshot &state)
  {
    state = {};
  }

  bool sample_from_imu_api(imu_sample_snapshot &state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms)
  {
    imu_api::imu_sample sample = {};
    const bool read_ok = imu_api::read_sample(imu_id, sample);

    if (!read_ok)
    {
      return false;
    }

    if (state.has_input)
    {
      state.previous_tick_id = state.current_tick_id;
      state.previous_time_ms = state.current_time_ms;
      state.previous_sample = state.current_sample;
      state.has_previous = true;
    }

    state.imu_id = imu_id;
    state.current_tick_id = tick_id;
    state.current_time_ms = now_ms;
    state.current_sample = sample;
    state.has_input = true;

    if (sample.gyroscope_state == imu_api::gyroscope_status::ok)
    {
      state.can_use_gyro = true;
    }
    else
    {
      state.can_use_gyro = false;
    }

    if (sample.accelerometer_state == imu_api::accelerometer_status::ok)
    {
      state.can_use_accelerometer = true;
    }
    else
    {
      state.can_use_accelerometer = false;
    }

    return true;
  }
}
