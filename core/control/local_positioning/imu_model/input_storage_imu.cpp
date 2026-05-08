#include "core/control/local_positioning/imu_model/input_storage_imu.hpp"

namespace imu_input_storage
{
  namespace
  {
    bool has_fresh_gyro_sample(const imu_sample_snapshot &state, const imu_api::imu_sample &sample)
    {
      if (!state.gyro.has_current_sample)
      {
        return true;
      }

      if (sample.gyroscope_sample_id == state.gyro.current_sample_id)
      {
        return false;
      }

      return true;
    }

    bool has_fresh_accelerometer_sample(const imu_sample_snapshot &state, const imu_api::imu_sample &sample)
    {
      if (!state.accelerometer.has_current_sample)
      {
        return true;
      }

      if (sample.accelerometer_sample_id == state.accelerometer.current_sample_id)
      {
        return false;
      }

      return true;
    }

    void prepare_new_tick(imu_sample_snapshot &state, std::uint8_t imu_id, const imu_api::imu_sample &sample, std::uint32_t now_ms)
    {
      state.imu_id = imu_id;
      state.latest_sample = sample;
      state.latest_time_ms = now_ms;
      state.has_input = true;
      state.gyro.has_fresh_sample = false;
      state.gyro.can_estimate_delta = false;
      state.accelerometer.has_fresh_sample = false;
      state.accelerometer.can_estimate_delta = false;
    }

    void update_gyro_input(imu_sample_snapshot &state, const imu_api::imu_sample &sample, std::uint32_t tick_id, std::uint32_t now_ms)
    {
      if (state.gyro.has_current_sample)
      {
        state.gyro.previous_tick_id = state.gyro.current_tick_id;
        state.gyro.previous_time_ms = state.gyro.current_time_ms;
        state.gyro.previous_sample_id = state.gyro.current_sample_id;
        state.gyro.previous_gyroscope_z_calibrated_mdps = state.gyro.current_gyroscope_z_calibrated_mdps;
        state.gyro.has_previous_sample = true;
      }

      state.gyro.current_tick_id = tick_id;
      state.gyro.current_time_ms = now_ms;
      state.gyro.current_sample_id = sample.gyroscope_sample_id;
      state.gyro.current_gyroscope_z_calibrated_mdps = sample.gyroscope_z_calibrated_mdps;
      state.gyro.has_current_sample = true;
      state.gyro.has_fresh_sample = true;

      if (state.gyro.has_previous_sample)
      {
        state.gyro.can_estimate_delta = true;
      }
    }

    void update_accelerometer_input(imu_sample_snapshot &state, const imu_api::imu_sample &sample, std::uint32_t tick_id, std::uint32_t now_ms)
    {
      if (state.accelerometer.has_current_sample)
      {
        state.accelerometer.previous_tick_id = state.accelerometer.current_tick_id;
        state.accelerometer.previous_time_ms = state.accelerometer.current_time_ms;
        state.accelerometer.previous_sample_id = state.accelerometer.current_sample_id;
        state.accelerometer.previous_accelerometer_x_calibrated_mg = state.accelerometer.current_accelerometer_x_calibrated_mg;
        state.accelerometer.previous_accelerometer_y_calibrated_mg = state.accelerometer.current_accelerometer_y_calibrated_mg;
        state.accelerometer.previous_accelerometer_z_calibrated_mg = state.accelerometer.current_accelerometer_z_calibrated_mg;
        state.accelerometer.has_previous_sample = true;
      }

      state.accelerometer.current_tick_id = tick_id;
      state.accelerometer.current_time_ms = now_ms;
      state.accelerometer.current_sample_id = sample.accelerometer_sample_id;
      state.accelerometer.current_accelerometer_x_calibrated_mg = sample.accelerometer_x_calibrated_mg;
      state.accelerometer.current_accelerometer_y_calibrated_mg = sample.accelerometer_y_calibrated_mg;
      state.accelerometer.current_accelerometer_z_calibrated_mg = sample.accelerometer_z_calibrated_mg;
      state.accelerometer.has_current_sample = true;
      state.accelerometer.has_fresh_sample = true;

      if (state.accelerometer.has_previous_sample)
      {
        state.accelerometer.can_estimate_delta = true;
      }
    }
  }

  void reset(imu_sample_snapshot &state)
  {
    state = {};
  }

  bool sample_from_imu_api(imu_sample_snapshot &state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms)
  {
    imu_api::imu_sample sample = {};

    if (!imu_api::read_sample(imu_id, sample))
    {
      return false;
    }

    const bool has_fresh_gyro = has_fresh_gyro_sample(state, sample);
    const bool has_fresh_accelerometer = has_fresh_accelerometer_sample(state, sample);
    prepare_new_tick(state, imu_id, sample, now_ms);

    if (!has_fresh_gyro && !has_fresh_accelerometer)
    {
      return false;
    }

    if (sample.gyroscope_state == imu_api::gyroscope_status::ok && has_fresh_gyro)
    {
      update_gyro_input(state, sample, tick_id, now_ms);
    }

    if (sample.accelerometer_state == imu_api::accelerometer_status::ok && has_fresh_accelerometer)
    {
      update_accelerometer_input(state, sample, tick_id, now_ms);
    }

    if (!state.gyro.has_fresh_sample && !state.accelerometer.has_fresh_sample)
    {
      return false;
    }

    return true;
  }
}
