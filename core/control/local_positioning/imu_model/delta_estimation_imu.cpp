#include "core/control/local_positioning/imu_model/delta_estimation_imu.hpp"

namespace
{
  constexpr std::int64_t k_urad_per_degree = 17453LL;
  constexpr std::int64_t k_mdps_dt_scale = 1000000LL;

  std::uint32_t compute_delta_time_ms(std::uint32_t previous_time_ms, std::uint32_t current_time_ms)
  {
    if (current_time_ms >= previous_time_ms)
    {
      return current_time_ms - previous_time_ms;
    }

    return 0u;
  }

  std::int64_t compute_delta_rotation_urad(std::int32_t gyroscope_z_calibrated_mdps, std::uint32_t dt_ms)
  {
    return (static_cast<std::int64_t>(gyroscope_z_calibrated_mdps) *
            static_cast<std::int64_t>(dt_ms) *
            k_urad_per_degree) /
           k_mdps_dt_scale;
  }
}

namespace delta_estimation_imu
{
  void reset(delta_snapshot &state)
  {
    state = {};
  }

  bool estimate_from_imu_snapshot(
      const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot,
      const stationary_detection_imu::stationary_snapshot &stationary_snapshot,
      delta_snapshot &out)
  {
    out = {};

    if (!imu_sample_snapshot.has_previous)
    {
      return false;
    }

    if (!imu_sample_snapshot.can_use_gyro)
    {
      return false;
    }

    if (!imu_sample_snapshot.can_use_accelerometer)
    {
      return false;
    }

    if (!stationary_snapshot.has_stationary_detection)
    {
      return false;
    }

    const imu_api::imu_sample &current_sample = imu_sample_snapshot.current_sample;

    out.imu_id = imu_sample_snapshot.imu_id;
    out.previous_tick_id = imu_sample_snapshot.previous_tick_id;
    out.current_tick_id = imu_sample_snapshot.current_tick_id;
    out.dt_ms = compute_delta_time_ms(imu_sample_snapshot.previous_time_ms, imu_sample_snapshot.current_time_ms);
    out.gyroscope_z_calibrated_mdps = current_sample.gyroscope_z_calibrated_mdps;
    out.accelerometer_x_calibrated_mg = current_sample.accelerometer_x_calibrated_mg;
    out.accelerometer_y_calibrated_mg = current_sample.accelerometer_y_calibrated_mg;
    out.accelerometer_z_calibrated_mg = current_sample.accelerometer_z_calibrated_mg;
    out.is_stationary = stationary_snapshot.is_stationary;

    if (out.dt_ms == 0u)
    {
      return false;
    }

    if (out.is_stationary)
    {
      out.delta_rotation_urad = 0;
    }
    else
    {
      out.delta_rotation_urad = compute_delta_rotation_urad(out.gyroscope_z_calibrated_mdps, out.dt_ms);
    }

    out.has_delta = true;
    return true;
  }
}
