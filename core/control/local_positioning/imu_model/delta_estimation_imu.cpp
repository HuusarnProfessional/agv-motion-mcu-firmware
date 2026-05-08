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
    return (static_cast<std::int64_t>(gyroscope_z_calibrated_mdps) * static_cast<std::int64_t>(dt_ms) * k_urad_per_degree) / k_mdps_dt_scale;
  }

  bool update_rotation_delta(
      const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot,
      const stationary_detection_imu::stationary_snapshot &stationary_snapshot,
      delta_estimation_imu::delta_snapshot &out)
  {
    if (!imu_sample_snapshot.gyro.can_estimate_delta)
    {
      return false;
    }

    out.previous_gyro_tick_id = imu_sample_snapshot.gyro.previous_tick_id;
    out.current_gyro_tick_id = imu_sample_snapshot.gyro.current_tick_id;
    out.gyro_dt_ms = compute_delta_time_ms(imu_sample_snapshot.gyro.previous_time_ms, imu_sample_snapshot.gyro.current_time_ms);
    out.gyroscope_z_calibrated_mdps = imu_sample_snapshot.gyro.current_gyroscope_z_calibrated_mdps;

    if (out.gyro_dt_ms == 0u)
    {
      return false;
    }

    if (stationary_snapshot.has_stationary_detection && stationary_snapshot.is_stationary)
    {
      out.delta_rotation_urad = 0;
    }
    else
    {
      const std::int32_t previous_gyro_z_calibrated_mdps = imu_sample_snapshot.gyro.previous_gyroscope_z_calibrated_mdps;
      const std::int32_t current_gyro_z_calibrated_mdps = imu_sample_snapshot.gyro.current_gyroscope_z_calibrated_mdps;
      const std::int32_t average_gyro_z_calibrated_mdps = (previous_gyro_z_calibrated_mdps + current_gyro_z_calibrated_mdps) / 2;
      out.delta_rotation_urad = compute_delta_rotation_urad(average_gyro_z_calibrated_mdps, out.gyro_dt_ms);
    }

    out.has_rotation_delta = true;
    return true;
  }

  bool update_translation_delta(const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot, delta_estimation_imu::delta_snapshot &out)
  {
    if (!imu_sample_snapshot.accelerometer.can_estimate_delta)
    {
      return false;
    }

    out.previous_accelerometer_tick_id = imu_sample_snapshot.accelerometer.previous_tick_id;
    out.current_accelerometer_tick_id = imu_sample_snapshot.accelerometer.current_tick_id;
    out.accelerometer_dt_ms = compute_delta_time_ms(imu_sample_snapshot.accelerometer.previous_time_ms, imu_sample_snapshot.accelerometer.current_time_ms);
    out.accelerometer_x_calibrated_mg = imu_sample_snapshot.accelerometer.current_accelerometer_x_calibrated_mg;
    out.accelerometer_y_calibrated_mg = imu_sample_snapshot.accelerometer.current_accelerometer_y_calibrated_mg;
    out.accelerometer_z_calibrated_mg = imu_sample_snapshot.accelerometer.current_accelerometer_z_calibrated_mg;

    if (out.accelerometer_dt_ms == 0u)
    {
      return false;
    }

    out.has_translation_delta = true;
    return true;
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
    out.imu_id = imu_sample_snapshot.imu_id;
    out.is_stationary = stationary_snapshot.is_stationary;
    update_rotation_delta(imu_sample_snapshot, stationary_snapshot, out);
    update_translation_delta(imu_sample_snapshot, out);

    if (!out.has_rotation_delta && !out.has_translation_delta)
    {
      return false;
    }

    out.has_delta = true;
    return true;
  }
}
