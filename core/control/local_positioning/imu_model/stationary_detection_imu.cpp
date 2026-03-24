#include "core/control/local_positioning/imu_model/stationary_detection_imu.hpp"
#include "core/control/local_positioning/imu_model/imu_model_tuning.hpp"

#include <cstdlib>

namespace
{
  std::int32_t scale_noise_limit(std::int32_t noise_value, std::int32_t minimum_limit)
  {
    const std::int64_t scaled_limit =
        (static_cast<std::int64_t>(noise_value) * imu_model_tuning::k_stationary_noise_scale_percent) / 100;

    if (scaled_limit > minimum_limit)
    {
      return static_cast<std::int32_t>(scaled_limit);
    }

    return minimum_limit;
  }

  std::int32_t low_pass_filter(std::int32_t previous_value, std::int32_t current_value)
  {
    const std::int64_t previous_weight = 100u - imu_model_tuning::k_stationary_low_pass_alpha_percent;
    const std::int64_t current_weight = imu_model_tuning::k_stationary_low_pass_alpha_percent;

    return static_cast<std::int32_t>(
        ((static_cast<std::int64_t>(previous_value) * previous_weight) +
         (static_cast<std::int64_t>(current_value) * current_weight)) /
        100);
  }
}

namespace stationary_detection_imu
{
  void reset(stationary_snapshot &state)
  {
    state = {};
  }

  bool estimate_from_imu_snapshot(const imu_input_storage::imu_sample_snapshot &imu_sample_snapshot, stationary_snapshot &out)
  {
    out.is_stationary = false;
    out.has_stationary_detection = false;

    if (!imu_sample_snapshot.has_input)
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

    const imu_api::imu_sample &sample = imu_sample_snapshot.current_sample;
    imu_api::imu_calibration_profile calibration_profile = {};

    if (!out.has_filter_state)
    {
      out.filtered_gyroscope_z_mdps = sample.gyroscope_z_calibrated_mdps;
      out.filtered_accelerometer_x_mg = sample.accelerometer_x_calibrated_mg;
      out.filtered_accelerometer_y_mg = sample.accelerometer_y_calibrated_mg;
      out.has_filter_state = true;
    }
    else
    {
      out.filtered_gyroscope_z_mdps = low_pass_filter(out.filtered_gyroscope_z_mdps, sample.gyroscope_z_calibrated_mdps);
      out.filtered_accelerometer_x_mg = low_pass_filter(out.filtered_accelerometer_x_mg, sample.accelerometer_x_calibrated_mg);
      out.filtered_accelerometer_y_mg = low_pass_filter(out.filtered_accelerometer_y_mg, sample.accelerometer_y_calibrated_mg);
    }

    std::int32_t gyro_z_limit_mdps = imu_model_tuning::k_stationary_gyro_z_min_limit_mdps;
    std::int32_t acc_x_limit_mg = imu_model_tuning::k_stationary_acc_x_min_limit_mg;
    std::int32_t acc_y_limit_mg = imu_model_tuning::k_stationary_acc_y_min_limit_mg;

    if (imu_api::get_calibration(imu_sample_snapshot.imu_id, calibration_profile) &&
        calibration_profile.noise.has_noise_profile)
    {
      gyro_z_limit_mdps = scale_noise_limit(calibration_profile.noise.gyroscope_z_p_proc_mdps, gyro_z_limit_mdps);
      acc_x_limit_mg = scale_noise_limit(calibration_profile.noise.accelerometer_x_p_proc_mg, acc_x_limit_mg);
      acc_y_limit_mg = scale_noise_limit(calibration_profile.noise.accelerometer_y_p_proc_mg, acc_y_limit_mg);
    }

    const bool gyro_is_small = std::abs(out.filtered_gyroscope_z_mdps) <= gyro_z_limit_mdps;
    const bool acc_x_is_small = std::abs(out.filtered_accelerometer_x_mg) <= acc_x_limit_mg;
    const bool acc_y_is_small = std::abs(out.filtered_accelerometer_y_mg) <= acc_y_limit_mg;

    out.is_stationary = gyro_is_small && acc_x_is_small && acc_y_is_small;
    out.has_stationary_detection = true;
    return true;
  }
}
