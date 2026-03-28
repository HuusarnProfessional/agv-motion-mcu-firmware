#include "core/control/local_positioning/imu_model/motion_model_imu.hpp"
#include "core/control/local_positioning/imu_model/imu_model_tuning.hpp"

#include "core/api/imu_api.hpp"

#include <cstdlib>

namespace
{
  std::int32_t choose_noise_limit(std::int32_t noise_value, std::int32_t minimum_limit)
  {
    if (noise_value > minimum_limit)
    {
      return noise_value;
    }

    return minimum_limit;
  }

  std::int64_t compute_signal_confidence(std::int32_t signal_value, std::int32_t noise_limit)
  {
    const std::int64_t absolute_signal = std::abs(static_cast<std::int64_t>(signal_value));
    const std::int64_t denominator = absolute_signal + static_cast<std::int64_t>(noise_limit);

    if (denominator <= 0)
    {
      return 0;
    }

    return (absolute_signal * imu_model_tuning::k_motion_confidence_max) / denominator;
  }

  std::int64_t convert_acceleration_to_um_per_s2(std::int32_t accelerometer_x_calibrated_mg)
  {
    return static_cast<std::int64_t>(accelerometer_x_calibrated_mg) * imu_model_tuning::k_um_per_mg_per_s2;
  }
}

namespace motion_model_imu
{
  void reset(motion_model_state &state)
  {
    state = {};
  }

  void reset(motion_model_snapshot &state)
  {
    state = {};
  }

  bool estimate_from_imu_delta(const delta_estimation_imu::delta_snapshot &delta_snapshot, motion_model_state &state, motion_model_snapshot &out)
  {
    out = {};

    if (!delta_snapshot.has_delta)
    {
      return false;
    }

    out.is_stationary = delta_snapshot.is_stationary;
    out.gyroscope_z_calibrated_mdps = delta_snapshot.gyroscope_z_calibrated_mdps;

    imu_api::imu_calibration_profile calibration_profile = {};

    std::int32_t gyro_z_noise_limit_mdps = imu_model_tuning::k_motion_gyro_z_noise_floor_mdps;
    std::int32_t acc_x_noise_limit_mg = imu_model_tuning::k_motion_acc_x_noise_floor_mg;

    if (imu_api::get_calibration(delta_snapshot.imu_id, calibration_profile) &&
        calibration_profile.noise.has_noise_profile)
    {
      gyro_z_noise_limit_mdps = choose_noise_limit(calibration_profile.noise.gyroscope_z_p_proc_mdps, gyro_z_noise_limit_mdps);
      acc_x_noise_limit_mg = choose_noise_limit(calibration_profile.noise.accelerometer_x_p_proc_mg, acc_x_noise_limit_mg);
    }

    if (delta_snapshot.is_stationary)
    {
      state.forward_velocity_um_per_s = 0;
      out.has_motion_model = true;
      return true;
    }

    const std::int64_t acceleration_x_um_per_s2 = convert_acceleration_to_um_per_s2(delta_snapshot.accelerometer_x_calibrated_mg);
    const std::int64_t previous_velocity_um_per_s = state.forward_velocity_um_per_s;
    const std::int64_t dt_ms = static_cast<std::int64_t>(delta_snapshot.dt_ms);

    out.translation =
        (previous_velocity_um_per_s * dt_ms) / 1000 +
        (acceleration_x_um_per_s2 * dt_ms * dt_ms) / 2000000;

    state.forward_velocity_um_per_s =
        previous_velocity_um_per_s +
        (acceleration_x_um_per_s2 * dt_ms) / 1000;

    out.rotation = delta_snapshot.delta_rotation_urad;
    out.confidence_translation = compute_signal_confidence(delta_snapshot.accelerometer_x_calibrated_mg, acc_x_noise_limit_mg);
    out.confidence_rotation = compute_signal_confidence(delta_snapshot.gyroscope_z_calibrated_mdps, gyro_z_noise_limit_mdps);
    out.has_motion_model = true;
    return true;
  }
}


