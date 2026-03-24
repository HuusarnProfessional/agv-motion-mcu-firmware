#include "core/control/imu_calibration/imu_calibration_stop_and_sample_tare.hpp"
#include "core/control/imu_calibration/imu_calibration_tuning.hpp"

#include "core/api/imu_api.hpp"
#include "core/api/motor_api.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include <cmath>

namespace
{
  constexpr std::uint8_t k_motor_count = 4u;
  constexpr std::int64_t k_tare_sample_count = 60000LL;
  constexpr std::int64_t k_tare_samples_per_tick = 100LL;
}

void stop_motor()
{
  for (std::uint8_t motor_id = 0u; motor_id < k_motor_count; ++motor_id)
  {
    motor_api::set_u(motor_id, 0);
  }
}

bool is_still_from_encoder_model(const local_positioning::state &encoder_model_state)
{
  const motion_model_encoders::motion_model_snapshot &motion = encoder_model_state.encoder_motion_snapshot;

  if (!motion.has_motion_model)
  {
    return false;
  }

  if (std::abs(motion.translation) > imu_calibration_tuning::k_still_translation_limit_um)
  {
    return false;
  }

  if (std::abs(motion.rotation) > imu_calibration_tuning::k_still_rotation_limit_urad)
  {
    return false;
  }

  return true;
}

bool tick_build_tare_values(imu_tare_step_state &tare_step_state, std::uint8_t imu_id, imu_api::imu_tare_values &out_tare)
{
  if (tare_step_state.done)
  {
    return true;
  }

  for (std::int64_t i = 0; i < k_tare_samples_per_tick; ++i)
  {
    if (tare_step_state.valid_sample_count >= k_tare_sample_count)
    {
      break;
    }

    imu_api::imu_sample sample = {};
    if (!imu_api::read_sample(imu_id, sample))
    {
      continue;
    }

    tare_step_state.gyroscope_x_sum += sample.gyroscope_x_mdps;
    tare_step_state.gyroscope_y_sum += sample.gyroscope_y_mdps;
    tare_step_state.gyroscope_z_sum += sample.gyroscope_z_mdps;
    tare_step_state.accelerometer_x_sum += sample.accelerometer_x_mg;
    tare_step_state.accelerometer_y_sum += sample.accelerometer_y_mg;
    tare_step_state.accelerometer_z_sum += sample.accelerometer_z_mg;
    tare_step_state.magnetometer_x_sum += sample.magnetometer_x_mgauss;
    tare_step_state.magnetometer_y_sum += sample.magnetometer_y_mgauss;
    tare_step_state.magnetometer_z_sum += sample.magnetometer_z_mgauss;
    ++tare_step_state.valid_sample_count;
  }

  if (tare_step_state.valid_sample_count < k_tare_sample_count)
  {
    return false;
  }

  out_tare = {};
  out_tare.gyroscope_x_mdps = static_cast<std::int32_t>(tare_step_state.gyroscope_x_sum / tare_step_state.valid_sample_count);
  out_tare.gyroscope_y_mdps = static_cast<std::int32_t>(tare_step_state.gyroscope_y_sum / tare_step_state.valid_sample_count);
  out_tare.gyroscope_z_mdps = static_cast<std::int32_t>(tare_step_state.gyroscope_z_sum / tare_step_state.valid_sample_count);
  out_tare.accelerometer_x_mg = static_cast<std::int32_t>(tare_step_state.accelerometer_x_sum / tare_step_state.valid_sample_count);
  out_tare.accelerometer_y_mg = static_cast<std::int32_t>(tare_step_state.accelerometer_y_sum / tare_step_state.valid_sample_count);
  out_tare.accelerometer_z_mg = static_cast<std::int32_t>(tare_step_state.accelerometer_z_sum / tare_step_state.valid_sample_count);
  out_tare.magnetometer_x_mgauss = static_cast<std::int32_t>(tare_step_state.magnetometer_x_sum / tare_step_state.valid_sample_count);
  out_tare.magnetometer_y_mgauss = static_cast<std::int32_t>(tare_step_state.magnetometer_y_sum / tare_step_state.valid_sample_count);
  out_tare.magnetometer_z_mgauss = static_cast<std::int32_t>(tare_step_state.magnetometer_z_sum / tare_step_state.valid_sample_count);
  out_tare.has_tare = true;
  tare_step_state.done = true;
  return true;
}
