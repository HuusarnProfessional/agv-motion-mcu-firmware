#include "core/control/imu_calibration/imu_calibration_drive_and_sample_alignment.hpp"
#include "core/control/imu_calibration/imu_calibration_tuning.hpp"
#include "core/api/motor_api.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include <cstdlib>

namespace
{
  constexpr std::uint8_t k_motor_count = 4U;

  void set_motor_u_all(std::int16_t u)
  {
    for (std::uint8_t motor_id = 0U; motor_id < k_motor_count; ++motor_id)
    {
      motor_api::set_u(motor_id, u);
    }
  }

  bool has_new_accelerometer_sample(const imu_drive_sample_values &values, const imu_api::imu_sample &sample)
  {
    if (sample.accelerometer_state != imu_api::accelerometer_status::ok)
    {
      return false;
    }

    if (values.has_last_accelerometer_sample_id && sample.accelerometer_sample_id == values.last_accelerometer_sample_id)
    {
      return false;
    }

    return true;
  }

  void add_drive_sample(const imu_api::imu_sample &sample, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &io_values)
  {
    io_values.accelerometer_x_raw_sum += sample.accelerometer_x_raw;
    io_values.accelerometer_y_raw_sum += sample.accelerometer_y_raw;
    io_values.accelerometer_z_raw_sum += sample.accelerometer_z_raw;
    io_values.last_accelerometer_sample_id = sample.accelerometer_sample_id;
    io_values.has_last_accelerometer_sample_id = true;
    ++io_values.sample_count;

    const std::int64_t ax_mg = static_cast<std::int64_t>(sample.accelerometer_x_mg) - tare_values.accelerometer_x_mg;
    const std::int64_t ay_mg = static_cast<std::int64_t>(sample.accelerometer_y_mg) - tare_values.accelerometer_y_mg;
    const std::int64_t az_mg = static_cast<std::int64_t>(sample.accelerometer_z_mg) - tare_values.accelerometer_z_mg;
    io_values.last_acceleration_mg = std::abs(ax_mg) + std::abs(ay_mg) + std::abs(az_mg);

    if (io_values.last_acceleration_mg > io_values.peak_acceleration_mg)
    {
      io_values.peak_acceleration_mg = io_values.last_acceleration_mg;
    }

  }

  bool try_add_drive_sample(std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &io_values)
  {
    imu_api::imu_sample sample = {};

    if (!imu_api::read_sample(imu_id, sample))
    {
      return false;
    }

    if (!has_new_accelerometer_sample(io_values, sample))
    {
      return false;
    }

    add_drive_sample(sample, tare_values, io_values);
    return true;
  }

  bool should_stop_drive_sample(const imu_drive_sample_values &values)
  {
    if (values.translation_um >= imu_calibration_tuning::k_max_translation_um)
    {
      return true;
    }

    if (values.translation_um < imu_calibration_tuning::k_min_translation_before_stop_um)
    {
      return false;
    }

    if (values.sample_count < imu_calibration_tuning::k_drive_min_sample_count)
    {
      return false;
    }

    if (values.peak_acceleration_mg == 0)
    {
      return false;
    }

    return (values.last_acceleration_mg * 4) < values.peak_acceleration_mg;
  }

  void update_drive_translation(const encoder_motion::state &encoder_model_state, imu_drive_sample_values &out_values)
  {
    const motion_model_encoders::motion_model_snapshot &motion = encoder_model_state.encoder_motion_snapshot;

    if (motion.has_motion_model)
    {
      out_values.translation_um += std::abs(motion.translation);
    }
  }

  bool finish_drive_if_needed(imu_drive_sample_step_state &drive_step_state, imu_drive_sample_values &out_values)
  {
    const bool timed_out = drive_step_state.tick_count >= imu_calibration_tuning::k_max_drive_tick_count;
    const bool should_stop = should_stop_drive_sample(out_values);

    if (!timed_out && !should_stop)
    {
      return false;
    }

    set_motor_u_all(0);
    drive_step_state.done = true;

    if (timed_out)
    {
      drive_step_state.failed = true;
    }

    if (out_values.sample_count < imu_calibration_tuning::k_drive_min_sample_count)
    {
      drive_step_state.failed = true;
    }

    return true;
  }

  bool tick_drive_and_sample(imu_drive_sample_step_state &drive_step_state, const encoder_motion::state &encoder_model_state, std::int16_t motor_u, std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &out_values)
  {
    if (drive_step_state.done)
    {
      return true;
    }

    if (!drive_step_state.motor_command_sent)
    {
      out_values = {};
      set_motor_u_all(motor_u);
      drive_step_state.motor_command_sent = true;
    }

    ++drive_step_state.tick_count;
    update_drive_translation(encoder_model_state, out_values);
    try_add_drive_sample(imu_id, tare_values, out_values);
    return finish_drive_if_needed(drive_step_state, out_values);
  }
}

bool tick_drive_forward_and_sample(imu_drive_sample_step_state &drive_step_state, const encoder_motion::state &encoder_model_state, std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &out_values)
{
  return tick_drive_and_sample(drive_step_state, encoder_model_state, imu_calibration_tuning::k_drive_u, imu_id, tare_values, out_values);
}

bool tick_drive_backward_and_sample(imu_drive_sample_step_state &drive_step_state, const encoder_motion::state &encoder_model_state, std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &out_values)
{
  return tick_drive_and_sample(drive_step_state, encoder_model_state, -imu_calibration_tuning::k_drive_u, imu_id, tare_values, out_values);
}
