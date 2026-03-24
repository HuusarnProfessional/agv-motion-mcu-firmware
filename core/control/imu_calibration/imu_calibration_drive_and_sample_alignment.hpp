#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"

struct imu_drive_sample_values
{
  std::int64_t translation_um = 0;
  std::int64_t peak_block_acceleration_mg = 0;
  std::int64_t last_block_acceleration_mg = 0;

  std::int64_t gyroscope_x_raw_sum = 0;
  std::int64_t gyroscope_y_raw_sum = 0;
  std::int64_t gyroscope_z_raw_sum = 0;
  std::int64_t accelerometer_x_raw_sum = 0;
  std::int64_t accelerometer_y_raw_sum = 0;
  std::int64_t accelerometer_z_raw_sum = 0;
  std::int64_t magnetometer_x_raw_sum = 0;
  std::int64_t magnetometer_y_raw_sum = 0;
  std::int64_t magnetometer_z_raw_sum = 0;

  std::int64_t sample_count = 0;
  bool has_samples = false;
};

struct imu_drive_sample_step_state
{
  bool motor_command_sent = false;
  bool done = false;
  std::uint32_t tick_count = 0u;
};

bool tick_drive_forward_and_sample(imu_drive_sample_step_state &drive_step_state, const local_positioning::state &encoder_model_state, std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &out_values);
bool tick_drive_backward_and_sample(imu_drive_sample_step_state &drive_step_state, const local_positioning::state &encoder_model_state, std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, imu_drive_sample_values &out_values);
