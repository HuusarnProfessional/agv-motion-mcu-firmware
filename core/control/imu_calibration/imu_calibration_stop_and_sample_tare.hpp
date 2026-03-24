#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"

struct imu_tare_step_state
{
  std::int64_t gyroscope_x_sum = 0;
  std::int64_t gyroscope_y_sum = 0;
  std::int64_t gyroscope_z_sum = 0;
  std::int64_t accelerometer_x_sum = 0;
  std::int64_t accelerometer_y_sum = 0;
  std::int64_t accelerometer_z_sum = 0;
  std::int64_t magnetometer_x_sum = 0;
  std::int64_t magnetometer_y_sum = 0;
  std::int64_t magnetometer_z_sum = 0;
  std::int64_t valid_sample_count = 0;
  bool done = false;
};

void stop_motor();
bool is_still_from_encoder_model(const local_positioning::state &encoder_model_state);
bool tick_build_tare_values(imu_tare_step_state &tare_step_state, std::uint8_t imu_id, imu_api::imu_tare_values &out_tare);
