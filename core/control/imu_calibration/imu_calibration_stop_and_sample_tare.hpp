#pragma once

#include <array>
#include <cstdint>
#include "core/api/imu_api.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"

struct imu_noise_p_proc_estimator_state
{
  std::array<double, 5> marker_heights = {};
  std::array<double, 5> marker_positions = {};
  std::array<double, 5> desired_positions = {};
  std::array<double, 5> desired_increments = {};
  std::array<double, 5> initial_samples = {};
  std::uint8_t initial_sample_count = 0U;
  bool is_initialized = false;
};

struct imu_tare_step_state
{
  std::int64_t gyroscope_x_sum = 0;
  std::int64_t gyroscope_y_sum = 0;
  std::int64_t gyroscope_z_sum = 0;
  std::int64_t accelerometer_x_sum = 0;
  std::int64_t accelerometer_y_sum = 0;
  std::int64_t accelerometer_z_sum = 0;
  std::uint32_t sample_count = 0U;
  std::uint32_t tick_count = 0U;
  std::uint32_t last_gyroscope_sample_id = 0U;
  std::uint32_t last_accelerometer_sample_id = 0U;
  bool has_last_gyroscope_sample_id = false;
  bool has_last_accelerometer_sample_id = false;
  imu_noise_p_proc_estimator_state gyroscope_x_noise_state = {};
  imu_noise_p_proc_estimator_state gyroscope_y_noise_state = {};
  imu_noise_p_proc_estimator_state gyroscope_z_noise_state = {};
  imu_noise_p_proc_estimator_state accelerometer_x_noise_state = {};
  imu_noise_p_proc_estimator_state accelerometer_y_noise_state = {};
  imu_noise_p_proc_estimator_state accelerometer_z_noise_state = {};
  bool failed = false;
  bool done = false;
};

void stop_motor();
bool is_still_from_encoder_model(const encoder_motion::state &encoder_model_state);
bool tick_build_tare_values(imu_tare_step_state &tare_step_state, std::uint8_t imu_id, imu_api::imu_tare_values &out_tare, imu_api::imu_noise_profile &out_noise);
