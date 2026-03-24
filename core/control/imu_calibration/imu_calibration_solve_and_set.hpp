#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"
#include "core/control/imu_calibration/imu_calibration_drive_and_sample_alignment.hpp"

struct imu_drive_sample_mean_values
{
  std::int32_t gyroscope_x_raw = 0;
  std::int32_t gyroscope_y_raw = 0;
  std::int32_t gyroscope_z_raw = 0;
  std::int32_t accelerometer_x_raw = 0;
  std::int32_t accelerometer_y_raw = 0;
  std::int32_t accelerometer_z_raw = 0;
  std::int32_t magnetometer_x_raw = 0;
  std::int32_t magnetometer_y_raw = 0;
  std::int32_t magnetometer_z_raw = 0;
  bool has_mean = false;
};

void build_mean_values_from_drive_samples(const imu_drive_sample_values &sample_values, imu_drive_sample_mean_values &out_mean_values);
void solve_alignment_matrix(const imu_drive_sample_mean_values &forward_mean_values, const imu_drive_sample_mean_values &backward_mean_values, imu_api::imu_tare_values &io_tare_values);
void build_calibration_profile_from_tare(const imu_api::imu_tare_values &tare_values, imu_api::imu_calibration_profile &out_profile);
void set_calibration_profile_to_imu(std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values);
