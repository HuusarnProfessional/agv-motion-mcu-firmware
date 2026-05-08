#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"

namespace imu_input_storage
{
  struct gyro_input_snapshot
  {
    std::uint32_t previous_tick_id = 0u;
    std::uint32_t current_tick_id = 0u;
    std::uint32_t previous_time_ms = 0u;
    std::uint32_t current_time_ms = 0u;
    std::uint32_t previous_sample_id = 0u;
    std::uint32_t current_sample_id = 0u;
    std::int32_t previous_gyroscope_z_calibrated_mdps = 0;
    std::int32_t current_gyroscope_z_calibrated_mdps = 0;
    bool has_current_sample = false;
    bool has_previous_sample = false;
    bool has_fresh_sample = false;
    bool can_estimate_delta = false;
  };

  struct accelerometer_input_snapshot
  {
    std::uint32_t previous_tick_id = 0u;
    std::uint32_t current_tick_id = 0u;
    std::uint32_t previous_time_ms = 0u;
    std::uint32_t current_time_ms = 0u;
    std::uint32_t previous_sample_id = 0u;
    std::uint32_t current_sample_id = 0u;
    std::int32_t previous_accelerometer_x_calibrated_mg = 0;
    std::int32_t previous_accelerometer_y_calibrated_mg = 0;
    std::int32_t previous_accelerometer_z_calibrated_mg = 0;
    std::int32_t current_accelerometer_x_calibrated_mg = 0;
    std::int32_t current_accelerometer_y_calibrated_mg = 0;
    std::int32_t current_accelerometer_z_calibrated_mg = 0;
    bool has_current_sample = false;
    bool has_previous_sample = false;
    bool has_fresh_sample = false;
    bool can_estimate_delta = false;
  };

  struct imu_sample_snapshot
  {
    std::uint8_t imu_id = 0u;
    imu_api::imu_sample latest_sample = {};
    std::uint32_t latest_time_ms = 0u;
    bool has_input = false;
    gyro_input_snapshot gyro = {};
    accelerometer_input_snapshot accelerometer = {};
  };

  void reset(imu_sample_snapshot &state);
  bool sample_from_imu_api(imu_sample_snapshot &state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms);
}
