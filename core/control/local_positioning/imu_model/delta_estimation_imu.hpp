#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"

namespace delta_estimation_imu
{
  struct delta_snapshot
  {
    std::uint32_t tick_id = 0u;
    std::uint32_t time_ms = 0u;
    std::int32_t gyroscope_z_mdps = 0;
    imu_api::gyroscope_status gyroscope_state = imu_api::gyroscope_status::stale;
    std::int32_t accelerometer_x_mg = 0;
    std::int32_t accelerometer_y_mg = 0;
    std::int32_t accelerometer_z_mg = 0;
    imu_api::accelerometer_status accelerometer_state = imu_api::accelerometer_status::stale;
    bool has_delta = false;
  };

  void reset(delta_snapshot &state);
  bool sample_from_imu_api(std::uint8_t imu_id, std::uint32_t tick_id, delta_snapshot &out);
}
