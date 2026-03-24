#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"

namespace imu_input_storage
{
  struct imu_sample_snapshot
  {
    std::uint32_t previous_tick_id = 0u;
    std::uint32_t current_tick_id = 0u;
    std::uint32_t previous_time_ms = 0u;
    std::uint32_t current_time_ms = 0u;
    imu_api::imu_sample previous_sample = {};
    imu_api::imu_sample current_sample = {};
    bool has_input = false;
  };

  void reset(imu_sample_snapshot &state);
  bool sample_from_imu_api(imu_sample_snapshot &state, std::uint8_t imu_id, std::uint32_t tick_id, std::uint32_t now_ms);
}
