#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/api/encoder_api.hpp"

namespace encoder_input_storage
{
  constexpr std::size_t k_wheel_count = 4u;

  struct wheel_encoder_sample
  {
    std::uint16_t angle_raw_12bit;
    std::uint32_t sample_id;
    encoder_api::encoder_status status;
    std::uint32_t time_ms;
  };
  using wheel_sample_array = std::array<wheel_encoder_sample, k_wheel_count>;

  struct encoder_snapshot
  {
    std::uint32_t previous_tick_id = 0u;
    std::uint32_t current_tick_id = 0u;
    wheel_sample_array previous_wheels = {};
    wheel_sample_array current_wheels = {};
    bool has_previous = false;
    bool has_current = false;
  };

  void reset(encoder_snapshot &state);
  bool sample_from_encoder_api(encoder_snapshot &state, std::uint32_t tick_id);
  bool read_previous(const encoder_snapshot &state, wheel_sample_array &out_wheels, std::uint32_t &out_tick_id);
  bool read_current(const encoder_snapshot &state, wheel_sample_array &out_wheels, std::uint32_t &out_tick_id);
}
