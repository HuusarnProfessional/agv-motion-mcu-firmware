#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/obstacle_api.hpp"

namespace obstacle_hcsr04_impl
{
  enum class sample_status : std::uint8_t
  {
    ok = 0,
    timeout,
    stale,
    no_signal,
    out_of_range,
    invalid_id
  };

  struct sample
  {
    std::uint32_t distance_mm;
    std::uint32_t time_ms;
    sample_status status;
  };

  void init(const obstacle_api::obstacle_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t sensor_id, sample &out);
}
