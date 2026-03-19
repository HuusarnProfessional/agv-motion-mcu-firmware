#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/voltage_monitor_api.hpp"

namespace voltage_monitor_adc_impl
{
  enum class sample_status : std::uint8_t
  {
    ok = 0,
    no_signal,
    stale,
    invalid_id
  };

  struct sample
  {
    std::uint16_t raw_adc;
    std::uint32_t voltage_mv;
    std::uint32_t time_ms;
    sample_status status;
  };

  void init(const voltage_monitor_api::voltage_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t sensor_id, sample &out);
}
