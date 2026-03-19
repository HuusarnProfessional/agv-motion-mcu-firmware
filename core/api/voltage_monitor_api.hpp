#pragma once

#include <cstddef>
#include <cstdint>

namespace voltage_monitor_api
{
  enum class voltage_status : std::uint8_t
  {
    ok = 0,
    no_signal,
    stale,
    invalid_id
  };

  struct voltage_sample
  {
    std::uint16_t raw_adc;
    std::uint32_t voltage_mv;
    std::uint32_t time_ms;
    voltage_status status;
  };

  using read_adc_raw_fn = bool (*)(void *platform_handle, std::uint8_t channel, std::uint16_t &raw_adc_out, std::uint32_t &time_ms_out);

  //board wrapper
  struct adc_operations
  {
    read_adc_raw_fn read_adc_raw;
  };

  
  struct voltage_input
  {
    void *platform_handle;
    std::uint8_t channel;
    const adc_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const voltage_input *inputs, std::size_t count);
  using backend_read_sample_fn = bool (*)(std::uint8_t sensor_id, voltage_sample &out);

  //api wrapper
  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_read_sample_fn read_sample_fn;
  };

  void init(const voltage_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t sensor_id, voltage_sample &out);
}
