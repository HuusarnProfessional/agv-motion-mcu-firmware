#pragma once

#include <cstddef>
#include <cstdint>

namespace obstacle_api
{
  enum class obstacle_status : std::uint8_t
  {
    ok = 0,
    timeout,
    stale,
    no_signal,
    out_of_range,
    invalid_id
  };

  struct obstacle_sample
  {
    std::uint32_t distance_mm;
    std::uint32_t time_ms;
    obstacle_status status;
  };

  using read_echo_pulse_us_fn = bool (*)(void *platform_handle, std::uint8_t channel, std::uint32_t &pulse_width_us_out, std::uint32_t &time_ms_out);

  struct ultrasonic_operations
  {
    read_echo_pulse_us_fn read_echo_pulse_us;
  };

  struct obstacle_input
  {
    void *platform_handle;
    std::uint8_t channel;
    const ultrasonic_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const obstacle_input *inputs, std::size_t count);
  using backend_read_sample_fn = bool (*)(std::uint8_t sensor_id, obstacle_sample &out);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_read_sample_fn read_sample_fn;
  };

  void init(const obstacle_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t sensor_id, obstacle_sample &out);
}

