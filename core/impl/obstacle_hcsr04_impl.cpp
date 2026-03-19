#include "core/impl/obstacle_hcsr04_impl.hpp"

namespace
{
  const obstacle_api::obstacle_input *g_input = nullptr;
  std::size_t g_input_count = 0u;

    constexpr std::uint32_t k_min_distance_mm = 20u;
    constexpr std::uint32_t k_min_valid_pulse_us = (2u * k_min_distance_mm * 1000u) / 343u;

    constexpr std::uint32_t k_max_distance_mm = 4000u;
    constexpr std::uint32_t k_max_valid_pulse_us = (2u * k_max_distance_mm * 1000u) / 343u;

    //values for converting echo pulse width us to distance mm
    constexpr std::uint32_t k_distance_numerator = 343u;
    constexpr std::uint32_t k_distance_denominator = 2000u;

static bool convert_pulse_us_to_distance_mm(std::uint32_t pulse_width_us, std::uint32_t &distance_mm_out)
{
  if (pulse_width_us < k_min_valid_pulse_us || pulse_width_us > k_max_valid_pulse_us)
  {
    return false;
  }

  const std::uint32_t scaled = pulse_width_us * k_distance_numerator;
  const std::uint32_t rounded = scaled + (k_distance_denominator / 2u);
  distance_mm_out = rounded / k_distance_denominator;
  return true;
}



}

namespace obstacle_hcsr04_impl
{
void init(const obstacle_api::obstacle_input *inputs, std::size_t count)
{
  if (inputs == nullptr || count == 0u)
  {
    g_input = nullptr;
    g_input_count = 0u;
    return;
  }

  //check valid sensors
  for (std::size_t i = 0u; i < count; ++i)
  {
    if (inputs[i].platform_operations == nullptr ||
        inputs[i].platform_operations->read_echo_pulse_us == nullptr)
    {
      g_input = nullptr;
      g_input_count = 0u;
      return;
    }
  }

  g_input = inputs;
  g_input_count = count;
}

bool read_sample(std::uint8_t sensor_id, sample &out)
{
  out = {};

  if (g_input == nullptr || g_input_count == 0u)
  {
    out.status = sample_status::no_signal;
    return false;
  }

  if (sensor_id >= g_input_count)
  {
    out.status = sample_status::invalid_id;
    return false;
  }

  const obstacle_api::obstacle_input &sensor = g_input[sensor_id];

  std::uint32_t pulse_width_us = 0u;

  if (!sensor.platform_operations->read_echo_pulse_us(sensor.platform_handle, sensor.channel, pulse_width_us, out.time_ms))
  {
    out.status = sample_status::timeout;
    return false;
  }

  if (!convert_pulse_us_to_distance_mm(pulse_width_us, out.distance_mm))
  {
    out.status = sample_status::out_of_range;
    return false;
  }

  out.status = sample_status::ok;
  return true;
}

}
