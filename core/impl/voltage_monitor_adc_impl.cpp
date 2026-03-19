#include "core/impl/voltage_monitor_adc_impl.hpp"

namespace
{
  const voltage_monitor_api::voltage_input *g_input = nullptr;
  std::size_t g_input_count = 0u;

  // update after resistor measurement
  constexpr std::uint32_t k_divider_r_top_ohm = 10000u;
  constexpr std::uint32_t k_divider_r_bottom_ohm = 10000u;

  // hardware adc configuration
  constexpr std::uint32_t k_adc_reference_mv = 3300u;
  constexpr std::uint32_t k_adc_max_count = 4095u;

  std::uint32_t raw_to_adc_mv(std::uint16_t raw_adc)
  {
    return (static_cast<std::uint32_t>(raw_adc) * k_adc_reference_mv) / k_adc_max_count;
  }

  std::uint32_t adc_mv_to_input_mv(std::uint32_t adc_mv)
  {
    return (adc_mv * (k_divider_r_top_ohm + k_divider_r_bottom_ohm)) / k_divider_r_bottom_ohm;
  }
}

namespace voltage_monitor_adc_impl
{
  void init(const voltage_monitor_api::voltage_input *input, std::size_t count)
  {
    if (input == nullptr || count == 0u)
    {
      g_input = nullptr;
      g_input_count = 0u;
      return;
    }

    g_input = input;
    g_input_count = count;
  }

  bool read_sample(std::uint8_t sensor_id, sample &out)
  {
    out.raw_adc = 0u;
    out.voltage_mv = 0u;
    out.time_ms = 0u;
    out.status = sample_status::stale;

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

    const voltage_monitor_api::voltage_input &selected_input = g_input[sensor_id];

    if (selected_input.platform_operations == nullptr)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    if (selected_input.platform_operations->read_adc_raw == nullptr)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    std::uint16_t raw_adc = 0u;
    std::uint32_t time_ms = 0u;

    // fetches adc raw value and time from board
    const bool ok = selected_input.platform_operations->read_adc_raw(selected_input.platform_handle, selected_input.channel, raw_adc, time_ms);
    
    if (!ok)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    out.raw_adc = raw_adc;
    out.time_ms = time_ms;

    const std::uint32_t adc_mv = raw_to_adc_mv(raw_adc);
    const std::uint32_t input_mv = adc_mv_to_input_mv(adc_mv);


    out.voltage_mv = input_mv;
    out.status = sample_status::ok;
    return true;
  }
}
