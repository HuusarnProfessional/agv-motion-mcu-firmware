#include "core/system_select/system_select.hpp"

#include "core/api/voltage_monitor_api.hpp"
#include "core/impl/voltage_monitor_adc_impl.hpp"

namespace
{
  void init_voltage_monitor_adc(const voltage_monitor_api::voltage_input *inputs, std::size_t count)
  {
    voltage_monitor_adc_impl::init(inputs, count);
  }

  voltage_monitor_api::voltage_status map_voltage_status(voltage_monitor_adc_impl::sample_status status)
  {
    switch (status)
    {
      case voltage_monitor_adc_impl::sample_status::ok:
        return voltage_monitor_api::voltage_status::ok;
      case voltage_monitor_adc_impl::sample_status::no_signal:
        return voltage_monitor_api::voltage_status::no_signal;
      case voltage_monitor_adc_impl::sample_status::stale:
        return voltage_monitor_api::voltage_status::stale;
      case voltage_monitor_adc_impl::sample_status::invalid_id:
        return voltage_monitor_api::voltage_status::invalid_id;
      default:
        return voltage_monitor_api::voltage_status::stale;
    }
  }

  bool read_voltage_monitor_adc(std::uint8_t sensor_id, voltage_monitor_api::voltage_sample &out)
  {
    voltage_monitor_adc_impl::sample impl_sample = {};
    const bool ok = voltage_monitor_adc_impl::read_sample(sensor_id, impl_sample);

    out.raw_adc = impl_sample.raw_adc;
    out.voltage_mv = impl_sample.voltage_mv;
    out.time_ms = impl_sample.time_ms;
    out.status = map_voltage_status(impl_sample.status);

    return ok;
  }
}
