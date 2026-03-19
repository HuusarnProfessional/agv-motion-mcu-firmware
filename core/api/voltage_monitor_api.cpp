#include "core/api/voltage_monitor_api.hpp"
#include "core/system_select/system_select.hpp"

namespace
{
  voltage_monitor_api::backend_operation g_backend = {};
  bool g_backend_ready = false;
}

namespace voltage_monitor_api
{
  void init(const voltage_input *inputs, std::size_t count)
  {
    g_backend = {};
    g_backend_ready = false;

    if (inputs == nullptr || count == 0u)
    {
      return;
    }

    system_select::select_voltage_monitor_backend(g_backend);

    if (g_backend.init_fn == nullptr || g_backend.read_sample_fn == nullptr)
    {
      return;
    }

    g_backend.init_fn(inputs, count);
    g_backend_ready = true;
  }

  bool read_sample(std::uint8_t sensor_id, voltage_sample &out)
  {
    if (!g_backend_ready || g_backend.read_sample_fn == nullptr)
    {
      out.raw_adc = 0u;
      out.voltage_mv = 0u;
      out.time_ms = 0u;
      out.status = voltage_status::stale;
      return false;
    }

    return g_backend.read_sample_fn(sensor_id, out);
  }
}
