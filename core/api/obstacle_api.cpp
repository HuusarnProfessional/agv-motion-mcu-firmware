#include "core/api/obstacle_api.hpp"

#include "core/system_select/system_select.hpp"

namespace
{
  obstacle_api::backend_operation g_backend = {};
  bool g_backend_ready = false;
}

namespace obstacle_api
{
  void init(const obstacle_input *inputs, std::size_t count)
  {
    g_backend = {};
    g_backend_ready = false;

    if (inputs == nullptr || count == 0u)
    {
      return;
    }

    system_select::select_obstacle_backend(g_backend);

    if (g_backend.init_fn == nullptr || g_backend.read_sample_fn == nullptr)
    {
      return;
    }

    g_backend.init_fn(inputs, count);
    g_backend_ready = true;
  }

  bool read_sample(std::uint8_t sensor_id, obstacle_sample &out)
  {
    if (!g_backend_ready)
    {
      out = {};
      out.status = obstacle_status::no_signal;
      return false;
    }

    return g_backend.read_sample_fn(sensor_id, out);
  }
}
