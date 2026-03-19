#include "core/api/motor_api.hpp"
#include "core/system_select/system_select.hpp"

namespace
{
  motor_api::backend_operation g_backend{};
  bool g_backend_ready = false;
}

namespace motor_api
{
  void init(const motor_pwm2 *motors, std::size_t count)
  {
    g_backend = {};
    g_backend_ready = false;

    if (motors == nullptr || count == 0U)
    {
      return;
    }

    system_select::select_motor_backend(g_backend);
    if (g_backend.init_fn == nullptr || g_backend.set_u_fn == nullptr)
    {
      return;
    }

    g_backend.init_fn(motors, count);
    g_backend_ready = true;
  }

  void set_u(std::uint8_t motor_id, std::int16_t u)
  {
    if (!g_backend_ready || g_backend.set_u_fn == nullptr)
    {
      return;
    }

    g_backend.set_u_fn(motor_id, u);
  }
}
