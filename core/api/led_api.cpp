#include "core/api/led_api.hpp"

#include "core/system_select/system_select.hpp"

namespace
{
  led_api::backend_operation global_backend = {};
  bool global_backend_ready = false;
}

namespace led_api
{
  void init(const led_output *outputs, std::size_t count)
  {
    global_backend = {};
    global_backend_ready = false;

    if (outputs == nullptr || count == 0U)
    {
      return;
    }

    system_select::select_led_backend(global_backend);

    if (global_backend.init_fn == nullptr || global_backend.set_fn == nullptr || global_backend.toggle_fn == nullptr || global_backend.read_state_fn == nullptr)
    {
      return;
    }

    global_backend.init_fn(outputs, count);
    global_backend_ready = true;
  }

  bool set(std::uint8_t led_id, bool is_on)
  {
    if (!global_backend_ready || global_backend.set_fn == nullptr)
    {
      return false;
    }

    return global_backend.set_fn(led_id, is_on);
  }

  bool toggle(std::uint8_t led_id)
  {
    if (!global_backend_ready || global_backend.toggle_fn == nullptr)
    {
      return false;
    }

    return global_backend.toggle_fn(led_id);
  }

  bool read_state(std::uint8_t led_id, led_state &out)
  {
    if (!global_backend_ready || global_backend.read_state_fn == nullptr)
    {
      out = {};
      out.status = led_status::no_signal;
      return false;
    }

    return global_backend.read_state_fn(led_id, out);
  }
}
