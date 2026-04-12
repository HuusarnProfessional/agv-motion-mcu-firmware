#include "core/system_select/system_select.hpp"

#include "core/api/led_api.hpp"
#include "core/impl/led_gpio_impl.hpp"

namespace
{
  void init_led_gpio(const led_api::led_output *outputs, std::size_t count)
  {
    led_gpio_impl::init(outputs, count);
  }

  led_api::led_status map_led_status(led_gpio_impl::state_status status)
  {
    switch (status)
    {
      case led_gpio_impl::state_status::ok:
        return led_api::led_status::ok;
      case led_gpio_impl::state_status::no_signal:
        return led_api::led_status::no_signal;
      case led_gpio_impl::state_status::invalid_id:
        return led_api::led_status::invalid_id;
      default:
        return led_api::led_status::no_signal;
    }
  }

  bool set_led_gpio(std::uint8_t led_id, bool is_on)
  {
    return led_gpio_impl::set(led_id, is_on);
  }

  bool toggle_led_gpio(std::uint8_t led_id)
  {
    return led_gpio_impl::toggle(led_id);
  }

  bool read_led_gpio_state(std::uint8_t led_id, led_api::led_state &out)
  {
    led_gpio_impl::state impl_state = {};
    const bool ok = led_gpio_impl::read_state(led_id, impl_state);

    out.is_on = impl_state.is_on;
    out.status = map_led_status(impl_state.status);
    return ok;
  }
}
