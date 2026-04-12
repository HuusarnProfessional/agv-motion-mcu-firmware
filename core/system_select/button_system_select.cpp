#include "core/system_select/system_select.hpp"

#include "core/api/button_api.hpp"
#include "core/impl/button_gpio_impl.hpp"

namespace
{
  void init_button_gpio(const button_api::button_input *inputs, std::size_t count)
  {
    button_gpio_impl::init(inputs, count);
  }

  button_api::button_status map_button_status(button_gpio_impl::sample_status status)
  {
    switch (status)
    {
      case button_gpio_impl::sample_status::ok:
        return button_api::button_status::ok;
      case button_gpio_impl::sample_status::no_signal:
        return button_api::button_status::no_signal;
      case button_gpio_impl::sample_status::invalid_id:
        return button_api::button_status::invalid_id;
      default:
        return button_api::button_status::no_signal;
    }
  }

  bool read_button_gpio(std::uint8_t button_id, button_api::button_sample &out)
  {
    button_gpio_impl::sample impl_sample = {};
    const bool ok = button_gpio_impl::read_sample(button_id, impl_sample);

    out.is_pressed = impl_sample.is_pressed;
    out.was_pressed_since_last_read = impl_sample.was_pressed_since_last_read;
    out.was_released_since_last_read = impl_sample.was_released_since_last_read;
    out.time_ms = impl_sample.time_ms;
    out.status = map_button_status(impl_sample.status);

    return ok;
  }
}
