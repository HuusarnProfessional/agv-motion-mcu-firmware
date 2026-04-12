#include "core/impl/led_gpio_impl.hpp"

namespace
{
  constexpr std::size_t k_max_leds = 16U;

  const led_api::led_output *global_output = nullptr;
  std::size_t global_output_count = 0U;

  void clear_global_state(void)
  {
    global_output = nullptr;
    global_output_count = 0U;
  }

  bool validate_output(const led_api::led_output *output, std::size_t count)
  {
    if (output == nullptr || count == 0U || count > k_max_leds)
    {
      return false;
    }

    for (std::size_t led_index = 0U; led_index < count; ++led_index)
    {
      const led_api::gpio_operations *selected_operations = output[led_index].platform_operations;

      if (selected_operations == nullptr || selected_operations->write_gpio_level == nullptr || selected_operations->read_gpio_level == nullptr)
      {
        return false;
      }
    }

    return true;
  }

  bool get_selected_output(std::uint8_t led_id, const led_api::led_output *&selected_output_out)
  {
    if (global_output == nullptr || global_output_count == 0U)
    {
      return false;
    }

    if (led_id >= global_output_count)
    {
      return false;
    }

    selected_output_out = &global_output[led_id];
    return true;
  }

  bool convert_gpio_level_to_is_on(const led_api::led_output &selected_output, bool is_high)
  {
    if (selected_output.active_is_high)
    {
      return is_high;
    }

    return !is_high;
  }

  bool write_selected_output(const led_api::led_output &selected_output, bool is_on)
  {
    bool is_high = !is_on;

    if (selected_output.active_is_high)
    {
      is_high = is_on;
    }

    return selected_output.platform_operations->write_gpio_level(selected_output.platform_handle, is_high);
  }

  bool read_selected_output(const led_api::led_output &selected_output, bool &is_on_out)
  {
    bool is_high = false;

    if (!selected_output.platform_operations->read_gpio_level(selected_output.platform_handle, is_high))
    {
      return false;
    }

    is_on_out = convert_gpio_level_to_is_on(selected_output, is_high);
    return true;
  }
}

namespace led_gpio_impl
{
  void init(const led_api::led_output *outputs, std::size_t count)
  {
    clear_global_state();

    if (!validate_output(outputs, count))
    {
      return;
    }

    global_output = outputs;
    global_output_count = count;
  }

  bool set(std::uint8_t led_id, bool is_on)
  {
    const led_api::led_output *selected_output = nullptr;

    if (!get_selected_output(led_id, selected_output))
    {
      return false;
    }

    return write_selected_output(*selected_output, is_on);
  }

  bool toggle(std::uint8_t led_id)
  {
    const led_api::led_output *selected_output = nullptr;
    bool is_on = false;

    if (!get_selected_output(led_id, selected_output) || !read_selected_output(*selected_output, is_on))
    {
      return false;
    }

    return write_selected_output(*selected_output, !is_on);
  }

  bool read_state(std::uint8_t led_id, state &out)
  {
    out = {};

    const led_api::led_output *selected_output = nullptr;

    if (global_output == nullptr || global_output_count == 0U)
    {
      out.status = state_status::no_signal;
      return false;
    }

    if (!get_selected_output(led_id, selected_output))
    {
      out.status = state_status::invalid_id;
      return false;
    }

    if (!read_selected_output(*selected_output, out.is_on))
    {
      out.status = state_status::no_signal;
      return false;
    }

    out.status = state_status::ok;
    return true;
  }
}
