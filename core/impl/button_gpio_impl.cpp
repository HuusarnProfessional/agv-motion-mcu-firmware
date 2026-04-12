#include "core/impl/button_gpio_impl.hpp"

namespace
{
  constexpr std::size_t k_max_buttons = 16U;

  struct button_runtime
  {
    bool is_pressed = false;
    bool was_pressed_since_last_read = false;
    bool was_released_since_last_read = false;
    std::uint32_t time_ms = 0U;
    bool has_signal = false;
  };

  const button_api::button_input *global_input = nullptr;
  std::size_t global_input_count = 0U;
  button_runtime global_button_runtime[k_max_buttons] = {};

  void clear_global_state(void)
  {
    global_input = nullptr;
    global_input_count = 0U;

    for (std::size_t button_index = 0U; button_index < k_max_buttons; ++button_index)
    {
      global_button_runtime[button_index] = {};
    }
  }

  bool validate_input(const button_api::button_input *input, std::size_t count)
  {
    if (input == nullptr || count == 0U || count > k_max_buttons)
    {
      return false;
    }

    for (std::size_t button_index = 0U; button_index < count; ++button_index)
    {
      const button_api::gpio_operations *selected_operations = input[button_index].platform_operations;

      if (selected_operations == nullptr)
      {
        return false;
      }

      if (selected_operations->register_event_source == nullptr || selected_operations->read_gpio_level == nullptr || selected_operations->read_gpio_event == nullptr)
      {
        return false;
      }
    }

    return true;
  }

  bool convert_level_to_pressed(const button_api::button_input &selected_button, bool is_high)
  {
    if (selected_button.pressed_is_high)
    {
      return is_high;
    }

    return !is_high;
  }

  void apply_gpio_event(const button_api::button_input &selected_button, button_runtime &selected_button_runtime, const button_api::gpio_event &selected_event)
  {
    const bool is_pressed = convert_level_to_pressed(selected_button, selected_event.is_high);

    if (is_pressed != selected_button_runtime.is_pressed)
    {
      if (is_pressed)
      {
        selected_button_runtime.was_pressed_since_last_read = true;
      }
      else
      {
        selected_button_runtime.was_released_since_last_read = true;
      }
    }

    selected_button_runtime.is_pressed = is_pressed;
    selected_button_runtime.time_ms = selected_event.time_ms;
    selected_button_runtime.has_signal = true;
  }

  void initialize_button_runtime(const button_api::button_input &selected_button, button_runtime &selected_button_runtime)
  {
    bool is_high = false;

    if (!selected_button.platform_operations->read_gpio_level(selected_button.platform_handle, is_high))
    {
      return;
    }

    selected_button_runtime.is_pressed = convert_level_to_pressed(selected_button, is_high);
    selected_button_runtime.time_ms = 0U;
    selected_button_runtime.has_signal = true;
  }

  void update_pending_events(void)
  {
    if (global_input == nullptr)
    {
      return;
    }

    for (std::size_t button_index = 0U; button_index < global_input_count; ++button_index)
    {
      const button_api::button_input &selected_button = global_input[button_index];
      button_runtime &selected_button_runtime = global_button_runtime[button_index];

      while (true)
      {
        button_api::gpio_event selected_event = {};

        if (!selected_button.platform_operations->read_gpio_event(selected_button.platform_handle, selected_event))
        {
          selected_button_runtime.has_signal = false;
          break;
        }

        if (selected_event.status == button_api::gpio_event_status::none)
        {
          break;
        }

        if (selected_event.status == button_api::gpio_event_status::invalid_id)
        {
          selected_button_runtime.has_signal = false;
          break;
        }

        apply_gpio_event(selected_button, selected_button_runtime, selected_event);
      }
    }
  }
}

namespace button_gpio_impl
{
  void init(const button_api::button_input *inputs, std::size_t count)
  {
    clear_global_state();

    if (!validate_input(inputs, count))
    {
      return;
    }

    global_input = inputs;
    global_input_count = count;

    for (std::size_t button_index = 0U; button_index < count; ++button_index)
    {
      if (!inputs[button_index].platform_operations->register_event_source(inputs[button_index].platform_handle))
      {
        clear_global_state();
        return;
      }

      initialize_button_runtime(inputs[button_index], global_button_runtime[button_index]);
    }
  }

  bool read_sample(std::uint8_t button_id, sample &out)
  {
    out = {};

    if (global_input == nullptr || global_input_count == 0U)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    if (button_id >= global_input_count)
    {
      out.status = sample_status::invalid_id;
      return false;
    }

    update_pending_events();

    button_runtime &selected_button_runtime = global_button_runtime[button_id];

    if (!selected_button_runtime.has_signal)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    out.is_pressed = selected_button_runtime.is_pressed;
    out.was_pressed_since_last_read = selected_button_runtime.was_pressed_since_last_read;
    out.was_released_since_last_read = selected_button_runtime.was_released_since_last_read;
    out.time_ms = selected_button_runtime.time_ms;
    out.status = sample_status::ok;

    selected_button_runtime.was_pressed_since_last_read = false;
    selected_button_runtime.was_released_since_last_read = false;
    return true;
  }
}
