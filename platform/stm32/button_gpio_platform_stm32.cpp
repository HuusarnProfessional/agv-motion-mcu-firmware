#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  constexpr std::size_t k_max_registered_buttons = 16U;
  constexpr std::size_t k_event_queue_capacity = 8U;

  struct button_runtime
  {
    const platform_stm32_hal::gpio_pin_handle *selected_pin = nullptr;
    button_api::gpio_event event_queue[k_event_queue_capacity] = {};
    std::size_t read_index = 0U;
    std::size_t write_index = 0U;
    std::size_t count = 0U;
    bool in_use = false;
  };

  button_runtime global_button_runtime[k_max_registered_buttons] = {};

  static button_runtime *find_button_runtime(const platform_stm32_hal::gpio_pin_handle *selected_pin)
  {
    for (std::size_t button_index = 0U; button_index < k_max_registered_buttons; ++button_index)
    {
      button_runtime &selected_button_runtime = global_button_runtime[button_index];

      if (selected_button_runtime.in_use && selected_button_runtime.selected_pin == selected_pin)
      {
        return &selected_button_runtime;
      }
    }

    return nullptr;
  }

  static button_runtime *allocate_button_runtime(const platform_stm32_hal::gpio_pin_handle *selected_pin)
  {
    for (std::size_t button_index = 0U; button_index < k_max_registered_buttons; ++button_index)
    {
      button_runtime &selected_button_runtime = global_button_runtime[button_index];

      if (selected_button_runtime.in_use)
      {
        continue;
      }

      selected_button_runtime = {};
      selected_button_runtime.selected_pin = selected_pin;
      selected_button_runtime.in_use = true;
      return &selected_button_runtime;
    }

    return nullptr;
  }

  static button_runtime *get_or_create_button_runtime(const platform_stm32_hal::gpio_pin_handle *selected_pin)
  {
    button_runtime *selected_button_runtime = find_button_runtime(selected_pin);

    if (selected_button_runtime != nullptr)
    {
      return selected_button_runtime;
    }

    return allocate_button_runtime(selected_pin);
  }

  static bool read_gpio_level(const platform_stm32_hal::gpio_pin_handle &selected_pin, bool &is_high_out)
  {
    if (selected_pin.port == nullptr)
    {
      return false;
    }

    is_high_out = HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef *>(selected_pin.port), selected_pin.pin) == GPIO_PIN_SET;
    return true;
  }

  static void push_gpio_event(button_runtime &selected_button_runtime, const button_api::gpio_event &selected_event)
  {
    if (selected_button_runtime.count == k_event_queue_capacity)
    {
      selected_button_runtime.read_index = (selected_button_runtime.read_index + 1U) % k_event_queue_capacity;
      selected_button_runtime.count--;
    }

    selected_button_runtime.event_queue[selected_button_runtime.write_index] = selected_event;
    selected_button_runtime.write_index = (selected_button_runtime.write_index + 1U) % k_event_queue_capacity;
    selected_button_runtime.count++;
  }

  static void handle_gpio_exti(void *callback_context, std::uint16_t gpio_pin)
  {
    button_runtime *selected_button_runtime = static_cast<button_runtime *>(callback_context);

    if (selected_button_runtime == nullptr || !selected_button_runtime->in_use || selected_button_runtime->selected_pin == nullptr || selected_button_runtime->selected_pin->pin != gpio_pin)
    {
      return;
    }

    bool is_high = false;

    if (!read_gpio_level(*selected_button_runtime->selected_pin, is_high))
    {
      return;
    }

    const button_api::gpio_event selected_event = { is_high, HAL_GetTick(), button_api::gpio_event_status::ok };
    push_gpio_event(*selected_button_runtime, selected_event);
  }
}

namespace platform_stm32_hal
{
  bool button_register_event_source(void *platform_handle)
  {
    if (platform_handle == nullptr)
    {
      return false;
    }

    const gpio_pin_handle *selected_pin = static_cast<const gpio_pin_handle *>(platform_handle);

    if (selected_pin->port == nullptr)
    {
      return false;
    }

    button_runtime *selected_button_runtime = get_or_create_button_runtime(selected_pin);

    if (selected_button_runtime == nullptr)
    {
      return false;
    }

    if (selected_button_runtime->count == 0U && selected_button_runtime->read_index == 0U && selected_button_runtime->write_index == 0U)
    {
      return register_gpio_exti_callback(selected_pin->pin, handle_gpio_exti, selected_button_runtime);
    }

    return true;
  }

  bool button_read_gpio_level(void *platform_handle, bool &is_high_out)
  {
    if (platform_handle == nullptr)
    {
      return false;
    }

    const gpio_pin_handle *selected_pin = static_cast<const gpio_pin_handle *>(platform_handle);
    return read_gpio_level(*selected_pin, is_high_out);
  }

  bool button_read_gpio_event(void *platform_handle, button_api::gpio_event &event_out)
  {
    event_out = {};
    event_out.status = button_api::gpio_event_status::none;

    if (platform_handle == nullptr)
    {
      event_out.status = button_api::gpio_event_status::invalid_id;
      return false;
    }

    const gpio_pin_handle *selected_pin = static_cast<const gpio_pin_handle *>(platform_handle);
    button_runtime *selected_button_runtime = find_button_runtime(selected_pin);

    if (selected_button_runtime == nullptr)
    {
      event_out.status = button_api::gpio_event_status::invalid_id;
      return false;
    }

    if (selected_button_runtime->count == 0U)
    {
      return true;
    }

    event_out = selected_button_runtime->event_queue[selected_button_runtime->read_index];
    selected_button_runtime->read_index = (selected_button_runtime->read_index + 1U) % k_event_queue_capacity;
    selected_button_runtime->count--;
    return true;
  }
}
