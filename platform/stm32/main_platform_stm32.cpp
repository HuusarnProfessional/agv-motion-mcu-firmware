#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  constexpr std::size_t k_max_gpio_exti_callbacks = 32U;

  struct gpio_exti_callback_runtime
  {
    std::uint16_t gpio_pin = 0U;
    platform_stm32_hal::gpio_exti_callback_fn callback = nullptr;
    void *callback_context = nullptr;
    bool in_use = false;
  };

  gpio_exti_callback_runtime global_gpio_exti_callbacks[k_max_gpio_exti_callbacks] = {};
}

namespace platform_stm32_hal
{
  bool register_gpio_exti_callback(std::uint16_t gpio_pin, gpio_exti_callback_fn callback, void *callback_context)
  {
    if (gpio_pin == 0U || callback == nullptr)
    {
      return false;
    }

    for (std::size_t callback_index = 0U; callback_index < k_max_gpio_exti_callbacks; ++callback_index)
    {
      gpio_exti_callback_runtime &selected_callback_runtime = global_gpio_exti_callbacks[callback_index];

      if (selected_callback_runtime.in_use && selected_callback_runtime.gpio_pin == gpio_pin && selected_callback_runtime.callback == callback && selected_callback_runtime.callback_context == callback_context)
      {
        return true;
      }
    }

    for (std::size_t callback_index = 0U; callback_index < k_max_gpio_exti_callbacks; ++callback_index)
    {
      gpio_exti_callback_runtime &selected_callback_runtime = global_gpio_exti_callbacks[callback_index];

      if (selected_callback_runtime.in_use)
      {
        continue;
      }

      selected_callback_runtime.gpio_pin = gpio_pin;
      selected_callback_runtime.callback = callback;
      selected_callback_runtime.callback_context = callback_context;
      selected_callback_runtime.in_use = true;
      return true;
    }

    return false;
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t gpio_pin)
{
  for (std::size_t callback_index = 0U; callback_index < k_max_gpio_exti_callbacks; ++callback_index)
  {
    const gpio_exti_callback_runtime &selected_callback_runtime = global_gpio_exti_callbacks[callback_index];

    if (!selected_callback_runtime.in_use || selected_callback_runtime.gpio_pin != gpio_pin || selected_callback_runtime.callback == nullptr)
    {
      continue;
    }

    selected_callback_runtime.callback(selected_callback_runtime.callback_context, gpio_pin);
  }
}
