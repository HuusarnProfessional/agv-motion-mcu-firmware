#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace platform_stm32_hal
{
  bool led_write_gpio_level(void *platform_handle, bool is_high)
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

    HAL_GPIO_WritePin(static_cast<GPIO_TypeDef *>(selected_pin->port), selected_pin->pin, is_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
  }

  bool led_read_gpio_level(void *platform_handle, bool &is_high_out)
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

    is_high_out = HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef *>(selected_pin->port), selected_pin->pin) == GPIO_PIN_SET;
    return true;
  }
}
