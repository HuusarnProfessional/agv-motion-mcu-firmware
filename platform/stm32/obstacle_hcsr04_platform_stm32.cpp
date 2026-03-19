#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  static bool write_gpio_level(const platform_stm32_hal::gpio_pin_handle &pin, GPIO_PinState level)
  {
    if (pin.port == nullptr)
    {
      return false;
    }

    GPIO_TypeDef *port = static_cast<GPIO_TypeDef *>(pin.port);
    HAL_GPIO_WritePin(port, pin.pin, level);
    return true;
  }

  static bool read_gpio_level(const platform_stm32_hal::gpio_pin_handle &pin, GPIO_PinState &level_out)
  {
    if (pin.port == nullptr)
    {
      return false;
    }

    GPIO_TypeDef *port = static_cast<GPIO_TypeDef *>(pin.port);
    level_out = HAL_GPIO_ReadPin(port, pin.pin);
    return true;
  }
}

namespace platform_stm32_hal
{
  bool obstacle_read_echo_pulse_us(void *platform_handle,
                                   std::uint8_t channel,
                                   std::uint32_t &pulse_width_us_out,
                                   std::uint32_t &time_ms_out)
  {
    pulse_width_us_out = 0U;
    time_ms_out = HAL_GetTick();

    if (platform_handle == nullptr)
    {
      return false;
    }

    obstacle_hcsr04_handle *obstacle_handle = static_cast<obstacle_hcsr04_handle *>(platform_handle);
    if (obstacle_handle->sensors == nullptr || obstacle_handle->sensor_count == 0U)
    {
      return false;
    }

    if (channel >= obstacle_handle->sensor_count)
    {
      return false;
    }

    const obstacle_pin_map &selected_sensor = obstacle_handle->sensors[channel];

    // Keep platform side minimal for now:
    // validate mapped pins and keep timestamping consistent.
    if (!write_gpio_level(selected_sensor.trigger, GPIO_PIN_RESET))
    {
      return false;
    }

    GPIO_PinState echo_level = GPIO_PIN_RESET;
    if (!read_gpio_level(selected_sensor.echo, echo_level))
    {
      return false;
    }

    (void)echo_level;
    (void)obstacle_handle->echo_timeout_us;

    // TODO: implement microsecond trigger + echo timing with a proper us time source.
    return false;
  }
}
