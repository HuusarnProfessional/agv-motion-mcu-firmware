#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/led_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
  static const platform_stm32_hal::gpio_pin_handle k_led_gpio[] = { { static_cast<void *>(led_01___GPIO_Output_GPIO_Port), led_01___GPIO_Output_Pin } };
  static const led_api::gpio_operations k_led_gpio_ops = { platform_stm32_hal::led_write_gpio_level, platform_stm32_hal::led_read_gpio_level };
  static const led_api::led_output k_leds[] = { { static_cast<void *>(const_cast<platform_stm32_hal::gpio_pin_handle *>(&k_led_gpio[0])), true, &k_led_gpio_ops } };

  static constexpr std::size_t k_led_count = sizeof(k_leds) / sizeof(k_leds[0]);
}

void board_stm32g474re_agv_init_leds(void)
{
  led_api::init(k_leds, k_led_count);
}
