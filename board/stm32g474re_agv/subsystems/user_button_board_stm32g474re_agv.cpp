#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/button_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
  static const platform_stm32_hal::gpio_pin_handle k_button_gpio[] = { { static_cast<void *>(user_button__GPIO_EXTI13_GPIO_Port), user_button__GPIO_EXTI13_Pin } };
  static const button_api::gpio_operations k_button_gpio_ops = { platform_stm32_hal::button_register_event_source, platform_stm32_hal::button_read_gpio_level, platform_stm32_hal::button_read_gpio_event };
  static const button_api::button_input k_buttons[] = { { static_cast<void *>(const_cast<platform_stm32_hal::gpio_pin_handle *>(&k_button_gpio[0])), true, &k_button_gpio_ops } };

  static constexpr std::size_t k_button_count = sizeof(k_buttons) / sizeof(k_buttons[0]);
}

void board_stm32g474re_agv_init_buttons(void)
{
  button_api::init(k_buttons, k_button_count);
}
