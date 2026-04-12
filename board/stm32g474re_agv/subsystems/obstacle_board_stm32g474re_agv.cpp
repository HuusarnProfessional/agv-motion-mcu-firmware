#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/obstacle_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
#include "tim.h"
}

namespace
{
  static const obstacle_api::ultrasonic_operations k_ultrasonic_ops = { platform_stm32_hal::obstacle_register_event_callback, platform_stm32_hal::obstacle_set_trigger_level, platform_stm32_hal::obstacle_schedule_alarm_us, platform_stm32_hal::obstacle_cancel_alarm, platform_stm32_hal::obstacle_read_time_ms };

  static const platform_stm32_hal::obstacle_pin_map k_obstacle_pin_map[] =
  {
    { { static_cast<void *>(hc_f_trig__GPIO_OUTPUT_GPIO_Port), hc_f_trig__GPIO_OUTPUT_Pin }, { static_cast<void *>(hc_f_echo__GPIO_EXTI0_GPIO_Port), hc_f_echo__GPIO_EXTI0_Pin } },
    { { static_cast<void *>(hc_b_trig__GPIO_OUTPUT_GPIO_Port), hc_b_trig__GPIO_OUTPUT_Pin }, { static_cast<void *>(hc_b_echo_GPIO_EXTI11_GPIO_Port), hc_b_echo_GPIO_EXTI11_Pin } }
  };

  static platform_stm32_hal::obstacle_hcsr04_handle k_obstacle_handle = { k_obstacle_pin_map, sizeof(k_obstacle_pin_map) / sizeof(k_obstacle_pin_map[0]), static_cast<void *>(&htim6) };

  static const obstacle_api::obstacle_input k_obstacle_sensors[] =
  {
    { static_cast<void *>(&k_obstacle_handle), 0u, 30000U, &k_ultrasonic_ops },
    { static_cast<void *>(&k_obstacle_handle), 1u, 30000U, &k_ultrasonic_ops }
  };

  static constexpr std::size_t k_obstacle_sensor_count = sizeof(k_obstacle_sensors) / sizeof(k_obstacle_sensors[0]);
}

void board_stm32g474re_agv_init_obstacle(void)
{
  obstacle_api::init(k_obstacle_sensors, k_obstacle_sensor_count);
}
