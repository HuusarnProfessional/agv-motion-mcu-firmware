#include "board/stm32l152discovery/main_board_stm32l152discovery.hpp"

#include "core/api/obstacle_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
  static const obstacle_api::ultrasonic_operations k_ultrasonic_ops =
  {
    platform_stm32_hal::obstacle_read_echo_pulse_us
  };

  static const platform_stm32_hal::obstacle_pin_map k_obstacle_pin_map[] =
  {
    // todo: replace with real trigger/echo pin mapping
    { { nullptr, 0U }, { nullptr, 0U } }, // front
    { { nullptr, 0U }, { nullptr, 0U } }  // rear
  };

  static platform_stm32_hal::obstacle_hcsr04_handle k_obstacle_handle =
  {
    k_obstacle_pin_map,
    sizeof(k_obstacle_pin_map) / sizeof(k_obstacle_pin_map[0]),
    30000U
  };

  static const obstacle_api::obstacle_input k_obstacle_sensors[] =
  {
    // todo: replace channel/handle when trigger+echo pin mapping is ready
    { static_cast<void *>(&k_obstacle_handle), 0u, &k_ultrasonic_ops }, // front
    { static_cast<void *>(&k_obstacle_handle), 1u, &k_ultrasonic_ops }  // rear
  };

  static constexpr std::size_t k_obstacle_sensor_count = sizeof(k_obstacle_sensors) / sizeof(k_obstacle_sensors[0]);
}

void board_stm32l152discovery_init_obstacle(void)
{
  obstacle_api::init(k_obstacle_sensors, k_obstacle_sensor_count);
}
