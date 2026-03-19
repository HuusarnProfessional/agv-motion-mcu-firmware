#include "board/stm32l152discovery/main_board_stm32l152discovery.hpp"

#include "app/app_entry.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
  static void *get_uart_esp32_hal_handle(void)
  {
    const std::uintptr_t uart3_addr = reinterpret_cast<std::uintptr_t>(USART3);
    if (uart3_addr == 0U)
    {
      return nullptr;
    }
    return reinterpret_cast<void *>(uart3_addr);
  }

  // physical dimensions of the agv
  static constexpr app_drive_defaults k_drive_defaults =
  {
    63, // wheel_diameter_mm
    164 // wheel_separation_mm
  };
}

extern "C" void board_init(void)
{
  platform_stm32_hal::set_default_uart_transport_ctx(get_uart_esp32_hal_handle());
  board_stm32l152discovery_init_motors();
  board_stm32l152discovery_init_encoders();
  board_stm32l152discovery_init_imu();
  board_stm32l152discovery_init_voltage_monitor();
  board_stm32l152discovery_init_obstacle();
  app_init();
  app_apply_drive_defaults(&k_drive_defaults);
}

extern "C" void board_tick(void)
{
  app_step(HAL_GetTick());
}
