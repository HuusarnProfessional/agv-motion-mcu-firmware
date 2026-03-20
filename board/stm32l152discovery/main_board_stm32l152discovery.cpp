#include "board/stm32l152discovery/main_board_stm32l152discovery.hpp"

#include "app/app_entry.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
  static constexpr app_drive_defaults k_drive_defaults =
  {
    63,
    164
  };
}

extern "C" void board_init(void)
{
  board_stm32l152discovery_init_motors();
  board_stm32l152discovery_init_encoders();
  board_stm32l152discovery_init_imu();
  board_stm32l152discovery_init_voltage_monitor();
  board_stm32l152discovery_init_obstacle();
  board_stm32l152discovery_init_comm_uart();
  app_init();
  app_apply_drive_defaults(&k_drive_defaults);
}

extern "C" void board_tick(void)
{
  app_step(HAL_GetTick());
}
