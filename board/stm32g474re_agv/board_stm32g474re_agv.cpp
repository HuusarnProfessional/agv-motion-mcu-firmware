#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

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
  board_stm32g474re_agv_init_motors();
  board_stm32g474re_agv_init_encoders();
  HAL_Delay(50);
  board_stm32g474re_agv_init_imu();
  board_stm32g474re_agv_init_voltage_monitor();
  board_stm32g474re_agv_init_obstacle();
  board_stm32g474re_agv_init_comm_uart();
  board_stm32g474re_agv_init_leds();
  board_stm32g474re_agv_init_buttons();
  app_init();
  app_apply_drive_defaults(&k_drive_defaults);
}

extern "C" void board_tick(void)
{
  app_step(HAL_GetTick());
}
