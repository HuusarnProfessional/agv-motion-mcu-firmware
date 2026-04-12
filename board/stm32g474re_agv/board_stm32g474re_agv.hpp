#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void board_init(void);
void board_tick(void);

#ifdef __cplusplus
}

void board_stm32g474re_agv_init_motors(void);
void board_stm32g474re_agv_init_encoders(void);
void board_stm32g474re_agv_init_imu(void);
void board_stm32g474re_agv_init_voltage_monitor(void);
void board_stm32g474re_agv_init_obstacle(void);
void board_stm32g474re_agv_init_comm_uart(void);
void board_stm32g474re_agv_init_leds(void);
void board_stm32g474re_agv_init_buttons(void);
#endif
