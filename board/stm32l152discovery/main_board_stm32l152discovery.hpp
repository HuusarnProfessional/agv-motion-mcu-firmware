#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void board_init(void);
void board_tick(void);

#ifdef __cplusplus
}

void board_stm32l152discovery_init_motors(void);
void board_stm32l152discovery_init_encoders(void);
void board_stm32l152discovery_init_imu(void);
void board_stm32l152discovery_init_voltage_monitor(void);
void board_stm32l152discovery_init_obstacle(void);
void board_stm32l152discovery_init_comm_uart(void);
#endif
