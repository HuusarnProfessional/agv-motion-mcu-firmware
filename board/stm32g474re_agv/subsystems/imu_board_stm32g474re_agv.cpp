#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/imu_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern SPI_HandleTypeDef hspi1;
}

namespace
{
  static SPI_HandleTypeDef *k_spi1 = &hspi1;

  static platform_stm32_hal::imu_spi_bus_handle k_imu_spi_bus =
  {
    static_cast<void *>(k_spi1),
    { static_cast<void *>(imu_spi_cs_ag___GPIO_Output_GPIO_Port), imu_spi_cs_ag___GPIO_Output_Pin },
    { nullptr, 0U },
    10U
  };

  static const imu_api::imu_input k_imus[] = { { static_cast<void *>(&k_imu_spi_bus), platform_stm32_hal::imu_read_register_spi, platform_stm32_hal::imu_write_register_spi } };

  static constexpr std::size_t k_imu_count = sizeof(k_imus) / sizeof(k_imus[0]);
}

void board_stm32g474re_agv_init_imu(void)
{
  imu_api::init(k_imus, k_imu_count);
}
