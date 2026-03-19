#include "board/stm32l152discovery/main_board_stm32l152discovery.hpp"

#include "core/api/imu_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern SPI_HandleTypeDef hspi2;
}

namespace
{
  static SPI_HandleTypeDef *k_spi2 = &hspi2;

  static platform_stm32_hal::imu_spi_bus_handle k_imu_spi_bus =
  {
    static_cast<void *>(k_spi2),
    { static_cast<void *>(IMU_CS_AG_GPIO_Port), IMU_CS_AG_Pin },
    { static_cast<void *>(IMU_CS_M_GPIO_Port), IMU_CS_M_Pin },
    10U
  };

  static const imu_api::imu_input k_imus[] =
  {
    {
      static_cast<void *>(&k_imu_spi_bus),
      platform_stm32_hal::imu_read_register_spi,
      platform_stm32_hal::imu_write_register_spi
    }
  };

  static constexpr std::size_t k_imu_count = sizeof(k_imus) / sizeof(k_imus[0]);
}

void board_stm32l152discovery_init_imu(void)
{
  imu_api::init(k_imus, k_imu_count);
}
