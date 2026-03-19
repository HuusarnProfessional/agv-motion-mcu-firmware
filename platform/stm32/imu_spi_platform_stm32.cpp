#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  static const platform_stm32_hal::gpio_pin_handle &select_chip_select_pin(
      const platform_stm32_hal::imu_spi_bus_handle &bus,
      imu_api::imu_target target)
  {
    if (target == imu_api::imu_target::accelerometer_gyroscope)
    {
      return bus.chip_select_accelerometer_gyroscope;
    }

    return bus.chip_select_magnetometer;
  }

  static bool set_chip_select_level(const platform_stm32_hal::gpio_pin_handle &pin, GPIO_PinState level)
  {
    if (pin.port == nullptr)
    {
      return false;
    }

    GPIO_TypeDef *port = static_cast<GPIO_TypeDef *>(pin.port);
    HAL_GPIO_WritePin(port, pin.pin, level);
    return true;
  }
}

namespace platform_stm32_hal
{
  bool imu_read_register_spi(void *platform_handle,
                             imu_api::imu_target target,
                             std::uint8_t register_address,
                             std::uint8_t *data_out,
                             std::size_t length)
  {
    if (platform_handle == nullptr || data_out == nullptr || length == 0U)
    {
      return false;
    }

    imu_spi_bus_handle *bus = static_cast<imu_spi_bus_handle *>(platform_handle);
    if (bus->spi_handle == nullptr)
    {
      return false;
    }

    SPI_HandleTypeDef *spi_handle = static_cast<SPI_HandleTypeDef *>(bus->spi_handle);
    const std::uint32_t timeout_ms = (bus->transfer_timeout_ms == 0U) ? 10U : bus->transfer_timeout_ms;

    std::uint8_t command = static_cast<std::uint8_t>(register_address | 0x80U);
    if (length > 1U)
    {
      command = static_cast<std::uint8_t>(command | 0x40U);
    }

    const gpio_pin_handle &chip_select_pin = select_chip_select_pin(*bus, target);
    if (!set_chip_select_level(chip_select_pin, GPIO_PIN_RESET))
    {
      return false;
    }

    const HAL_StatusTypeDef tx_status = HAL_SPI_Transmit(spi_handle, &command, 1U, timeout_ms);
    const HAL_StatusTypeDef rx_status = HAL_SPI_Receive(spi_handle, data_out, static_cast<std::uint16_t>(length), timeout_ms);
    (void)set_chip_select_level(chip_select_pin, GPIO_PIN_SET);

    return (tx_status == HAL_OK) && (rx_status == HAL_OK);
  }

  bool imu_write_register_spi(void *platform_handle,
                              imu_api::imu_target target,
                              std::uint8_t register_address,
                              const std::uint8_t *data_in,
                              std::size_t length)
  {
    if (platform_handle == nullptr || data_in == nullptr || length == 0U)
    {
      return false;
    }

    imu_spi_bus_handle *bus = static_cast<imu_spi_bus_handle *>(platform_handle);
    if (bus->spi_handle == nullptr)
    {
      return false;
    }

    SPI_HandleTypeDef *spi_handle = static_cast<SPI_HandleTypeDef *>(bus->spi_handle);
    const std::uint32_t timeout_ms = (bus->transfer_timeout_ms == 0U) ? 10U : bus->transfer_timeout_ms;

    std::uint8_t command = static_cast<std::uint8_t>(register_address & 0x7FU);
    if (length > 1U)
    {
      command = static_cast<std::uint8_t>(command | 0x40U);
    }

    const gpio_pin_handle &chip_select_pin = select_chip_select_pin(*bus, target);
    if (!set_chip_select_level(chip_select_pin, GPIO_PIN_RESET))
    {
      return false;
    }

    const HAL_StatusTypeDef tx_command_status = HAL_SPI_Transmit(spi_handle, &command, 1U, timeout_ms);
    const HAL_StatusTypeDef tx_data_status = HAL_SPI_Transmit(spi_handle, const_cast<std::uint8_t *>(data_in), static_cast<std::uint16_t>(length), timeout_ms);
    (void)set_chip_select_level(chip_select_pin, GPIO_PIN_SET);

    return (tx_command_status == HAL_OK) && (tx_data_status == HAL_OK);
  }
}
