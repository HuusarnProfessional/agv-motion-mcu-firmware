#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  constexpr std::size_t k_max_spi_transfer_length = 32U;

  static const platform_stm32_hal::gpio_pin_handle &select_chip_select_pin(const platform_stm32_hal::imu_spi_bus_handle &bus, imu_api::imu_target target)
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
  bool imu_read_register_spi(void *platform_handle, imu_api::imu_target target, std::uint8_t register_address, std::uint8_t *data_out, std::size_t length)
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
    std::uint32_t timeout_ms = bus->transfer_timeout_ms;

    if (timeout_ms == 0U)
    {
      timeout_ms = 10U;
    }

    std::uint8_t command = static_cast<std::uint8_t>(register_address | 0x80U);
    if (target == imu_api::imu_target::magnetometer && length > 1U)
    {
      command = static_cast<std::uint8_t>(command | 0x40U);
    }

    if (length > k_max_spi_transfer_length)
    {
      return false;
    }

    std::uint8_t tx_buffer[k_max_spi_transfer_length + 1U] = {};
    std::uint8_t rx_buffer[k_max_spi_transfer_length + 1U] = {};
    tx_buffer[0] = command;

    const gpio_pin_handle &chip_select_pin = select_chip_select_pin(*bus, target);
    if (!set_chip_select_level(chip_select_pin, GPIO_PIN_RESET))
    {
      return false;
    }

    const HAL_StatusTypeDef transfer_status = HAL_SPI_TransmitReceive(spi_handle, tx_buffer, rx_buffer, static_cast<std::uint16_t>(length + 1U), timeout_ms);
    (void)set_chip_select_level(chip_select_pin, GPIO_PIN_SET);

    if (transfer_status != HAL_OK)
    {
      return false;
    }

    for (std::size_t index = 0U; index < length; ++index)
    {
      data_out[index] = rx_buffer[index + 1U];
    }

    return true;
  }

  bool imu_write_register_spi(void *platform_handle, imu_api::imu_target target, std::uint8_t register_address, const std::uint8_t *data_in, std::size_t length)
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
    std::uint32_t timeout_ms = bus->transfer_timeout_ms;

    if (timeout_ms == 0U)
    {
      timeout_ms = 10U;
    }

    std::uint8_t command = static_cast<std::uint8_t>(register_address & 0x7FU);
    if (target == imu_api::imu_target::magnetometer && length > 1U)
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
