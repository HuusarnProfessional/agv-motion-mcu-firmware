#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

extern "C"
{
  extern UART_HandleTypeDef huart3;
}

extern "C" void platform_stm32_hal_comm_uart_irq_handler(void *platform_handle)
{
  if (platform_handle == nullptr)
  {
    return;
  }

  HAL_UART_IRQHandler(static_cast<UART_HandleTypeDef *>(platform_handle));
}

extern "C" void platform_stm32_hal_uart3_irq_handler(void)
{
  platform_stm32_hal_comm_uart_irq_handler(&huart3);
}

namespace platform_stm32_hal
{
  bool comm_uart_configure(void *platform_handle, std::uint16_t tx_pin_id, std::uint16_t rx_pin_id, std::uint32_t baud_rate)
  {
    (void)tx_pin_id;
    (void)rx_pin_id;
    (void)baud_rate;

    UART_HandleTypeDef *uart_handle = static_cast<UART_HandleTypeDef *>(platform_handle);

    __HAL_UART_DISABLE_IT(uart_handle, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(uart_handle, UART_IT_ERR);

    return true;
  }

  bool comm_uart_tx_bytes(void *platform_handle, const std::uint8_t *data, std::size_t length)
  {
    UART_HandleTypeDef *uart_handle = static_cast<UART_HandleTypeDef *>(platform_handle);

    return HAL_UART_Transmit(uart_handle, const_cast<std::uint8_t *>(data), static_cast<std::uint16_t>(length), 10u) == HAL_OK;
  }

  std::size_t comm_uart_rx_bytes(void *platform_handle, std::uint8_t *data_out, std::size_t capacity)
  {
    UART_HandleTypeDef *uart_handle = static_cast<UART_HandleTypeDef *>(platform_handle);

    std::size_t count = 0u;
    while (count < capacity)
    {
      if (HAL_UART_Receive(uart_handle, &data_out[count], 1u, 0u) != HAL_OK)
      {
        break;
      }
      ++count;
    }

    return count;
  }
}

