#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace
{
  constexpr std::size_t k_max_uart_runtime = 4U;

  struct uart_runtime
  {
    UART_HandleTypeDef *uart_handle = nullptr;
    comm_uart_api::platform_event_callback_fn event_callback = nullptr;
    void *event_context = nullptr;
    std::uint8_t rx_byte = 0U;
    std::uint8_t tx_byte = 0U;
    bool in_use = false;
  };

  std::array<uart_runtime, k_max_uart_runtime> g_uart_runtime = {};

  uart_runtime *find_runtime(UART_HandleTypeDef *uart_handle)
  {
    if (uart_handle == nullptr)
    {
      return nullptr;
    }

    for (std::size_t runtime_index = 0U; runtime_index < g_uart_runtime.size(); ++runtime_index)
    {
      uart_runtime &selected_runtime = g_uart_runtime[runtime_index];

      if (selected_runtime.in_use && selected_runtime.uart_handle == uart_handle)
      {
        return &selected_runtime;
      }
    }

    return nullptr;
  }

  uart_runtime *find_or_create_runtime(UART_HandleTypeDef *uart_handle)
  {
    uart_runtime *selected_runtime = find_runtime(uart_handle);

    if (selected_runtime != nullptr)
    {
      return selected_runtime;
    }

    for (std::size_t runtime_index = 0U; runtime_index < g_uart_runtime.size(); ++runtime_index)
    {
      uart_runtime &free_runtime = g_uart_runtime[runtime_index];

      if (free_runtime.in_use)
      {
        continue;
      }

      free_runtime = {};
      free_runtime.uart_handle = uart_handle;
      free_runtime.in_use = true;
      return &free_runtime;
    }

    return nullptr;
  }

  bool arm_receive(uart_runtime &runtime)
  {
    return HAL_UART_Receive_IT(runtime.uart_handle, &runtime.rx_byte, 1U) == HAL_OK;
  }
}

extern "C" void platform_stm32_hal_uart_irq_handler(void *platform_handle)
{
  if (platform_handle == nullptr)
  {
    return;
  }

  HAL_UART_IRQHandler(static_cast<UART_HandleTypeDef *>(platform_handle));
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_runtime *selected_runtime = find_runtime(huart);

  if (selected_runtime == nullptr)
  {
    return;
  }

  if (selected_runtime->event_callback != nullptr)
  {
    const comm_uart_api::uart_platform_event event = { comm_uart_api::uart_platform_event_type::received_byte, selected_runtime->rx_byte };

    selected_runtime->event_callback(selected_runtime->event_context, &event);
  }

  arm_receive(*selected_runtime);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_runtime *selected_runtime = find_runtime(huart);

  if (selected_runtime == nullptr || selected_runtime->event_callback == nullptr)
  {
    return;
  }

  const comm_uart_api::uart_platform_event event = { comm_uart_api::uart_platform_event_type::transmitted_byte, selected_runtime->tx_byte };

  selected_runtime->event_callback(selected_runtime->event_context, &event);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_runtime *selected_runtime = find_runtime(huart);

  if (selected_runtime == nullptr)
  {
    return;
  }

  arm_receive(*selected_runtime);
}

namespace platform_stm32_hal
{
  bool comm_uart_configure(void *platform_handle, std::uint16_t tx_pin_id, std::uint16_t rx_pin_id, std::uint32_t baud_rate)
  {
    (void)tx_pin_id;
    (void)rx_pin_id;
    (void)baud_rate;

    UART_HandleTypeDef *uart_handle = static_cast<UART_HandleTypeDef *>(platform_handle);
    return find_or_create_runtime(uart_handle) != nullptr;
  }

  bool comm_uart_register_event_callback(void *platform_handle, comm_uart_api::platform_event_callback_fn event_callback, void *event_context)
  {
    uart_runtime *selected_runtime = find_or_create_runtime(static_cast<UART_HandleTypeDef *>(platform_handle));

    if (selected_runtime == nullptr || event_callback == nullptr)
    {
      return false;
    }

    selected_runtime->event_callback = event_callback;
    selected_runtime->event_context = event_context;
    return true;
  }

  bool comm_uart_enable_receive(void *platform_handle)
  {
    uart_runtime *selected_runtime = find_or_create_runtime(static_cast<UART_HandleTypeDef *>(platform_handle));

    if (selected_runtime == nullptr)
    {
      return false;
    }

    return arm_receive(*selected_runtime);
  }

  bool comm_uart_transmit_byte(void *platform_handle, std::uint8_t byte)
  {
    uart_runtime *selected_runtime = find_or_create_runtime(static_cast<UART_HandleTypeDef *>(platform_handle));

    if (selected_runtime == nullptr)
    {
      return false;
    }

    selected_runtime->tx_byte = byte;
    return HAL_UART_Transmit_IT(selected_runtime->uart_handle, &selected_runtime->tx_byte, 1U) == HAL_OK;
  }
}
