#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/comm_uart_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern UART_HandleTypeDef huart1;
}

namespace
{
  static void *get_comm_uart_handle(void)
  {
    return static_cast<void *>(&huart1);
  }

  static constexpr std::uint16_t k_comm_uart_tx_pin_id = 4u; // USART1_TX on PC4
  static constexpr std::uint16_t k_comm_uart_rx_pin_id = 5u; // USART1_RX on PC5
  static constexpr std::uint32_t k_comm_uart_baud_rate = 115200u;

  static const comm_uart_api::uart_operations k_uart_operations = { platform_stm32_hal::comm_uart_configure, platform_stm32_hal::comm_uart_register_event_callback, platform_stm32_hal::comm_uart_enable_receive, platform_stm32_hal::comm_uart_transmit_byte };
  static const comm_uart_api::comm_uart_input k_comm_uart_inputs[] = { { get_comm_uart_handle(), k_comm_uart_tx_pin_id, k_comm_uart_rx_pin_id, k_comm_uart_baud_rate, &k_uart_operations } };
  static constexpr std::size_t k_comm_uart_count = sizeof(k_comm_uart_inputs) / sizeof(k_comm_uart_inputs[0]);
}

void board_stm32g474re_agv_init_comm_uart(void)
{
  comm_uart_api::init(k_comm_uart_inputs, k_comm_uart_count);
}
