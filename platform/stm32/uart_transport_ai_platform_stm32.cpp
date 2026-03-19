#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  static void *g_default_uart_transport_ctx = nullptr;

  static constexpr std::size_t k_uart_rx_ring_cap = 512U;
  static volatile std::uint8_t g_uart_rx_ring[k_uart_rx_ring_cap] = {};
  static volatile std::uint16_t g_uart_rx_head = 0U;
  static volatile std::uint16_t g_uart_rx_tail = 0U;

  struct uart_diag_state
  {
    std::uint32_t bytes_rx = 0U;
    std::uint32_t ring_overflow = 0U;
    std::uint32_t err_overrun = 0U;
    std::uint32_t err_noise = 0U;
    std::uint32_t err_framing = 0U;
    std::uint32_t err_parity = 0U;
    std::uint32_t sentinels = 0U;
  };

  static volatile uart_diag_state g_uart_diag = {};

  static void diag_inc(volatile std::uint32_t &counter)
  {
    // Avoid deprecated volatile ++ while keeping explicit read/modify/write.
    counter = static_cast<std::uint32_t>(counter + 1U);
  }

  static std::uint16_t ring_next(std::uint16_t index)
  {
    const std::uint16_t next = static_cast<std::uint16_t>(index + 1U);
    return (next >= static_cast<std::uint16_t>(k_uart_rx_ring_cap)) ? 0U : next;
  }

  static bool ring_push_isr(std::uint8_t b)
  {
    const std::uint16_t head = g_uart_rx_head;
    const std::uint16_t next = ring_next(head);
    if (next == g_uart_rx_tail)
    {
      // Ring full: drop oldest byte so ingress keeps moving forward.
      g_uart_rx_tail = ring_next(g_uart_rx_tail);
      diag_inc(g_uart_diag.ring_overflow);
    }

    g_uart_rx_ring[head] = b;
    g_uart_rx_head = next;
    diag_inc(g_uart_diag.bytes_rx);
    return true;
  }

  static bool ring_pop(std::uint8_t &out)
  {
    const std::uint16_t tail = g_uart_rx_tail;
    if (tail == g_uart_rx_head)
    {
      return false;
    }

    out = g_uart_rx_ring[tail];
    g_uart_rx_tail = ring_next(tail);
    return true;
  }

#if !defined(USART_ISR_RXNE) || !defined(USART_ICR_ORECF)
  static std::uint32_t uart_reg_status(const USART_TypeDef *uart)
  {
    return uart->SR;
  }

  static bool uart_reg_has_error(std::uint32_t status)
  {
    return (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U;
  }

  static bool uart_reg_has_rx(std::uint32_t status)
  {
    return (status & USART_SR_RXNE) != 0U;
  }

  static bool uart_reg_has_tx_empty(std::uint32_t status)
  {
    return (status & USART_SR_TXE) != 0U;
  }

  static bool uart_reg_tx_complete(std::uint32_t status)
  {
    return (status & USART_SR_TC) != 0U;
  }

  static bool uart_reg_err_overrun(std::uint32_t status)
  {
    return (status & USART_SR_ORE) != 0U;
  }

  static bool uart_reg_err_noise(std::uint32_t status)
  {
    return (status & USART_SR_NE) != 0U;
  }

  static bool uart_reg_err_framing(std::uint32_t status)
  {
    return (status & USART_SR_FE) != 0U;
  }

  static bool uart_reg_err_parity(std::uint32_t status)
  {
    return (status & USART_SR_PE) != 0U;
  }

  static std::uint8_t uart_reg_read_rx(USART_TypeDef *uart)
  {
    return static_cast<std::uint8_t>(uart->DR & 0xFFU);
  }

  static void uart_reg_write_tx(USART_TypeDef *uart, std::uint8_t value)
  {
    uart->DR = value;
  }

  static void uart_reg_discard_error_byte(USART_TypeDef *uart, std::uint32_t status)
  {
    (void)uart;
    (void)status;
  }

  static void uart_reg_clear_error(USART_TypeDef *uart)
  {
    // L1 clears ORE/NE/FE/PE by read SR followed by read DR.
    volatile std::uint32_t sr = uart->SR;
    (void)sr;
    volatile std::uint32_t dr = uart->DR;
    (void)dr;
  }
#else
  static std::uint32_t uart_reg_status(const USART_TypeDef *uart)
  {
    return uart->ISR;
  }

  static bool uart_reg_has_error(std::uint32_t status)
  {
    return (status & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) != 0U;
  }

  static bool uart_reg_has_rx(std::uint32_t status)
  {
    return (status & USART_ISR_RXNE) != 0U;
  }

  static bool uart_reg_has_tx_empty(std::uint32_t status)
  {
    return (status & USART_ISR_TXE) != 0U;
  }

  static bool uart_reg_tx_complete(std::uint32_t status)
  {
    return (status & USART_ISR_TC) != 0U;
  }

  static bool uart_reg_err_overrun(std::uint32_t status)
  {
    return (status & USART_ISR_ORE) != 0U;
  }

  static bool uart_reg_err_noise(std::uint32_t status)
  {
    return (status & USART_ISR_NE) != 0U;
  }

  static bool uart_reg_err_framing(std::uint32_t status)
  {
    return (status & USART_ISR_FE) != 0U;
  }

  static bool uart_reg_err_parity(std::uint32_t status)
  {
    return (status & USART_ISR_PE) != 0U;
  }

  static std::uint8_t uart_reg_read_rx(USART_TypeDef *uart)
  {
    return static_cast<std::uint8_t>(uart->RDR & 0xFFU);
  }

  static void uart_reg_write_tx(USART_TypeDef *uart, std::uint8_t value)
  {
    uart->TDR = value;
  }

  static void uart_reg_discard_error_byte(USART_TypeDef *uart, std::uint32_t status)
  {
    if (uart_reg_has_rx(status))
    {
      (void)uart_reg_read_rx(uart);
    }
  }

  static void uart_reg_clear_error(USART_TypeDef *uart)
  {
    uart->ICR = USART_ICR_ORECF | USART_ICR_NCF | USART_ICR_FECF | USART_ICR_PECF;
  }
#endif

  static void uart_capture_error_to_ring(USART_TypeDef *uart, std::uint32_t isr)
  {
    if (uart_reg_err_overrun(isr))
    {
      diag_inc(g_uart_diag.err_overrun);
    }
    if (uart_reg_err_noise(isr))
    {
      diag_inc(g_uart_diag.err_noise);
    }
    if (uart_reg_err_framing(isr))
    {
      diag_inc(g_uart_diag.err_framing);
    }
    if (uart_reg_err_parity(isr))
    {
      diag_inc(g_uart_diag.err_parity);
    }

    // Keep parser resync behavior on transport errors.
    if (ring_push_isr(0x01U))
    {
      diag_inc(g_uart_diag.sentinels);
    }

    uart_reg_discard_error_byte(uart, isr);
    uart_reg_clear_error(uart);
  }

  static void uart_drain_hw_to_ring_isr(USART_TypeDef *uart)
  {
    if (uart == nullptr)
    {
      return;
    }

    for (;;)
    {
      const std::uint32_t isr = uart_reg_status(uart);
      if (uart_reg_has_error(isr))
      {
        uart_capture_error_to_ring(uart, isr);
        continue;
      }

      if (!uart_reg_has_rx(isr))
      {
        break;
      }

      const std::uint8_t b = uart_reg_read_rx(uart);
      (void)ring_push_isr(b);
    }
  }
}

extern "C" void platform_stm32_hal_uart2_irq_handler(void)
{
#if defined(USART2)
  uart_drain_hw_to_ring_isr(USART2);
#endif
}

extern "C" void platform_stm32_hal_uart3_irq_handler(void)
{
#if defined(USART3)
  uart_drain_hw_to_ring_isr(USART3);
#endif
}

namespace platform_stm32_hal
{
  static comm_api_ai::status uart_tx_bytes(void *ctx, const std::uint8_t *data, std::size_t len)
  {
    if (!ctx || (!data && len > 0U))
    {
      return comm_api_ai::status::invalid_arg;
    }

    USART_TypeDef *uart = static_cast<USART_TypeDef *>(ctx);

    for (std::size_t i = 0U; i < len; ++i)
    {
      std::uint32_t spins = 200000U;
      while (!uart_reg_has_tx_empty(uart_reg_status(uart)))
      {
        uart_drain_hw_to_ring_isr(uart);
        if (spins == 0U)
        {
          return comm_api_ai::status::io_error;
        }
        --spins;
      }

      uart_reg_write_tx(uart, data[i]);
    }

    std::uint32_t tc_spins = 200000U;
    while (!uart_reg_tx_complete(uart_reg_status(uart)))
    {
      uart_drain_hw_to_ring_isr(uart);
      if (tc_spins == 0U)
      {
        return comm_api_ai::status::io_error;
      }
      --tc_spins;
    }

    return comm_api_ai::status::ok;
  }

  static std::size_t uart_rx_bytes(void *ctx, std::uint8_t *dst, std::size_t cap)
  {
    if (!ctx || !dst || cap == 0U)
    {
      return 0U;
    }

    USART_TypeDef *uart = static_cast<USART_TypeDef *>(ctx);
    uart_drain_hw_to_ring_isr(uart);

    std::size_t n = 0U;
    while (n < cap)
    {
      std::uint8_t b = 0U;
      if (!ring_pop(b))
      {
        break;
      }

      dst[n] = b;
      ++n;
    }

    return n;
  }

  static const comm_api_ai::transport_ops k_uart_ops =
  {
    uart_tx_bytes,
    uart_rx_bytes
  };

  const comm_api_ai::transport_ops *get_uart_transport_ops(void)
  {
    return &k_uart_ops;
  }

  void set_default_uart_transport_ctx(void *ctx)
  {
    g_default_uart_transport_ctx = ctx;
  }

  void *get_default_uart_transport_ctx(void)
  {
    return g_default_uart_transport_ctx;
  }

  void get_uart_diag_snapshot(uart_diag_snapshot &out)
  {
    __disable_irq();
    out.bytes_rx = g_uart_diag.bytes_rx;
    out.ring_overflow = g_uart_diag.ring_overflow;
    out.err_overrun = g_uart_diag.err_overrun;
    out.err_noise = g_uart_diag.err_noise;
    out.err_framing = g_uart_diag.err_framing;
    out.err_parity = g_uart_diag.err_parity;
    out.sentinels = g_uart_diag.sentinels;
    __enable_irq();
  }
}
