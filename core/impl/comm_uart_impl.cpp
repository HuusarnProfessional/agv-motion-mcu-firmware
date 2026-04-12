#include "core/impl/comm_uart_impl.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace
{
  constexpr std::size_t k_max_comm_uarts = 1U; //change if more is used, but takes more ram.
  constexpr std::size_t k_rx_buffer_capacity = 500U; //just a guess, decrease if it somehow can be measured when running
  constexpr std::size_t k_tx_buffer_capacity = 500U;

  struct uart_runtime
  {
    const comm_uart_api::comm_uart_input *input = nullptr;
    bool in_use = false;
    std::array<std::uint8_t, k_rx_buffer_capacity> rx_buffer = {};
    std::array<std::uint8_t, k_tx_buffer_capacity> tx_buffer = {};
    std::atomic<std::size_t> rx_head = 0U;
    std::atomic<std::size_t> rx_tail = 0U;
    std::atomic<std::size_t> tx_head = 0U;
    std::atomic<std::size_t> tx_tail = 0U;
    std::atomic<bool> tx_in_progress = false;
    std::atomic<bool> rx_overflow = false;
  };

  std::array<uart_runtime, k_max_comm_uarts> g_uart_runtime = {};
  bool g_is_initialized = false;
  std::size_t g_uart_count = 0U;

  std::size_t advance_index(std::size_t index, std::size_t capacity)
  {
    ++index;

    if (index >= capacity)
    {
      return 0U;
    }

    return index;
  }

  std::size_t used_slots(std::size_t head, std::size_t tail, std::size_t capacity)
  {
    if (head >= tail)
    {
      return head - tail;
    }

    return capacity - (tail - head);
  }

  std::size_t free_slots(std::size_t head, std::size_t tail, std::size_t capacity)
  {
    return (capacity - 1U) - used_slots(head, tail, capacity);
  }

  void reset_runtime(uart_runtime &runtime)
  {
    runtime.input = nullptr;
    runtime.in_use = false;
    runtime.rx_head.store(0U, std::memory_order_relaxed);
    runtime.rx_tail.store(0U, std::memory_order_relaxed);
    runtime.tx_head.store(0U, std::memory_order_relaxed);
    runtime.tx_tail.store(0U, std::memory_order_relaxed);
    runtime.tx_in_progress.store(false, std::memory_order_relaxed);
    runtime.rx_overflow.store(false, std::memory_order_relaxed);
  }

  uart_runtime *get_runtime(std::uint8_t comm_uart_id)
  {
    if (!g_is_initialized || comm_uart_id >= g_uart_count)
    {
      return nullptr;
    }

    uart_runtime &selected_runtime = g_uart_runtime[comm_uart_id];

    if (!selected_runtime.in_use)
    {
      return nullptr;
    }

    return &selected_runtime;
  }

  bool try_read_next_queued_tx_byte(const uart_runtime &runtime, std::uint8_t &byte_out)
  {
    const std::size_t tx_tail = runtime.tx_tail.load(std::memory_order_acquire);
    const std::size_t tx_head = runtime.tx_head.load(std::memory_order_acquire);

    if (tx_tail == tx_head)
    {
      return false;
    }

    byte_out = runtime.tx_buffer[tx_tail];
    return true;
  }

  void mark_current_tx_byte_as_sent(uart_runtime &runtime)
  {
    const std::size_t tx_tail = runtime.tx_tail.load(std::memory_order_relaxed);
    runtime.tx_tail.store(advance_index(tx_tail, runtime.tx_buffer.size()), std::memory_order_release);
  }

  bool try_start_next_tx_transfer(uart_runtime &runtime)
  {
    bool expected = false;

    if (!runtime.tx_in_progress.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    {
      return true;
    }

    std::uint8_t byte_to_send = 0U;

    if (!try_read_next_queued_tx_byte(runtime, byte_to_send))
    {
      runtime.tx_in_progress.store(false, std::memory_order_release);
      return true;
    }

    if (runtime.input == nullptr || runtime.input->platform_operations == nullptr || runtime.input->platform_operations->transmit_byte == nullptr)
    {
      runtime.tx_in_progress.store(false, std::memory_order_release);
      return false;
    }

    if (!runtime.input->platform_operations->transmit_byte(runtime.input->platform_handle, byte_to_send))
    {
      runtime.tx_in_progress.store(false, std::memory_order_release);
      return false;
    }

    return true;
  }

  bool push_rx_byte(uart_runtime &runtime, std::uint8_t byte)
  {
    const std::size_t rx_head = runtime.rx_head.load(std::memory_order_relaxed);
    const std::size_t rx_tail = runtime.rx_tail.load(std::memory_order_acquire);
    const std::size_t next_head = advance_index(rx_head, runtime.rx_buffer.size());

    if (next_head == rx_tail)
    {
      runtime.rx_overflow.store(true, std::memory_order_release);
      return false;
    }

    runtime.rx_buffer[rx_head] = byte;
    runtime.rx_head.store(next_head, std::memory_order_release);
    return true;
  }

  void on_platform_event(void *event_context, const comm_uart_api::uart_platform_event *event)
  {
    if (event_context == nullptr || event == nullptr)
    {
      return;
    }

    uart_runtime &runtime = *static_cast<uart_runtime *>(event_context);

    switch (event->type)
    {
    case comm_uart_api::uart_platform_event_type::received_byte:
      push_rx_byte(runtime, event->byte);
      break;

    case comm_uart_api::uart_platform_event_type::transmitted_byte:
      mark_current_tx_byte_as_sent(runtime);

      {
        std::uint8_t next_byte = 0U;

        if (try_read_next_queued_tx_byte(runtime, next_byte))
        {
          if (runtime.input == nullptr || runtime.input->platform_operations == nullptr || runtime.input->platform_operations->transmit_byte == nullptr)
          {
            runtime.tx_in_progress.store(false, std::memory_order_release);
            break;
          }

          if (!runtime.input->platform_operations->transmit_byte(runtime.input->platform_handle, next_byte))
          {
            runtime.tx_in_progress.store(false, std::memory_order_release);
          }
        }
        else
        {
          runtime.tx_in_progress.store(false, std::memory_order_release);
          try_start_next_tx_transfer(runtime);
        }
      }
      break;
    }
  }
}

namespace comm_uart_impl
{
  void init(const comm_uart_api::comm_uart_input *inputs, std::size_t count)
  {
    g_is_initialized = false;
    g_uart_count = 0U;

    for (std::size_t runtime_index = 0U; runtime_index < g_uart_runtime.size(); ++runtime_index)
    {
      reset_runtime(g_uart_runtime[runtime_index]);
    }

    if (inputs == nullptr || count == 0U || count > g_uart_runtime.size())
    {
      return;
    }

    for (std::size_t input_index = 0U; input_index < count; ++input_index)
    {
      const comm_uart_api::comm_uart_input &selected_input = inputs[input_index];

      if (selected_input.platform_operations == nullptr || selected_input.platform_operations->configure == nullptr || selected_input.platform_operations->register_event_callback == nullptr || selected_input.platform_operations->enable_receive == nullptr || selected_input.platform_operations->transmit_byte == nullptr)
      {
        return;
      }

      uart_runtime &selected_runtime = g_uart_runtime[input_index];
      reset_runtime(selected_runtime);
      selected_runtime.input = &selected_input;
      selected_runtime.in_use = true;

      if (!selected_input.platform_operations->configure(selected_input.platform_handle, selected_input.tx_pin_id, selected_input.rx_pin_id, selected_input.baud_rate))
      {
        return;
      }

      if (!selected_input.platform_operations->register_event_callback(selected_input.platform_handle, on_platform_event, &selected_runtime))
      {
        return;
      }

      if (!selected_input.platform_operations->enable_receive(selected_input.platform_handle))
      {
        return;
      }
    }

    g_uart_count = count;
    g_is_initialized = true;
  }

  comm_uart_api::comm_uart_status write_bytes(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length)
  {
    uart_runtime *selected_runtime = get_runtime(comm_uart_id);

    if (selected_runtime == nullptr)
    {
      if (g_is_initialized)
      {
        return comm_uart_api::comm_uart_status::invalid_id;
      }

      return comm_uart_api::comm_uart_status::not_initialized;
    }

    if (data == nullptr && length > 0U)
    {
      return comm_uart_api::comm_uart_status::invalid_arg;
    }

    const std::size_t tx_head = selected_runtime->tx_head.load(std::memory_order_relaxed);
    const std::size_t tx_tail = selected_runtime->tx_tail.load(std::memory_order_acquire);

    if (length > free_slots(tx_head, tx_tail, selected_runtime->tx_buffer.size()))
    {
      return comm_uart_api::comm_uart_status::buffer_full;
    }

    std::size_t next_head = tx_head;

    for (std::size_t byte_index = 0U; byte_index < length; ++byte_index)
    {
      selected_runtime->tx_buffer[next_head] = data[byte_index];
      next_head = advance_index(next_head, selected_runtime->tx_buffer.size());
    }

    selected_runtime->tx_head.store(next_head, std::memory_order_release);

    if (!try_start_next_tx_transfer(*selected_runtime))
    {
      return comm_uart_api::comm_uart_status::io_error;
    }

    return comm_uart_api::comm_uart_status::ok;
  }

  std::size_t read_bytes(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity)
  {
    uart_runtime *selected_runtime = get_runtime(comm_uart_id);

    if (selected_runtime == nullptr || (data_out == nullptr && capacity > 0U))
    {
      return 0U;
    }

    std::size_t rx_tail = selected_runtime->rx_tail.load(std::memory_order_relaxed);
    const std::size_t rx_head = selected_runtime->rx_head.load(std::memory_order_acquire);
    std::size_t bytes_read = 0U;

    while (bytes_read < capacity && rx_tail != rx_head)
    {
      data_out[bytes_read++] = selected_runtime->rx_buffer[rx_tail];
      rx_tail = advance_index(rx_tail, selected_runtime->rx_buffer.size());
    }

    selected_runtime->rx_tail.store(rx_tail, std::memory_order_release);
    return bytes_read;
  }
}
