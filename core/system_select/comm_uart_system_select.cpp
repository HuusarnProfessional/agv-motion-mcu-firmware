#include "core/system_select/system_select.hpp"

#include "core/api/comm_uart_api.hpp"
#include "core/impl/comm_uart_impl.hpp"

namespace
{
  void init_comm_uart(const comm_uart_api::comm_uart_input *inputs, std::size_t count)
  {
    comm_uart_impl::init(inputs, count);
  }

  comm_uart_api::comm_uart_status tx_comm_uart(std::uint8_t comm_uart_id, const std::uint8_t *data, std::size_t length)
  {
    return comm_uart_impl::write_bytes(comm_uart_id, data, length);
  }

  std::size_t rx_comm_uart(std::uint8_t comm_uart_id, std::uint8_t *data_out, std::size_t capacity)
  {
    return comm_uart_impl::read_bytes(comm_uart_id, data_out, capacity);
  }
}

