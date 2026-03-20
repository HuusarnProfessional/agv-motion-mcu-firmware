#include "core/system_select/motor_system_select.cpp"
#include "core/system_select/encoder_system_select.cpp"
#include "core/system_select/imu_system_select.cpp"
#include "core/system_select/voltage_monitor_system_select.cpp"
#include "core/system_select/obstacle_system_select.cpp"
#include "core/system_select/comm_uart_system_select.cpp"

namespace system_select
{
  void select_encoder_backend(encoder_api::backend_operation &backend)
  {
    backend.init_fn = nullptr;
    backend.read_sample_fn = nullptr;

    if constexpr (k_encoder_impl == encoder_impl::as5600_pwm)
    {
      backend.init_fn = init_encoder_as5600_pwm;
      backend.read_sample_fn = read_encoder_as5600_pwm;
    }
  }

  void select_motor_backend(motor_api::backend_operation &backend)
  {
    backend.init_fn = nullptr;
    backend.set_u_fn = nullptr;

    if constexpr (k_motor_impl == motor_impl::drv8871)
    {
      backend.init_fn = init_motor_drv8871;
      backend.set_u_fn = set_u_motor_drv8871;
    }
  }

  void select_imu_backend(imu_api::backend_operation &backend)
  {
    backend.init_fn = nullptr;
    backend.read_sample_fn = nullptr;

    if constexpr (k_imu_impl == imu_impl::lsm9ds1)
    {
      backend.init_fn = init_imu_lsm9ds1;
      backend.read_sample_fn = read_imu_lsm9ds1;
    }
  }

  void select_voltage_monitor_backend(voltage_monitor_api::backend_operation &backend)
  {
    backend.init_fn = nullptr;
    backend.read_sample_fn = nullptr;

    if constexpr (k_voltage_monitor_impl == voltage_monitor_impl::adc)
    {
      backend.init_fn = init_voltage_monitor_adc;
      backend.read_sample_fn = read_voltage_monitor_adc;
    }
  }

  void select_obstacle_backend(obstacle_api::backend_operation &backend)
  {
    backend.init_fn = nullptr;
    backend.read_sample_fn = nullptr;

    if constexpr (k_obstacle_impl == obstacle_impl::hcsr04)
    {
      backend.init_fn = init_obstacle_hcsr04;
      backend.read_sample_fn = read_obstacle_hcsr04;
    }
  }

  void select_comm_uart_backend(comm_uart_api::backend_operation &backend)
  {
    backend.init_fn = nullptr;
    backend.write_bytes_fn = nullptr;
    backend.read_bytes_fn = nullptr;

    if constexpr (k_comm_uart_impl == comm_uart_impl::uart)
    {
      backend.init_fn = init_comm_uart;
      backend.write_bytes_fn = tx_comm_uart;
      backend.read_bytes_fn = rx_comm_uart;
    }
  }
}
