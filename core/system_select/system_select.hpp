#pragma once

#include <cstdint>

namespace encoder_api
{
  struct backend_operation;
}
namespace motor_api
{
  struct backend_operation;
}
namespace imu_api
{
  struct backend_operation;
}
namespace voltage_monitor_api
{
  struct backend_operation;
}
namespace led_api
{
  struct backend_operation;
}
namespace obstacle_api
{
  struct backend_operation;
}
namespace button_api
{
  struct backend_operation;
}
namespace comm_uart_api
{
  struct backend_operation;
}

// compile-time selection of implementations.
// board/platform is selected by build target (toolchain/startup/linker), while system_select
// only describes which implementation the api layer should route to.
namespace system_select
{
  enum class motor_impl : std::uint8_t
  {
    drv8871 = 0,
    hbridge_pcb = 1
  };
  inline constexpr motor_impl k_motor_impl = motor_impl::drv8871;

  enum class encoder_impl : std::uint8_t
  {
    as5600_pwm = 0
  };
  inline constexpr encoder_impl k_encoder_impl = encoder_impl::as5600_pwm;

  enum class imu_impl : std::uint8_t
  {
    lsm9ds1 = 0
  };
  inline constexpr imu_impl k_imu_impl = imu_impl::lsm9ds1;

  enum class voltage_monitor_impl : std::uint8_t
  {
    adc = 0
  };
  inline constexpr voltage_monitor_impl k_voltage_monitor_impl = voltage_monitor_impl::adc;

  enum class led_impl : std::uint8_t
  {
    gpio = 0
  };
  inline constexpr led_impl k_led_impl = led_impl::gpio;

  enum class obstacle_impl : std::uint8_t
  {
    hcsr04 = 0
  };
  inline constexpr obstacle_impl k_obstacle_impl = obstacle_impl::hcsr04;

  enum class comm_uart_impl : std::uint8_t
  {
    uart = 0
  };
  inline constexpr comm_uart_impl k_comm_uart_impl = comm_uart_impl::uart;

  enum class button_impl : std::uint8_t
  {
    gpio = 0
  };
  inline constexpr button_impl k_button_impl = button_impl::gpio;

  void select_voltage_monitor_backend(voltage_monitor_api::backend_operation &backend);
  void select_encoder_backend(encoder_api::backend_operation &backend);
  void select_motor_backend(motor_api::backend_operation &backend);
  void select_imu_backend(imu_api::backend_operation &backend);
  void select_obstacle_backend(obstacle_api::backend_operation &backend);
  void select_comm_uart_backend(comm_uart_api::backend_operation &backend);
  void select_led_backend(led_api::backend_operation &backend);
  void select_button_backend(button_api::backend_operation &backend);
}
