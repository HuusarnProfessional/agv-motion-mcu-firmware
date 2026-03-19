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
namespace obstacle_api
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
  // motor implementation selection.
  inline constexpr motor_impl k_motor_impl = motor_impl::drv8871;

  enum class encoder_impl : std::uint8_t
  {
    as5600_pwm = 0
  };
  // encoder implementation selection.
  inline constexpr encoder_impl k_encoder_impl = encoder_impl::as5600_pwm;

  enum class imu_impl : std::uint8_t
  {
    lsm9ds1 = 0
  };
  // imu implementation selection.
  inline constexpr imu_impl k_imu_impl = imu_impl::lsm9ds1;

  enum class voltage_monitor_impl : std::uint8_t
  {
    adc = 0
  };
  // voltage monitor implementation selection.
  inline constexpr voltage_monitor_impl k_voltage_monitor_impl = voltage_monitor_impl::adc;

  enum class obstacle_impl : std::uint8_t
  {
    hcsr04 = 0
  };
  // obstacle sensor implementation selection.
  inline constexpr obstacle_impl k_obstacle_impl = obstacle_impl::hcsr04;

  // binds selected voltage monitor backend callbacks for voltage_monitor_api.
  void select_voltage_monitor_backend(voltage_monitor_api::backend_operation &backend);

  // binds selected encoder backend callbacks for encoder_api.
  void select_encoder_backend(encoder_api::backend_operation &backend);

  // binds selected motor backend callbacks for motor_api.
  void select_motor_backend(motor_api::backend_operation &backend);

  // binds selected imu backend callbacks for imu_api.
  void select_imu_backend(imu_api::backend_operation &backend);

  // binds selected obstacle backend callbacks for obstacle_api.
  void select_obstacle_backend(obstacle_api::backend_operation &backend);

}
