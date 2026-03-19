#include "core/system_select/system_select.hpp"

#include "core/api/motor_api.hpp"
#include "core/impl/motor_drv8871_impl.hpp"

namespace
{
  void init_motor_drv8871(const motor_api::motor_pwm2 *motors, std::size_t count)
  {
    motor_drv8871_impl::init(motors, count);
  }

  void set_u_motor_drv8871(std::uint8_t motor_id, std::int16_t u)
  {
    motor_drv8871_impl::set_u(motor_id, u);
  }
}

