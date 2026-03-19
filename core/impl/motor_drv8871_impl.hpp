#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/motor_api.hpp"

namespace motor_drv8871_impl
{
  void init(const motor_api::motor_pwm2 *motors, std::size_t count);
  void set_u(std::uint8_t motor_id, std::int16_t u);
}
