#pragma once

#include <cstdint>

#include "core/api/motor_api.hpp"

namespace rc_test_motor0_cycle_ai
{
  struct state
  {
    std::uint32_t t0_ms = 0;
    std::uint8_t phase = 0;
  };

  inline void init(state &s)
  {
    s.t0_ms = 0;
    s.phase = 0;
    motor_api::set_u(0, 0);
  }

  inline void tick(state &s, std::uint32_t now_ms)
  {
    const std::uint32_t dt = now_ms - s.t0_ms;
    if (dt < 5000)
    {
      return;
    }

    s.t0_ms = now_ms;

    static constexpr std::int16_t k_u_test = 400;

    if (s.phase == 0)
    {
      motor_api::set_u(0, 0);
      s.phase = 1;
    }
    else if (s.phase == 1)
    {
      motor_api::set_u(0, k_u_test);
      s.phase = 2;
    }
    else if (s.phase == 2)
    {
      motor_api::set_u(0, 0);
      s.phase = 3;
    }
    else
    {
      motor_api::set_u(0, -k_u_test);
      s.phase = 0;
    }
  }
}
