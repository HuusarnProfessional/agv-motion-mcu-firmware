#pragma once

#include <cstdint>

#include "core/api/motor_api.hpp"

namespace rc_test_all_motors_step_ai
{
  struct state
  {
    std::uint32_t t0_ms = 0;
    std::uint8_t phase = 0;
  };

  inline void set_all(std::int16_t u)
  {
    for (std::uint8_t id = 0; id < 4; ++id)
    {
      motor_api::set_u(id, u);
    }
  }

  inline void stop_all(void)
  {
    set_all(0);
  }

  inline void init(state &s)
  {
    s.t0_ms = 0;
    s.phase = 0;
    set_all(350);
  }

  inline void tick(state &s, std::uint32_t now_ms)
  {
    static constexpr std::int16_t k_u_test = 350;
    const std::uint32_t dt = now_ms - s.t0_ms;

    if (s.phase == 0)
    {
      set_all(k_u_test);
      if (dt >= 4000)
      {
        s.phase = 1;
        s.t0_ms = now_ms;
      }
    }
    else if (s.phase == 1)
    {
      set_all(-k_u_test);
      if (dt >= 4000)
      {
        s.phase = 2;
        s.t0_ms = now_ms;
      }
    }
    else if (s.phase == 2)
    {
      stop_all();
      if (dt >= 4000)
      {
        s.phase = 0;
        s.t0_ms = now_ms;
      }
    }
  }
}
