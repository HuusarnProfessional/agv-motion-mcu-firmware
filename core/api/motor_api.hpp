#pragma once

#include <cstdint>
#include <cstddef>

namespace motor_api
{
  // function pointers for board
  using pwm_start_fn       = void (*)(void *platform_handle, std::uint32_t channel);
  using pwm_set_compare_fn = void (*)(void *platform_handle, std::uint32_t channel, std::uint32_t ccr);
  using pwm_get_period_fn  = std::uint32_t (*)(void *platform_handle);

  // v table
  struct pwm_ops
  {
    pwm_start_fn       start;
    pwm_set_compare_fn set_compare;
    pwm_get_period_fn  get_period;
  };

  // one pwm output
  struct pwm_out
  {
    void *platform_handle;
    std::uint32_t channel;
    const pwm_ops *platform_operations;
  };

  // two pwm signals
  struct motor_pwm2
  {
    pwm_out pwm_a;
    pwm_out pwm_b;
  };

  // from board
  void init(const motor_pwm2 *motors, std::size_t count);

  // set motor command
  void set_u(std::uint8_t motor_id, std::int16_t u);

  using backend_init_fn = void (*)(const motor_pwm2 *motors, std::size_t count);
  using backend_set_u_fn = void (*)(std::uint8_t motor_id, std::int16_t u);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_set_u_fn set_u_fn;
  };
}
