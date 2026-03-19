#include "core/control/robot_control_ai.hpp"

#include "core/api/motor_api.hpp"
#include "core/control/RC_test_all_motors_step_ai.hpp"
#include "core/control/RC_test_comm_middleware_ai.hpp"
#include "core/control/RC_test_motor0_cycle_ai.hpp"
#include "main.h"

namespace robot_control_ai
{
  namespace
  {
    enum class active_test : std::uint8_t
    {
      motor0_cycle = 0,
      all_motors_step = 1,
      comm_middleware = 2
    };

    // Keep previous default behavior for legacy AI path.
    static constexpr active_test k_active_test = active_test::comm_middleware;
    static constexpr bool k_estop_button_enabled = true;
    static constexpr bool k_estop_latched_mode = true;
    static constexpr bool k_estop_active_high = true;
    static constexpr std::uint32_t k_estop_debounce_ms = 40U;

    static rc_test_motor0_cycle_ai::state g_motor0_cycle_state;
    static rc_test_all_motors_step_ai::state g_all_motors_step_state;
    static rc_test_comm_middleware_ai::state g_comm_state;
    static bool g_estop_latched = false;
    static bool g_estop_prev_pressed = false;
    static std::uint32_t g_estop_last_edge_ms = 0U;

    static bool estop_button_pressed_raw(void)
    {
#if defined(B1_GPIO_Port) && defined(B1_Pin)
      const GPIO_PinState pin = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
      return k_estop_active_high ? (pin == GPIO_PIN_SET) : (pin == GPIO_PIN_RESET);
#else
      return false;
#endif
    }

    static bool estop_button_pressed_edge(std::uint32_t now_ms)
    {
      const bool pressed_now = estop_button_pressed_raw();
      if (pressed_now != g_estop_prev_pressed)
      {
        if ((now_ms - g_estop_last_edge_ms) >= k_estop_debounce_ms)
        {
          g_estop_last_edge_ms = now_ms;
          g_estop_prev_pressed = pressed_now;
          if (pressed_now)
          {
            return true;
          }
        }
      }
      return false;
    }

    static bool estop_triggered(std::uint32_t now_ms)
    {
      if (!k_estop_button_enabled)
      {
        return false;
      }

      if (k_estop_latched_mode)
      {
        if (g_estop_latched)
        {
          return true;
        }

        if (estop_button_pressed_edge(now_ms))
        {
          g_estop_latched = true;
          return true;
        }
        return false;
      }

      return estop_button_pressed_raw();
    }

    static void stop_all_outputs(void)
    {
      for (std::uint8_t id = 0U; id < 4U; ++id)
      {
        motor_api::set_u(id, 0);
      }
    }

    static void apply_comm_stop(void)
    {
      g_comm_state.stop = true;
      g_comm_state.motion_active = false;
      g_comm_state.distance_target_active = false;
      g_comm_state.arc_active = false;
      g_comm_state.motion_u = 0;
      g_comm_state.manual_motor_active = false;
      g_comm_state.manual_motor_u = 0;
      stop_all_outputs();
    }
  }

  void init(void)
  {
    g_estop_latched = false;
    g_estop_prev_pressed = estop_button_pressed_raw();
    g_estop_last_edge_ms = 0U;

    if (k_active_test == active_test::motor0_cycle)
    {
      rc_test_motor0_cycle_ai::init(g_motor0_cycle_state);
      return;
    }

    if (k_active_test == active_test::all_motors_step)
    {
      rc_test_all_motors_step_ai::init(g_all_motors_step_state);
      return;
    }

    if (k_active_test == active_test::comm_middleware)
    {
      rc_test_comm_middleware_ai::init(g_comm_state);
      return;
    }

    motor_api::set_u(0, 0);
  }

  void tick(std::uint32_t now_ms)
  {
    if (k_active_test == active_test::comm_middleware)
    {
      if (k_estop_button_enabled && estop_button_pressed_edge(now_ms))
      {
        // In comm mode, map hardware e-stop button to the same stop flag used by set_stop().
        // This keeps one recovery path: set_stop(false) from host.
        apply_comm_stop();
      }

      rc_test_comm_middleware_ai::tick(g_comm_state, now_ms);
      return;
    }

    if (estop_triggered(now_ms))
    {
      stop_all_outputs();
      return;
    }

    if (k_active_test == active_test::motor0_cycle)
    {
      rc_test_motor0_cycle_ai::tick(g_motor0_cycle_state, now_ms);
      return;
    }

    if (k_active_test == active_test::all_motors_step)
    {
      rc_test_all_motors_step_ai::tick(g_all_motors_step_state, now_ms);
      return;
    }

    motor_api::set_u(0, 0);
  }

  void apply_comm_drive_defaults(const robot_control::comm_drive_defaults &defaults)
  {
    rc_test_comm_middleware_ai::drive_defaults cfg{};
    cfg.wheel_diameter_mm = defaults.wheel_diameter_mm;
    cfg.wheel_separation_mm = defaults.wheel_separation_mm;
    rc_test_comm_middleware_ai::apply_defaults(g_comm_state, cfg);
  }
}
