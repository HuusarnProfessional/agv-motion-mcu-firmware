#include "core/control/safe_guard/safe_guard_pipeline.hpp"

#include "core/api/led_api.hpp"
#include "core/api/motor_api.hpp"
#include "core/control/safe_guard/safe_guard_tuning.hpp"

namespace
{
  struct pipeline_state
  {
    bool trip_requested = false;
    bool unlock_requested = false;
    safe_guard::snapshot latest_snapshot = {};
  };

  pipeline_state g_pipeline_state = {};

  void stop_all_motors(void)
  {
    for (std::uint8_t motor_id = 0u; motor_id < safe_guard_tuning::k_motor_count; ++motor_id)
    {
      motor_api::set_u(motor_id, 0);
    }
  }

  void set_fault_led(bool is_on)
  {
    led_api::set(safe_guard_tuning::k_fault_led_id, is_on);
  }
}

namespace safe_guard
{
  void init(void)
  {
    g_pipeline_state = {};
    set_fault_led(false);
  }

  void trip(void)
  {
    g_pipeline_state.trip_requested = true;
  }

  void request_unlock(void)
  {
    g_pipeline_state.unlock_requested = true;
  }

  void tick(std::uint32_t now_ms)
  {
    const bool had_unlock_request = g_pipeline_state.unlock_requested;
    g_pipeline_state.unlock_requested = false;

    if (had_unlock_request && !g_pipeline_state.trip_requested)
    {
      g_pipeline_state.latest_snapshot = {};
    }

    if (g_pipeline_state.trip_requested)
    {
      g_pipeline_state.latest_snapshot.fault_latched = true;
      g_pipeline_state.latest_snapshot.unlock_requested = false;

      if (g_pipeline_state.latest_snapshot.latched_time_ms == 0u)
      {
        g_pipeline_state.latest_snapshot.latched_time_ms = now_ms;
      }

      g_pipeline_state.trip_requested = false;
    }

    if (g_pipeline_state.latest_snapshot.fault_latched)
    {
      stop_all_motors();
    }

    g_pipeline_state.latest_snapshot.unlock_requested = had_unlock_request;
    g_pipeline_state.latest_snapshot.fault_led_on = g_pipeline_state.latest_snapshot.fault_latched;
    set_fault_led(g_pipeline_state.latest_snapshot.fault_latched);
  }

  bool is_latched(void)
  {
    return g_pipeline_state.latest_snapshot.fault_latched;
  }

  void read_snapshot(snapshot &out)
  {
    out = g_pipeline_state.latest_snapshot;
  }
}
