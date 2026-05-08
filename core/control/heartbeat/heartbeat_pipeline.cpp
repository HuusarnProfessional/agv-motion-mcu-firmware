#include "core/control/heartbeat/heartbeat_pipeline.hpp"

namespace
{
  constexpr std::uint32_t k_heartbeat_timeout_ms = 2000u;

  struct pipeline_state
  {
    heartbeat::snapshot latest_snapshot = {};
  };

  pipeline_state g_pipeline_state = {};
}

namespace heartbeat
{
  void init(void)
  {
    g_pipeline_state = {};
  }

  void notify_heartbeat(std::uint32_t received_time_ms)
  {
    g_pipeline_state.latest_snapshot.has_seen_heartbeat = true;
    g_pipeline_state.latest_snapshot.heartbeat_timed_out = false;
    g_pipeline_state.latest_snapshot.last_heartbeat_time_ms = received_time_ms;
  }

  void tick(std::uint32_t now_ms)
  {
    if (!g_pipeline_state.latest_snapshot.has_seen_heartbeat)
    {
      return;
    }

    if (now_ms < g_pipeline_state.latest_snapshot.last_heartbeat_time_ms)
    {
      g_pipeline_state.latest_snapshot.heartbeat_timed_out = true;
      return;
    }

    if (now_ms - g_pipeline_state.latest_snapshot.last_heartbeat_time_ms > k_heartbeat_timeout_ms)
    {
      g_pipeline_state.latest_snapshot.heartbeat_timed_out = true;
      return;
    }

    g_pipeline_state.latest_snapshot.heartbeat_timed_out = false;
  }

  bool is_timed_out(void)
  {
    return g_pipeline_state.latest_snapshot.heartbeat_timed_out;
  }

  void read_snapshot(snapshot &out)
  {
    out = g_pipeline_state.latest_snapshot;
  }
}