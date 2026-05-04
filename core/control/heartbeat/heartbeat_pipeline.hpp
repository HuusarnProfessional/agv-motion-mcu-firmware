#pragma once

#include <cstdint>

namespace heartbeat
{
  struct snapshot
  {
    bool has_seen_heartbeat = false;
    bool heartbeat_timed_out = false;
    std::uint32_t last_heartbeat_time_ms = 0u;
  };

  void init(void);
  void notify_heartbeat(std::uint32_t received_time_ms);
  void tick(std::uint32_t now_ms);
  bool is_timed_out(void);
  void read_snapshot(snapshot &out);
}
