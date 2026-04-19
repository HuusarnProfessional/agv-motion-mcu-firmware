#pragma once

#include <cstdint>

namespace safe_guard
{
  struct snapshot
  {
    bool fault_latched = false;
    bool unlock_requested = false;
    bool fault_led_on = false;
    std::uint32_t latched_time_ms = 0u;
  };

  void init(void);
  void trip(void);
  void request_unlock(void);
  void tick(std::uint32_t now_ms);
  bool is_latched(void);
  void read_snapshot(snapshot &out);
}
