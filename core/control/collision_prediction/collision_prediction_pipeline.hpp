#pragma once

#include <cstdint>

namespace collision_prediction
{
  struct runtime_config
  {
    bool obstacle_safety_enabled = true;
  };

  struct snapshot
  {
    bool collision_blocked = false;
  };

  void init(void);
  void tick(std::uint32_t now_ms);
  void read_snapshot(snapshot &out);
  void set_obstacle_safety_enabled(bool enabled);
  bool is_obstacle_safety_enabled(void);
}
