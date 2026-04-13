#pragma once

#include <cstdint>

namespace collision_prediction
{
  struct snapshot
  {
    bool collision_blocked = false;
  };

  void init(void);
  void tick(std::uint32_t now_ms);
  void read_snapshot(snapshot &out);
}
