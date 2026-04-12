#pragma once

#include <cstdint>

#include "core/control/local_positioning/local_positioning.hpp"

namespace controller_robot_future
{
  void init(void);
  void tick(std::uint32_t now_ms);
  bool read_local_positioning_snapshot(local_positioning::snapshot &out);
}
