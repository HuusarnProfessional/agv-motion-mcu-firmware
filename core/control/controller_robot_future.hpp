#pragma once

#include <cstdint>

namespace controller_robot_future
{
  void init(void);
  void tick(std::uint32_t now_ms);
}
