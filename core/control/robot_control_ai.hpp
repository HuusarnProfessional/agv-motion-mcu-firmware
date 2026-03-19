#pragma once

#include <cstdint>

#include "core/control/robot_control.hpp"

namespace robot_control_ai
{
  void init(void);
  void tick(std::uint32_t now_ms);
  void apply_comm_drive_defaults(const robot_control::comm_drive_defaults &defaults);
}
