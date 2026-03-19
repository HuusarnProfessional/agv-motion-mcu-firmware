#pragma once

#include <cstdint>

namespace robot_control
{
  struct comm_drive_defaults
  {
    std::int32_t wheel_diameter_mm;
    std::int32_t wheel_separation_mm;
  };

  void init(void);
  void tick(std::uint32_t now_ms);
  void apply_comm_drive_defaults(const comm_drive_defaults &defaults);
}
