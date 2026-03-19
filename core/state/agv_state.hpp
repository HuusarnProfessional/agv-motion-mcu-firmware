#pragma once

#include <cstdint>

// Tillstandshantering for AGV:n.
// Haller flode (init, idle, fel) separat fran regler- och styrlogik.
namespace agv_state
{
  struct drive_defaults
  {
    std::int32_t wheel_diameter_mm;
    std::int32_t wheel_separation_mm;
  };

  void init(void);
  void tick(std::uint32_t now_ms);
  void apply_drive_defaults(const drive_defaults &defaults);
}
