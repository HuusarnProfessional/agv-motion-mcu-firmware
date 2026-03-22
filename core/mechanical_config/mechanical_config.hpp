#pragma once

#include <cstdint>

namespace mechanical_config
{
  struct drivetrain
  {
    std::int32_t wheel_diameter_mm = 63;
    std::int32_t wheel_separation_mm = 164;
  };
}
