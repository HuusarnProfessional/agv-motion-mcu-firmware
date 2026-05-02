#pragma once

#include <cstdint>

#include "core/control/motion_primitives/motion_primitives.hpp"

namespace motion_primitives_pipeline
{
  void init(void);
  bool request_start(const motion_primitives::request &primitive_request, std::uint32_t now_ms);
  void tick(std::uint32_t now_ms);
  void stop(std::uint32_t now_ms);
  void read_snapshot(motion_primitives::snapshot &out);
}
