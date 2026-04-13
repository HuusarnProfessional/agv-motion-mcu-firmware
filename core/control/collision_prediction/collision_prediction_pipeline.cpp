#include "core/control/collision_prediction/collision_prediction_pipeline.hpp"

namespace
{
  collision_prediction::snapshot g_snapshot = {};
}

namespace collision_prediction
{
  void init(void)
  {
    g_snapshot = {};
  }

  void tick(std::uint32_t now_ms)
  {
    (void)now_ms;
  }

  void read_snapshot(snapshot &out)
  {
    out = g_snapshot;
  }
}
