#include "core/control/collision_prediction/collision_prediction_pipeline.hpp"

#include "core/control/collision_prediction/collision_prediction_logic.hpp"

namespace
{
  struct pipeline_state
  {
    collision_prediction::snapshot output_snapshot = {};
    collision_prediction_logic::state logic_state = {};
  };

  pipeline_state g_pipeline_state = {};
}

namespace collision_prediction
{
  void init(void)
  {
    g_pipeline_state = {};
    collision_prediction_logic::reset(g_pipeline_state.logic_state);
  }

  void tick(std::uint32_t now_ms)
  {
    collision_prediction_logic::result logic_result = {};
    collision_prediction_logic::tick(g_pipeline_state.logic_state, now_ms, logic_result);
    g_pipeline_state.output_snapshot.collision_blocked = logic_result.collision_blocked;
  }

  void read_snapshot(snapshot &out)
  {
    out = g_pipeline_state.output_snapshot;
  }
}
