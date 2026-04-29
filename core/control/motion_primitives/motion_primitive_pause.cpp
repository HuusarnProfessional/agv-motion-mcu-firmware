#include "core/control/motion_primitives/motion_primitive_pause.hpp"

namespace motion_primitive_pause
{
  motion_primitives_common::tick_result tick(motion_primitives_common::state &primitive_state, std::uint32_t now_ms)
  {
    motion_primitives_common::stop_motion(now_ms);

    if (!motion_primitives_common::has_elapsed(now_ms, primitive_state.snapshot.start_time_ms + primitive_state.active_request.pause.duration_ms))
    {
      return motion_primitives_common::tick_result::keep_running;
    }

    return motion_primitives_common::tick_result::complete_success;
  }
}
