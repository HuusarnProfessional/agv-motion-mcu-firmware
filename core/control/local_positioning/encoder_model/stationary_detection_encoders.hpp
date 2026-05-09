#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/delta_estimation_encoders.hpp"

namespace stationary_detection_encoders
{
  struct state
  {
    bool is_moving = false;
    std::uint8_t stationary_tick_count = 0u;
    std::int64_t pending_fl_um = 0;
    std::int64_t pending_fr_um = 0;
    std::int64_t pending_rl_um = 0;
    std::int64_t pending_rr_um = 0;
  };

  struct snapshot
  {
    bool has_classified_delta = false;
    bool is_stationary = true;
    bool has_motion = false;
    delta_estimation_encoders::delta_snapshot classified_delta = {};
  };

  void reset(state &classifier_state);
  bool classify_delta(state &classifier_state, const delta_estimation_encoders::delta_snapshot &input_delta, snapshot &out);
}
