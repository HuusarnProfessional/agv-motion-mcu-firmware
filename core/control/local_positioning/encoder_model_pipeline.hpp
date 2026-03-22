#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/confidence_estimation_encoders.hpp"
#include "core/control/local_positioning/encoder_model/delta_estimation_encoders.hpp"
#include "core/control/local_positioning/encoder_model/input_storage_encoders.hpp"
#include "core/control/local_positioning/encoder_model/motion_model_encoders.hpp"

namespace local_positioning
{
  struct state
  {
    encoder_input_storage::encoder_snapshot encoder_input_snapshot;
    delta_estimation_encoders::delta_snapshot encoder_delta_snapshot;
    confidence_estimation_encoders::confidence_snapshot encoder_confidence_snapshot;
    motion_model_encoders::motion_model_snapshot encoder_motion_snapshot;
  };

  void reset(state &local_positioning_state);
  void tick(state &local_positioning_state, std::uint32_t tick_id);
}
