#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"

namespace local_positioning
{
  void reset(state &local_positioning_state)
  {
    encoder_input_storage::reset(local_positioning_state.encoder_input_snapshot);
    delta_estimation_encoders::reset(local_positioning_state.encoder_delta_snapshot);
    confidence_estimation_encoders::reset(local_positioning_state.encoder_confidence_snapshot);
    motion_model_encoders::reset(local_positioning_state.encoder_motion_snapshot);
  }

  void tick(state &local_positioning_state, std::uint32_t tick_id)
  {
    // step 1: sample and store encoder current/previous
    encoder_input_storage::sample_from_encoder_api(local_positioning_state.encoder_input_snapshot, tick_id);

    // step 2: estimate per-wheel delta from previous/current input samples
    delta_estimation_encoders::estimate_from_encoder_snapshot(local_positioning_state.encoder_input_snapshot, local_positioning_state.encoder_delta_snapshot);

    // step 3: estimate confidence/gating from wheel delta statuses
    confidence_estimation_encoders::estimate_from_delta_snapshot(local_positioning_state.encoder_delta_snapshot, local_positioning_state.encoder_confidence_snapshot);

    // step 4: estimate motion model from encoder deltas + confidence
    motion_model_encoders::estimate_from_encoder(local_positioning_state.encoder_delta_snapshot, local_positioning_state.encoder_confidence_snapshot, local_positioning_state.encoder_motion_snapshot);
  }
}
