#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"

namespace encoder_motion
{
  void reset(state &local_positioning_state)
  {
    encoder_input_storage::reset(local_positioning_state.encoder_input_snapshot);
    delta_estimation_encoders::reset(local_positioning_state.encoder_delta_snapshot);
    stationary_detection_encoders::reset(local_positioning_state.encoder_stationary_detection_state);
    confidence_estimation_encoders::reset(local_positioning_state.encoder_confidence_snapshot);
    motion_model_encoders::reset(local_positioning_state.encoder_motion_snapshot);
  }

  void tick(state &local_positioning_state, std::uint32_t tick_id)
  {
    // step 1: sample and store encoder current/previous
    const bool has_fresh_encoder_sample = encoder_input_storage::sample_from_encoder_api(local_positioning_state.encoder_input_snapshot, tick_id);

    if (!has_fresh_encoder_sample)
    {
      delta_estimation_encoders::reset(local_positioning_state.encoder_delta_snapshot);
      confidence_estimation_encoders::reset(local_positioning_state.encoder_confidence_snapshot);
      motion_model_encoders::reset(local_positioning_state.encoder_motion_snapshot);
      return;
    }

    // step 2: estimate per-wheel delta from previous/current input samples
    delta_estimation_encoders::estimate_from_encoder_snapshot(local_positioning_state.encoder_input_snapshot, local_positioning_state.encoder_delta_snapshot);

    // step 3: classify small/noisy deltas before they reach confidence and motion model
    stationary_detection_encoders::snapshot classified_encoder_snapshot = {};
    const bool has_classified_encoder_motion =
      stationary_detection_encoders::classify_delta(
        local_positioning_state.encoder_stationary_detection_state,
        local_positioning_state.encoder_delta_snapshot,
        classified_encoder_snapshot);

    if (!has_classified_encoder_motion)
    {
      confidence_estimation_encoders::reset(local_positioning_state.encoder_confidence_snapshot);
      motion_model_encoders::reset(local_positioning_state.encoder_motion_snapshot);
      return;
    }

    // step 4: estimate confidence/gating from classified wheel delta statuses
    confidence_estimation_encoders::estimate_from_delta_snapshot(classified_encoder_snapshot.classified_delta, local_positioning_state.encoder_confidence_snapshot);

    // step 5: estimate motion model from classified encoder deltas + confidence
    motion_model_encoders::estimate_from_encoder(classified_encoder_snapshot.classified_delta, local_positioning_state.encoder_confidence_snapshot, local_positioning_state.encoder_motion_snapshot);
  }
}
