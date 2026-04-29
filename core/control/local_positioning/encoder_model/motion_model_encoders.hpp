#pragma once

#include "core/control/local_positioning/encoder_model/confidence_estimation_encoders.hpp"
#include "core/control/local_positioning/encoder_model/delta_estimation_encoders.hpp"

namespace motion_model_encoders
{
  struct motion_model_snapshot
  {
    bool has_motion_model = false;
    bool rotate_in_spot_flag = false;
    std::int64_t translation = 0;
    std::int64_t rotation = 0;
    std::int64_t confidence_slip = 0;
    std::int64_t confidence_translation = 0;
    std::int64_t confidence_rotation = 0;
    bool has_left_delta = false;
    bool has_right_delta = false;
    std::int64_t left_delta = 0;
    std::int64_t right_delta = 0;
  };

  void reset(motion_model_snapshot &state);
  bool estimate_from_encoder(const delta_estimation_encoders::delta_snapshot &delta_snapshot, const confidence_estimation_encoders::confidence_snapshot &confidence_snapshot, motion_model_snapshot &out);
}
