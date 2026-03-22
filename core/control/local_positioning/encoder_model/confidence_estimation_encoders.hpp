#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/delta_estimation_encoders.hpp"

namespace confidence_estimation_encoders
{
  enum class confidence_case : std::uint8_t
  {
    all_ok = 0u,
    one_bad,
    two_bad_same_axle,
    two_bad_same_side,
    two_bad_mixed_axle,
    three_bad,
    four_bad
  };

  struct confidence_snapshot
  {
    confidence_case case_id = confidence_case::four_bad;
    bool meas_enable_fl = false;
    bool meas_enable_fr = false;
    bool meas_enable_rl = false;
    bool meas_enable_rr = false;
    bool can_estimate_translation = false;
    bool can_estimate_rotation = false;
    bool has_confidence = false;
  };

  void reset(confidence_snapshot &state);
  bool estimate_from_delta_snapshot(const delta_estimation_encoders::delta_snapshot &delta_snapshot, confidence_snapshot &out);
}
