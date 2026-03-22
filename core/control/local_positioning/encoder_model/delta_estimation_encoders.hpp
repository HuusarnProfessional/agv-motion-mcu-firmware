#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/control/local_positioning/encoder_model/input_storage_encoders.hpp"
#include "core/mechanical_config/mechanical_config.hpp"

namespace delta_estimation_encoders
{
  enum class delta_status : std::uint8_t
  {
    ok = 0u,
    stale,
    bad
  };

  struct wheel_delta_sample
  {
    std::int64_t delta_raw_12bit = 0;
    std::uint32_t delta_time_ms = 0u;
    delta_status status = delta_status::stale;
  };
  using wheel_delta_array = std::array<wheel_delta_sample, encoder_input_storage::k_wheel_count>;

  struct wheel_delta_motion_sample
  {
    std::int64_t wheel_delta_distance_um = 0;
  };
  using wheel_delta_motion_array = std::array<wheel_delta_motion_sample, encoder_input_storage::k_wheel_count>;

  struct delta_snapshot
  {
    std::uint32_t previous_tick_id = 0u;
    std::uint32_t current_tick_id = 0u;
    wheel_delta_array wheel_deltas = {};
    wheel_delta_motion_array wheel_delta_motion = {};
    bool has_delta = false;
  };

  void reset(delta_snapshot &state);
  void set_drivetrain_config(const mechanical_config::drivetrain &drivetrain_config);
  bool estimate_from_encoder_snapshot(const encoder_input_storage::encoder_snapshot &encoder_snapshot, delta_snapshot &out);
}
