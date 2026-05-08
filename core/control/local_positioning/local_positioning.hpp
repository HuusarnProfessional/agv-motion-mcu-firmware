#pragma once

#include <array>
#include <cstdint>

#include "core/control/local_positioning/sensorfusion/sensorfusion_pipeline.hpp"

namespace local_positioning
{
  struct pose_state
  {
    std::int64_t x_um = 0;
    std::int64_t y_um = 0;
    std::int32_t heading_urad = 0;
    std::uint32_t uncertainty_position_um2 = 0;
    std::uint32_t uncertainty_heading_urad2 = 0;
    std::uint32_t update_id = 0;
    std::uint32_t time_ms = 0;
    std::uint8_t pose_id = 0;
    std::uint8_t branch_id = 0;
  };

  struct history_entry
  {
    bool is_valid = false;
    bool has_fused_translation = false;
    bool has_fused_rotation = false;
    std::uint8_t pose_id = 0;
    std::uint8_t branch_id = 0;
    std::int32_t delta_translation_um = 0;
    std::int32_t delta_rotation_urad = 0;
    std::uint16_t confidence_translation = 0;
    std::uint16_t confidence_rotation = 0;
  };

  struct replay_job
  {
    bool active = false;
    bool ready_to_switch = false;
    std::uint8_t target_pose_id = 0;
    std::uint8_t next_replay_pose_id = 0;
    std::uint8_t source_branch_id = 0;
    std::uint8_t replay_branch_id = 0;
    pose_state replay_pose = {};
  };

  struct snapshot
  {
    bool has_pose = false;
    std::int64_t x_um = 0;
    std::int64_t y_um = 0;
    std::int32_t heading_urad = 0;
    std::uint16_t confidence_position = 0;
    std::uint16_t confidence_heading = 0;
    std::uint32_t update_id = 0;
    std::uint32_t time_ms = 0;
    std::uint8_t pose_id = 0;
    std::uint8_t branch_id = 0;
  };

  struct state
  {
    pose_state live_pose = {};
    std::array<history_entry, 256u> history = {};
    replay_job replay = {};
    snapshot output_snapshot = {};
  };

  struct external_correction_request
  {
    bool has_request = false;
    std::uint8_t pose_id = 0;
    std::uint8_t branch_id = 0;
  };

  void reset(state &local_positioning_state);
  void tick(state &local_positioning_state, const local_positioning_sensorfusion::state &sensorfusion_state, std::uint32_t now_ms);
  bool request_external_correction(state &local_positioning_state, const external_correction_request &request);
}
