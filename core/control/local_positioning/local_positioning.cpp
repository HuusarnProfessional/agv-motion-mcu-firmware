#include "core/control/local_positioning/local_positioning.hpp"

#include <cmath>


namespace
{
  constexpr std::uint8_t k_replay_steps_per_tick = 8u;

  void write_live_history_entry(local_positioning::state &local_positioning_state, std::int32_t delta_translation_um, std::int32_t delta_rotation_urad, std::uint16_t confidence_translation, std::uint16_t confidence_rotation)
  {
    local_positioning::history_entry &entry = local_positioning_state.history[local_positioning_state.live_pose.pose_id];

    entry = {};
    entry.is_valid = true;
    entry.pose_id = local_positioning_state.live_pose.pose_id;
    entry.branch_id = local_positioning_state.live_pose.branch_id;
    entry.delta_translation_um = delta_translation_um;
    entry.delta_rotation_urad = delta_rotation_urad;
    entry.confidence_translation = confidence_translation;
    entry.confidence_rotation = confidence_rotation;
  }

  std::uint32_t map_translation_confidence_to_position_uncertainty(std::uint16_t confidence_translation)
  {
    const std::uint32_t confidence_max = 1000u;
    const std::uint32_t uncertainty_min = 1u;
    const std::uint32_t uncertainty_max = 10000u;

    std::uint32_t confidence_clamped = confidence_translation;

    if (confidence_clamped > confidence_max)
    {
      confidence_clamped = confidence_max;
    }

    const std::uint32_t uncertainty_range = uncertainty_max - uncertainty_min;
    const std::uint32_t uncertainty = uncertainty_max - (confidence_clamped * uncertainty_range) / confidence_max;

    return uncertainty;
  }

  std::uint32_t map_rotation_confidence_to_heading_uncertainty(std::uint16_t confidence_rotation)
  {
    const std::uint32_t confidence_max = 1000u;
    const std::uint32_t uncertainty_min = 1u;
    const std::uint32_t uncertainty_max = 10000u;

    std::uint32_t confidence_clamped = confidence_rotation;

    if (confidence_clamped > confidence_max)
    {
      confidence_clamped = confidence_max;
    }

    const std::uint32_t uncertainty_range = uncertainty_max - uncertainty_min;
    const std::uint32_t uncertainty = uncertainty_max - (confidence_clamped * uncertainty_range) / confidence_max;

    return uncertainty;
  }

  std::uint16_t map_position_uncertainty_to_confidence(std::uint32_t uncertainty_position_um2)
  {
    const std::uint32_t confidence_max = 1000u;
    const std::uint32_t uncertainty_min = 1u;
    const std::uint32_t uncertainty_max = 1000000u;
    std::uint32_t uncertainty_clamped = uncertainty_position_um2;

    if (uncertainty_clamped < uncertainty_min)
    {
      uncertainty_clamped = uncertainty_min;
    }

    if (uncertainty_clamped > uncertainty_max)
    {
      uncertainty_clamped = uncertainty_max;
    }

    const std::uint32_t uncertainty_range = uncertainty_max - uncertainty_min;
    const std::uint32_t confidence = confidence_max - ((uncertainty_clamped - uncertainty_min) * confidence_max) / uncertainty_range;

    return static_cast<std::uint16_t>(confidence);
  }

  std::uint16_t map_heading_uncertainty_to_confidence(std::uint32_t uncertainty_heading_urad2)
  {
    const std::uint32_t confidence_max = 1000u;
    const std::uint32_t uncertainty_min = 1u;
    const std::uint32_t uncertainty_max = 1000000u;
    std::uint32_t uncertainty_clamped = uncertainty_heading_urad2;

    if (uncertainty_clamped < uncertainty_min)
    {
      uncertainty_clamped = uncertainty_min;
    }

    if (uncertainty_clamped > uncertainty_max)
    {
      uncertainty_clamped = uncertainty_max;
    }

    const std::uint32_t uncertainty_range = uncertainty_max - uncertainty_min;
    const std::uint32_t confidence = confidence_max - ((uncertainty_clamped - uncertainty_min) * confidence_max) / uncertainty_range;

    return static_cast<std::uint16_t>(confidence);
  }

  void apply_pose_delta(local_positioning::pose_state &pose, std::int32_t delta_translation_um, std::int32_t delta_rotation_urad, std::uint16_t confidence_translation, std::uint16_t confidence_rotation, std::uint32_t now_ms)
  {
    const double previous_heading_urad = static_cast<double>(pose.heading_urad);
    // use the average heading during this tick for x/y translation
    const double translation_heading_rad = (previous_heading_urad + static_cast<double>(delta_rotation_urad) / 2.0) / 1000000.0;
    const double delta_x_um = static_cast<double>(delta_translation_um) * std::cos(translation_heading_rad);
    const double delta_y_um = static_cast<double>(delta_translation_um) * std::sin(translation_heading_rad);

    pose.x_um += static_cast<std::int64_t>(delta_x_um);
    pose.y_um += static_cast<std::int64_t>(delta_y_um);
    pose.heading_urad += delta_rotation_urad;
    pose.update_id += 1u;
    pose.time_ms = now_ms;
    pose.pose_id += 1;
    pose.uncertainty_position_um2 += map_translation_confidence_to_position_uncertainty(confidence_translation);
    pose.uncertainty_heading_urad2 += map_rotation_confidence_to_heading_uncertainty(confidence_rotation);
  }

  void update_live_pose_from_sensorfusion(local_positioning::state &local_positioning_state, const local_positioning_sensorfusion::state &sensorfusion_state, std::uint32_t now_ms)
  {
    const sensorfusion_translation::translation_snapshot &translation_snapshot = sensorfusion_state.translation_snapshot;
    const sensorfusion_rotation::rotation_snapshot &rotation_snapshot = sensorfusion_state.rotation_snapshot;

    if (!translation_snapshot.has_fused_translation && !rotation_snapshot.has_fused_rotation)
    {
      return;
    }

    std::int32_t delta_translation_um = 0;
    std::int32_t delta_rotation_urad = 0;

    if (translation_snapshot.has_fused_translation)
    {
      delta_translation_um = static_cast<std::int32_t>(translation_snapshot.translation);
    }

    if (rotation_snapshot.has_fused_rotation)
    {
      delta_rotation_urad = static_cast<std::int32_t>(rotation_snapshot.rotation);
    }

    std::uint16_t confidence_translation = 0;
    std::uint16_t confidence_rotation = 0;

    if (translation_snapshot.has_fused_translation)
    {
      confidence_translation = static_cast<std::uint16_t>(translation_snapshot.confidence_translation);
    }

    if (rotation_snapshot.has_fused_rotation)
    {
      confidence_rotation = static_cast<std::uint16_t>(rotation_snapshot.confidence_rotation);
    }

    apply_pose_delta(local_positioning_state.live_pose, delta_translation_um, delta_rotation_urad, confidence_translation, confidence_rotation, now_ms);

    write_live_history_entry(local_positioning_state, delta_translation_um, delta_rotation_urad, confidence_translation, confidence_rotation);

    local_positioning_state.output_snapshot.has_pose = true;
    local_positioning_state.output_snapshot.x_um = local_positioning_state.live_pose.x_um;
    local_positioning_state.output_snapshot.y_um = local_positioning_state.live_pose.y_um;
    local_positioning_state.output_snapshot.heading_urad = local_positioning_state.live_pose.heading_urad;
    local_positioning_state.output_snapshot.confidence_position = map_position_uncertainty_to_confidence(local_positioning_state.live_pose.uncertainty_position_um2);
    local_positioning_state.output_snapshot.confidence_heading = map_heading_uncertainty_to_confidence(local_positioning_state.live_pose.uncertainty_heading_urad2);
    local_positioning_state.output_snapshot.update_id = local_positioning_state.live_pose.update_id;
    local_positioning_state.output_snapshot.time_ms = local_positioning_state.live_pose.time_ms;
    local_positioning_state.output_snapshot.pose_id = local_positioning_state.live_pose.pose_id;
    local_positioning_state.output_snapshot.branch_id = local_positioning_state.live_pose.branch_id;
  }

  local_positioning::history_entry *find_history_entry(local_positioning::state &local_positioning_state, std::uint8_t pose_id, std::uint8_t branch_id)
  {
    local_positioning::history_entry &entry = local_positioning_state.history[pose_id];

    if (!entry.is_valid)
    {
      return nullptr;
    }

    if (entry.pose_id != pose_id)
    {
      return nullptr;
    }

    if (entry.branch_id != branch_id)
    {
      return nullptr;
    }

    return &entry;
  }

  std::uint8_t get_other_branch_id(std::uint8_t branch_id)
  {
    if (branch_id == 0u)
    {
      return 1u;
    }

    return 0u;
  }

  void process_replay_steps(local_positioning::state &local_positioning_state)
  {
    if (!local_positioning_state.replay.active)
    {
      return;
    }

    for (std::uint8_t i = 0u; i < k_replay_steps_per_tick; ++i)
    {
      local_positioning::history_entry *entry = find_history_entry(local_positioning_state, local_positioning_state.replay.next_replay_pose_id, local_positioning_state.replay.source_branch_id);

      if (entry == nullptr)
      {
        return;
      }

      apply_pose_delta(local_positioning_state.replay.replay_pose, entry->delta_translation_um, entry->delta_rotation_urad, entry->confidence_translation, entry->confidence_rotation, local_positioning_state.live_pose.time_ms);

      local_positioning_state.replay.next_replay_pose_id += 1;

      if (local_positioning_state.replay.next_replay_pose_id == static_cast<std::uint8_t>(local_positioning_state.live_pose.pose_id + 1u))
      {
        local_positioning_state.replay.ready_to_switch = true;
        return;
      }
    }
  }

  void switch_to_replay_branch_if_ready(local_positioning::state &local_positioning_state)
  {
    if (!local_positioning_state.replay.active)
    {
      return;
    }

    if (!local_positioning_state.replay.ready_to_switch)
    {
      return;
    }

    local_positioning_state.live_pose = local_positioning_state.replay.replay_pose;
    local_positioning_state.live_pose.branch_id = local_positioning_state.replay.replay_branch_id;
    local_positioning_state.replay = {};
  }

}



namespace local_positioning
{
  void reset(state &local_positioning_state)
  {
    local_positioning_state = {};
  }

  void tick(state &local_positioning_state, const local_positioning_sensorfusion::state &sensorfusion_state, std::uint32_t now_ms)
  {
    local_positioning_state.output_snapshot = {};
    update_live_pose_from_sensorfusion(local_positioning_state, sensorfusion_state, now_ms);
    process_replay_steps(local_positioning_state);
    switch_to_replay_branch_if_ready(local_positioning_state);
  }
    bool request_external_correction(state &local_positioning_state, const external_correction_request &request)
  {
    if (!request.has_request)
    {
      return false;
    }

    history_entry *entry = find_history_entry(local_positioning_state, request.pose_id, request.branch_id);

    if (entry == nullptr)
    {
      return false;
    }

    local_positioning_state.replay = {};
    local_positioning_state.replay.active = true;
    local_positioning_state.replay.ready_to_switch = false;
    local_positioning_state.replay.target_pose_id = request.pose_id;
    local_positioning_state.replay.next_replay_pose_id = static_cast<std::uint8_t>(request.pose_id + 1u);
    local_positioning_state.replay.source_branch_id = request.branch_id;
    local_positioning_state.replay.replay_branch_id = get_other_branch_id(request.branch_id);
    local_positioning_state.replay.replay_pose.x_um = 0;
    local_positioning_state.replay.replay_pose.y_um = 0;
    local_positioning_state.replay.replay_pose.heading_urad = 0;
    local_positioning_state.replay.replay_pose.uncertainty_position_um2 = 0;
    local_positioning_state.replay.replay_pose.uncertainty_heading_urad2 = 0;
    local_positioning_state.replay.replay_pose.update_id = local_positioning_state.live_pose.update_id;
    local_positioning_state.replay.replay_pose.time_ms = local_positioning_state.live_pose.time_ms;
    local_positioning_state.replay.replay_pose.pose_id = request.pose_id;
    local_positioning_state.replay.replay_pose.branch_id = local_positioning_state.replay.replay_branch_id;

    return true;
  }

}
