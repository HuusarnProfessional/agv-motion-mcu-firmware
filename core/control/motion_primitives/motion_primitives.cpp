#include "core/control/motion_primitives/motion_primitives.hpp"

#include "core/control/drive_control/drive_control_pipeline.hpp"
#include "core/control/motion_primitives/motion_primitive_drive_forward.hpp"
#include "core/control/motion_primitives/motion_primitive_pause.hpp"
#include "core/control/motion_primitives/motion_primitive_rotate_delta.hpp"
#include "core/control/motion_primitives/motion_primitives_common.hpp"
#include "core/control/motion_primitives/motion_primitives_tuning.hpp"

namespace
{
  motion_primitives_common::state g_state = {};
  void finish_current_primitive(bool success, bool timed_out, motion_primitives::error_code failure_code, std::uint32_t now_ms);

  motion_primitives_common::tick_result tick_active_primitive(const motion_primitives::input_snapshot &input, std::uint32_t now_ms)
  {
    if (g_state.active_request.primitive_id_value == motion_primitives::primitive_id::pause)
    {
      return motion_primitive_pause::tick(g_state, now_ms);
    }

    if (g_state.active_request.primitive_id_value == motion_primitives::primitive_id::drive_forward)
    {
      return motion_primitive_drive_forward::tick(g_state, input, now_ms);
    }

    if (g_state.active_request.primitive_id_value == motion_primitives::primitive_id::rotate_delta)
    {
      return motion_primitive_rotate_delta::tick(g_state, input, now_ms);
    }

    return motion_primitives_common::tick_result::complete_failure;
  }

  void tick_settling(const motion_primitives::input_snapshot &input, std::uint32_t now_ms)
  {
    motion_primitives_common::stop_motion(now_ms);

    if (motion_primitives_common::has_elapsed(now_ms, g_state.snapshot.stop_time_ms))
    {
      finish_current_primitive(true, false, motion_primitives::error_code::none, now_ms);
      return;
    }

    if (!motion_primitives_common::is_settled(input.encoder_state, input.imu_state))
    {
      g_state.snapshot.still_since_ms = 0u;
      return;
    }

    if (g_state.snapshot.still_since_ms == 0u)
    {
      g_state.snapshot.still_since_ms = now_ms;
      return;
    }

    if (!motion_primitives_common::has_elapsed(now_ms, g_state.snapshot.still_since_ms + motion_primitives_tuning::k_settling_still_duration_ms))
    {
      return;
    }

    finish_current_primitive(true, false, motion_primitives::error_code::none, now_ms);
  }

  void apply_rotation_drive_tuning_if_needed(const motion_primitives::request &primitive_request)
  {
    if (primitive_request.primitive_id_value != motion_primitives::primitive_id::rotate_delta)
    {
      return;
    }

    if (primitive_request.rotate_delta.has_rotation_drive_tuning)
    {
      drive_control::set_rotation_drive_tuning_override(primitive_request.rotate_delta.rotation_min_drive_u, primitive_request.rotate_delta.rotation_startup_drive_u);
      return;
    }

    drive_control::set_rotation_drive_tuning_override(motion_primitives_tuning::k_rotate_delta_default_rotation_min_drive_u, motion_primitives_tuning::k_rotate_delta_default_rotation_startup_drive_u);
  }

  void clear_rotation_drive_tuning_if_needed(void)
  {
    if (g_state.active_request.primitive_id_value != motion_primitives::primitive_id::rotate_delta)
    {
      return;
    }

    drive_control::clear_rotation_drive_tuning_override();
  }

  void finish_current_primitive(bool success, bool timed_out, motion_primitives::error_code failure_code, std::uint32_t now_ms)
  {
    clear_rotation_drive_tuning_if_needed();
    motion_primitives_common::finish(g_state, success, timed_out, failure_code, now_ms);
  }
}

namespace motion_primitives
{
  void init(void)
  {
    g_state = {};
  }

  bool start(const request &primitive_request, std::uint32_t now_ms, const local_positioning::snapshot &current_pose)
  {
    if (g_state.snapshot.running)
    {
      return false;
    }

    if (primitive_request.primitive_id_value == primitive_id::none)
    {
      return false;
    }

    if (primitive_request.primitive_id_value != primitive_id::pause)
    {
      if (!current_pose.has_pose)
      {
        return false;
      }
    }

    g_state = {};
    g_state.snapshot.command_id = primitive_request.command_id;
    g_state.snapshot.running = true;
    g_state.snapshot.active_primitive_id = primitive_request.primitive_id_value;
    g_state.snapshot.start_time_ms = now_ms;
    g_state.snapshot.status_time_ms = now_ms;
    g_state.active_request = primitive_request;

    if (current_pose.has_pose)
    {
      g_state.snapshot.start_pose = current_pose;
      g_state.snapshot.phase_start_pose = current_pose;
      g_state.has_latest_pose = true;
      g_state.latest_pose = current_pose;
      g_state.latest_pose_received_time_ms = now_ms;
    }

    motion_primitives_common::begin_active_phase(g_state, current_pose);
    apply_rotation_drive_tuning_if_needed(primitive_request);

    if (primitive_request.primitive_id_value == primitive_id::pause)
    {
      g_state.snapshot.stop_time_ms = now_ms + primitive_request.pause.duration_ms;
    }
    else if (primitive_request.primitive_id_value == primitive_id::drive_forward)
    {
      g_state.snapshot.stop_time_ms = now_ms + motion_primitives_tuning::k_default_drive_forward_timeout_ms;
    }
    else if (primitive_request.primitive_id_value == primitive_id::rotate_delta)
    {
      g_state.snapshot.stop_time_ms = now_ms + motion_primitives_tuning::k_default_rotate_delta_timeout_ms;
    }

    motion_primitives_common::stop_motion(now_ms);
    return true;
  }

  void tick(std::uint32_t now_ms, const input_snapshot &input)
  {
    if (!g_state.snapshot.running)
    {
      return;
    }

    if (g_state.active_phase == motion_primitives_common::phase::active)
    {
      const motion_primitives_common::tick_result result = tick_active_primitive(input, now_ms);

      if (result == motion_primitives_common::tick_result::keep_running)
      {
        return;
      }

      if (result == motion_primitives_common::tick_result::begin_settling)
      {
        motion_primitives_common::begin_settling(g_state, input.pose, now_ms);
        return;
      }

      if (result == motion_primitives_common::tick_result::complete_success)
      {
        finish_current_primitive(true, false, motion_primitives::error_code::none, now_ms);
        return;
      }

      if (result == motion_primitives_common::tick_result::complete_timeout)
      {
        finish_current_primitive(false, true, motion_primitives::error_code::timeout, now_ms);
        return;
      }

      finish_current_primitive(false, false, motion_primitives::error_code::failure, now_ms);
      return;
    }

    if (g_state.active_phase == motion_primitives_common::phase::settling)
    {
      tick_settling(input, now_ms);
      return;
    }

    finish_current_primitive(false, false, motion_primitives::error_code::failure, now_ms);
  }

  void stop(std::uint32_t now_ms)
  {
    if (!g_state.snapshot.running)
    {
      return;
    }

    finish_current_primitive(false, false, motion_primitives::error_code::stopped, now_ms);
  }

  void read_snapshot(snapshot &out)
  {
    out = g_state.snapshot;
  }
}
