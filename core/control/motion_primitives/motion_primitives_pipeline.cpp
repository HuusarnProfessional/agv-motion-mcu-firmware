#include "core/control/motion_primitives/motion_primitives_pipeline.hpp"

#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/control/motion_primitives/motion_primitives_tuning.hpp"

namespace
{
  struct pipeline_state
  {
    std::uint32_t next_command_id = 1u;
    bool has_pending_start_request = false;
    motion_primitives::request pending_start_request = {};
    std::uint32_t pending_start_request_time_ms = 0u;
  };

  pipeline_state g_pipeline_state = {};

  void build_input_snapshot(motion_primitives::input_snapshot &out)
  {
    local_positioning_pipeline::read_snapshot(out.pose);
    local_positioning_pipeline::read_encoder_motion_state(out.encoder_state);
    local_positioning_pipeline::read_imu_state(out.imu_state);
  }

  void clear_pending_start_request(void)
  {
    g_pipeline_state.has_pending_start_request = false;
    g_pipeline_state.pending_start_request = {};
    g_pipeline_state.pending_start_request_time_ms = 0u;
  }

  bool has_elapsed(std::uint32_t now_ms, std::uint32_t target_ms)
  {
    return static_cast<std::int32_t>(now_ms - target_ms) >= 0;
  }
}

namespace motion_primitives_pipeline
{
  void init(void)
  {
    g_pipeline_state = {};
    motion_primitives::init();
  }

  bool request_start(const motion_primitives::request &primitive_request, std::uint32_t now_ms)
  {
    motion_primitives::snapshot primitive_snapshot = {};
    motion_primitives::read_snapshot(primitive_snapshot);

    if (primitive_snapshot.running)
    {
      return false;
    }

    if (g_pipeline_state.has_pending_start_request)
    {
      return false;
    }

    if (primitive_request.primitive_id_value == motion_primitives::primitive_id::none)
    {
      return false;
    }

    g_pipeline_state.has_pending_start_request = true;
    g_pipeline_state.pending_start_request = primitive_request;
    g_pipeline_state.pending_start_request.command_id = g_pipeline_state.next_command_id;
    g_pipeline_state.next_command_id += 1u;
    g_pipeline_state.pending_start_request_time_ms = now_ms;
    return true;
  }

  void tick(std::uint32_t now_ms)
  {
    motion_primitives::input_snapshot input = {};
    build_input_snapshot(input);

    if (g_pipeline_state.has_pending_start_request)
    {
      const bool start_accepted = motion_primitives::start(g_pipeline_state.pending_start_request, now_ms, input.pose);

      if (start_accepted)
      {
        clear_pending_start_request();
      }
      else if (has_elapsed(now_ms, g_pipeline_state.pending_start_request_time_ms + motion_primitives_tuning::k_start_pose_wait_timeout_ms))
      {
        clear_pending_start_request();
      }
    }

    motion_primitives::tick(now_ms, input);
  }

  void stop(std::uint32_t now_ms)
  {
    clear_pending_start_request();
    motion_primitives::stop(now_ms);
  }

  void read_snapshot(motion_primitives::snapshot &out)
  {
    motion_primitives::read_snapshot(out);
  }
}
