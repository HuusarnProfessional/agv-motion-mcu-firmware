#include "core/control/drive_control/drive_control_pipeline.hpp"

#include "core/control/drive_control/wheel_drive_controller.hpp"
#include "core/control/local_positioning/local_positioning_pipeline.hpp"

namespace
{
  struct pipeline_state
  {
    bool has_motion_command = false;
    middleware_incoming_payloads::motion_command_payload_data latest_motion_command = {};
    wheel_drive_controller::state wheel_drive_controller_state = {};
  };

  pipeline_state g_pipeline_state = {};
}

namespace drive_control
{
  void init(void)
  {
    g_pipeline_state = {};
    wheel_drive_controller::init(g_pipeline_state.wheel_drive_controller_state);
  }

  void set_motion_command(const middleware_incoming_payloads::motion_command_payload_data &motion_command)
  {
    g_pipeline_state.latest_motion_command = motion_command;
    g_pipeline_state.has_motion_command = true;
  }

  void tick(std::uint32_t now_ms)
  {
    local_positioning::snapshot local_position_snapshot = {};
    local_positioning_pipeline::read_snapshot(local_position_snapshot);

    if (!g_pipeline_state.has_motion_command)
    {
      wheel_drive_controller::stop();
      return;
    }

    wheel_drive_controller::tick(g_pipeline_state.wheel_drive_controller_state, now_ms, g_pipeline_state.latest_motion_command, local_position_snapshot);
  }

  void read_snapshot(snapshot &out)
  {
    out.has_motion_command = g_pipeline_state.has_motion_command;
    out.latest_motion_command = g_pipeline_state.latest_motion_command;
  }

  void set_rotation_drive_tuning_override(std::int32_t min_drive_u, std::int32_t startup_drive_u)
  {
    wheel_drive_controller::rotation_drive_tuning tuning = {};
    tuning.has_override = true;
    tuning.min_drive_u = min_drive_u;
    tuning.startup_drive_u = startup_drive_u;
    wheel_drive_controller::set_rotation_drive_tuning_override(tuning);
  }

  void clear_rotation_drive_tuning_override(void)
  {
    wheel_drive_controller::clear_rotation_drive_tuning_override();
  }
}
