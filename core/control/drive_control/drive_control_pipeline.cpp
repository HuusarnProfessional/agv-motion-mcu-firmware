#include "core/control/drive_control/drive_control_pipeline.hpp"

#include "core/control/drive_control/AI_drive_control.hpp"

namespace
{
  struct pipeline_state
  {
    bool has_motion_command = false;
    middleware_incoming_payloads::motion_command_payload_data latest_motion_command = {};
    AI_drive_control::state AI_drive_control_state = {};
  };

  pipeline_state g_pipeline_state = {};
}

namespace drive_control
{
  void init(void)
  {
    g_pipeline_state = {};
    AI_drive_control::init(g_pipeline_state.AI_drive_control_state);
  }

  void set_motion_command(const middleware_incoming_payloads::motion_command_payload_data &motion_command)
  {
    g_pipeline_state.latest_motion_command = motion_command;
    g_pipeline_state.has_motion_command = true;
  }

  void tick(std::uint32_t now_ms)
  {
    if (!g_pipeline_state.has_motion_command)
    {
      AI_drive_control::stop();
      return;
    }

    AI_drive_control::tick(g_pipeline_state.AI_drive_control_state, now_ms, g_pipeline_state.latest_motion_command);
  }

  void read_snapshot(snapshot &out)
  {
    out.has_motion_command = g_pipeline_state.has_motion_command;
    out.latest_motion_command = g_pipeline_state.latest_motion_command;
  }
}
