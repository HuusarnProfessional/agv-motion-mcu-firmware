#include "core/control/drive_control/drive_control_pipeline.hpp"

namespace
{
  struct pipeline_state
  {
    bool has_motion_command = false;
    middleware_incoming_payloads::motion_command_payload_data latest_motion_command = {};
  };

  pipeline_state g_pipeline_state = {};
}

namespace drive_control
{
  void init(void)
  {
    g_pipeline_state = {};
  }

  void set_motion_command(const middleware_incoming_payloads::motion_command_payload_data &motion_command)
  {
    g_pipeline_state.latest_motion_command = motion_command;
    g_pipeline_state.has_motion_command = true;
  }

  void tick(std::uint32_t now_ms)
  {
    (void)g_pipeline_state;
    (void)now_ms;
  }

  void read_snapshot(snapshot &out)
  {
    out.has_motion_command = g_pipeline_state.has_motion_command;
    out.latest_motion_command = g_pipeline_state.latest_motion_command;
  }
}
