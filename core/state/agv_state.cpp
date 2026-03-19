#include "core/state/agv_state.hpp"

#include "core/control/robot_control.hpp"

namespace agv_state
{
  // Minimal forsta version: delegerar direkt till regler- och styrmodulen.
  // Nar vi bygger vidare kan vi lagga in tillstand (init/idle/run/fel) och kommandon har.

  void init(void)
  {
    robot_control::init();
  }

  void tick(std::uint32_t now_ms)
  {
    robot_control::tick(now_ms);
  }

  void apply_drive_defaults(const drive_defaults &defaults)
  {
    robot_control::comm_drive_defaults cfg{};
    cfg.wheel_diameter_mm = defaults.wheel_diameter_mm;
    cfg.wheel_separation_mm = defaults.wheel_separation_mm;
    robot_control::apply_comm_drive_defaults(cfg);
  }
}
