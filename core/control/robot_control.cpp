#include "core/control/robot_control.hpp"

#include "core/control/controller_esp32_wroom32_demo.hpp"
#include "core/control/robot_control_ai.hpp"

namespace robot_control
{
  namespace
  {
    inline bool use_wroom_demo_path(void)
    {
      return (g_pick_controller == 1U);
    }
  }

  void init(void)
  {
    if (use_wroom_demo_path())
    {
      controller_esp32_wroom32_demo::init();
      return;
    }

    robot_control_ai::init();
  }

  void tick(std::uint32_t now_ms)
  {
    if (use_wroom_demo_path())
    {
      controller_esp32_wroom32_demo::tick(now_ms);
      return;
    }

    robot_control_ai::tick(now_ms);
  }

  void apply_comm_drive_defaults(const comm_drive_defaults &defaults)
  {
    if (use_wroom_demo_path())
    {
      (void)defaults;
      return;
    }

    robot_control_ai::apply_comm_drive_defaults(defaults);
  }
}
