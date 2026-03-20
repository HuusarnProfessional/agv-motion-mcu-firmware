#include "core/control/robot_control.hpp"

#include "core/control/controller_esp32_wroom32_demo.hpp"
#include "core/control/controller_robot_future.hpp"

namespace robot_control
{
  namespace
  {
    inline bool use_demo_path(void)
    {
      return (g_pick_controller == 1U);
    }
  }

  void init(void)
  {
    if (use_demo_path())
    {
      controller_esp32_wroom32_demo::init();
      return;
    }

    controller_robot_future::init();
  }

  void tick(std::uint32_t now_ms)
  {
    if (use_demo_path())
    {
      controller_esp32_wroom32_demo::tick(now_ms);
      return;
    }

    controller_robot_future::tick(now_ms);
  }

  void apply_comm_drive_defaults(const comm_drive_defaults &defaults)
  {
    (void)defaults;
  }
}
