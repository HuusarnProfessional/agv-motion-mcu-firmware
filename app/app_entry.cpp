#include "app/app_entry.hpp"

#include "core/state/agv_state.hpp"

extern "C" void app_init(void)
{
  agv_state::init();
}

extern "C" void app_step(uint32_t now_ms)
{
  agv_state::tick(now_ms);
}

extern "C" void app_apply_drive_defaults(const app_drive_defaults *defaults)
{
  if (defaults == nullptr)
  {
    return;
  }

  agv_state::drive_defaults cfg{};
  cfg.wheel_diameter_mm = defaults->wheel_diameter_mm;
  cfg.wheel_separation_mm = defaults->wheel_separation_mm;
  agv_state::apply_drive_defaults(cfg);
}
