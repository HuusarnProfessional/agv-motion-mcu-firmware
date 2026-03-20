#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t g_pick_controller;

#ifdef __cplusplus
}

namespace controller_esp32_wroom32_demo
{
  void init(void);
  void tick(uint32_t now_ms);
  void set_drive_forward_mm(int32_t mm);
}
#endif
