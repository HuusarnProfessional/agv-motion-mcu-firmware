#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct app_drive_defaults
{
  int32_t wheel_diameter_mm;
  int32_t wheel_separation_mm;
} app_drive_defaults;

// Gemensam entrypoint som anropas av target-specifik main (STM32 main.c, Arduino setup/loop, osv)
void app_init(void);
void app_step(uint32_t now_ms);
void app_apply_drive_defaults(const app_drive_defaults *defaults);

#ifdef __cplusplus
}
#endif
