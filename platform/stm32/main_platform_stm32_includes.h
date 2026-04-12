#pragma once

// Include the HAL header that matches the selected STM32 family.

#if defined(STM32F7xx)
  #include "stm32f7xx_hal.h"
#elif defined(STM32F3xx)
  #include "stm32f3xx_hal.h"
#elif defined(STM32L1xx)
  #include "stm32l1xx_hal.h"
#elif defined(STM32G4xx)
  #include "stm32g4xx_hal.h"
#elif defined(STM32F4xx)
  #include "stm32f4xx_hal.h"
#else
  // Fallback for projects that include the HAL via main.h.
  #include "main.h"
#endif
