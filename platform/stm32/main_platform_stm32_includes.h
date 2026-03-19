#pragma once

// Inkluderar korrekt STM32 HAL-header beroende pa vald MCU-serie.
// Dessa makron (STM32F7xx, STM32F3xx, ...) satts normalt av CubeMX-projektet.

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
  // Fallback: vissa projekt inkluderar korrekt HAL via main.h
  #include "main.h"
#endif
