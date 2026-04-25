#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/motor_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
}

namespace
{
  static TIM_HandleTypeDef *k_tim1 = &htim1;
  static TIM_HandleTypeDef *k_tim3 = &htim3;

  // motor order expected:
  // motor0 = front_left
  // motor1 = front_right
  // motor2 = rear_left
  // motor3 = rear_right

  // Each motor entry uses the verified wheel mapping and swaps a and b in {{a},{b}} so positive drive matches the forward encoder direction.

  // pwm pin map on stm32g474retx:
  // tim1_ch1 = pc0, tim1_ch2 = pc1, tim1_ch3 = pc2, tim1_ch4 = pc3
  // tim3_ch1 = pb4, tim3_ch2 = pb5, tim3_ch3 = pc8, tim3_ch4 = pc9
  static const motor_api::motor_pwm2 k_motors[] =
  {
    {
      { static_cast<void *>(k_tim3), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim3), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim1), TIM_CHANNEL_3, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim1), TIM_CHANNEL_4, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim3), TIM_CHANNEL_4, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim3), TIM_CHANNEL_3, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim1), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim1), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() }
    }
  };

  static constexpr std::size_t k_motor_count = sizeof(k_motors) / sizeof(k_motors[0]);
}

void board_stm32g474re_agv_init_motors(void)
{
  motor_api::init(k_motors, k_motor_count);
}
