#include "board/stm32l152discovery/main_board_stm32l152discovery.hpp"

#include "core/api/motor_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
}

namespace
{
  static TIM_HandleTypeDef *k_tim2 = &htim2;
  static TIM_HandleTypeDef *k_tim4 = &htim4;

  // motor mapping:
  // motor0 = front_left  (physical motor2, direction inverted by a/b swap)
  // motor1 = front_right (physical motor3)
  // motor2 = rear_left   (physical motor1, direction inverted by a/b swap)
  // motor3 = rear_right  (physical motor0)
  //
  // pwm pin map on stm32l152rctx:
  // tim4_ch1 = pb6,  tim4_ch2 = pb7,  tim4_ch3 = pb8,  tim4_ch4 = pb9
  // tim2_ch1 = pa15, tim2_ch2 = pb3,  tim2_ch3 = pb10, tim2_ch4 = pb11
  static const motor_api::motor_pwm2 k_motors[] =
  {
    {
      { static_cast<void *>(k_tim4), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim4), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim4), TIM_CHANNEL_3, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim4), TIM_CHANNEL_4, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim2), TIM_CHANNEL_4, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim2), TIM_CHANNEL_3, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim2), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim2), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() }
    }
  };

  static constexpr std::size_t k_motor_count = sizeof(k_motors) / sizeof(k_motors[0]);
}

void board_stm32l152discovery_init_motors(void)
{
  motor_api::init(k_motors, k_motor_count);
}
