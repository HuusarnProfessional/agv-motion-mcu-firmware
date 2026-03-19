#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace platform_stm32_hal
{
  static void pwm_start(void *ctx, std::uint32_t channel)
  {
    TIM_HandleTypeDef *htim = static_cast<TIM_HandleTypeDef *>(ctx);
    HAL_TIM_PWM_Start(htim, channel);
  }

  static void pwm_set_compare(void *ctx, std::uint32_t channel, std::uint32_t ccr)
  {
    TIM_HandleTypeDef *htim = static_cast<TIM_HandleTypeDef *>(ctx);
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
  }

  static std::uint32_t pwm_get_period(void *ctx)
  {
    TIM_HandleTypeDef *htim = static_cast<TIM_HandleTypeDef *>(ctx);
    return __HAL_TIM_GET_AUTORELOAD(htim);
  }

  static const motor_api::pwm_ops k_ops =
  {
    pwm_start,
    pwm_set_compare,
    pwm_get_period
  };

  const motor_api::pwm_ops *get_pwm_ops(void)
  {
    return &k_ops;
  }
}
