#include "board/stm32f3discovery/board_stm32f3discovery.hpp"

#include "app/app_entry.hpp"
#include "core/api/encoder_api.hpp"
#include "core/api/motor_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C" {
#include "main.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
}

namespace
{
  static TIM_HandleTypeDef *k_tim1 = &htim1;
  static TIM_HandleTypeDef *k_tim2 = &htim2;
  static TIM_HandleTypeDef *k_tim4 = &htim4;
  static TIM_HandleTypeDef *k_tim8 = &htim8;

  // Motor mapping for STM32F3DISCOVERY:
  // motor0: PA8/PA9   -> TIM1_CH1/TIM1_CH2
  // motor1: PC6/PC7   -> TIM8_CH1/TIM8_CH2
  // motor2: PC8/PC9   -> TIM8_CH3/TIM8_CH4
  // motor3: PD12/PD13 -> TIM4_CH1/TIM4_CH2
  static const motor_api::motor_pwm2 k_motors[] =
  {
    {
      { static_cast<void *>(k_tim1), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim1), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim8), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim8), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim8), TIM_CHANNEL_3, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim8), TIM_CHANNEL_4, platform_stm32_hal::get_pwm_ops() }
    },
    {
      { static_cast<void *>(k_tim4), TIM_CHANNEL_1, platform_stm32_hal::get_pwm_ops() },
      { static_cast<void *>(k_tim4), TIM_CHANNEL_2, platform_stm32_hal::get_pwm_ops() }
    }
  };

  static constexpr std::size_t k_motor_count = sizeof(k_motors) / sizeof(k_motors[0]);

  static bool read_capture_normal(void *platform_handle,
                                  std::uint8_t channel,
                                  std::uint32_t &high_ticks,
                                  std::uint32_t &period_ticks,
                                  std::uint32_t &time_ms)
  {
    const encoder_api::capture_operations *platform_ops = platform_stm32_hal::get_encoder_capture_ops();
    if (platform_ops == nullptr || platform_ops->read_capture == nullptr)
    {
      return false;
    }
    return platform_ops->read_capture(platform_handle, channel, high_ticks, period_ticks, time_ms);
  }

  static bool read_capture_inverted(void *platform_handle,
                                    std::uint8_t channel,
                                    std::uint32_t &high_ticks,
                                    std::uint32_t &period_ticks,
                                    std::uint32_t &time_ms)
  {
    if (!read_capture_normal(platform_handle, channel, high_ticks, period_ticks, time_ms))
    {
      return false;
    }

    if (high_ticks <= period_ticks)
    {
      high_ticks = period_ticks - high_ticks;
    }
    return true;
  }

  static const encoder_api::capture_operations k_encoder_cap_normal =
  {
    read_capture_normal
  };

  static const encoder_api::capture_operations k_encoder_cap_inverted =
  {
    read_capture_inverted
  };

  // Encoder PWM-input mapping for STM32F3DISCOVERY:
  // encoder0: PA0 -> TIM2_CH1
  // encoder1: PA1 -> TIM2_CH2
  // encoder2: PA2 -> TIM2_CH3
  // encoder3: PA3 -> TIM2_CH4
  static const encoder_api::encoder_input k_encoders[] =
  {
    // Keep current F3 behavior unchanged (all normal).
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_1), &k_encoder_cap_normal },
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_2), &k_encoder_cap_normal },
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_3), &k_encoder_cap_normal },
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_4), &k_encoder_cap_normal }
  };

  static constexpr std::size_t k_encoder_count = sizeof(k_encoders) / sizeof(k_encoders[0]);

  static constexpr app_drive_defaults k_drive_defaults =
  {
    63,  // wheel_diameter_mm
    164  // wheel_separation_mm
  };
}

extern "C" void board_init(void)
{
  motor_api::init(k_motors, k_motor_count);
  encoder_api::init(k_encoders, k_encoder_count);
  app_init();
  app_apply_drive_defaults(&k_drive_defaults);
}

extern "C" void board_tick(void)
{
  app_step(HAL_GetTick());
}
