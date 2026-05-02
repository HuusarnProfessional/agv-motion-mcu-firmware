#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/encoder_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern TIM_HandleTypeDef htim2;
}

namespace
{
  static TIM_HandleTypeDef *k_tim2 = &htim2;

  static bool read_capture_normal(void *platform_handle, std::uint8_t channel, std::uint32_t &high_ticks, std::uint32_t &period_ticks, std::uint32_t &sample_id, std::uint32_t &time_ms)
  {
    const encoder_api::capture_operations *platform_ops = platform_stm32_hal::get_encoder_capture_ops();
    if (platform_ops == nullptr || platform_ops->read_capture == nullptr)
    {
      return false;
    }
    return platform_ops->read_capture(platform_handle, channel, high_ticks, period_ticks, sample_id, time_ms);
  }

  static bool read_capture_inverted(void *platform_handle, std::uint8_t channel, std::uint32_t &high_ticks, std::uint32_t &period_ticks, std::uint32_t &sample_id, std::uint32_t &time_ms)
  {
    if (!read_capture_normal(platform_handle, channel, high_ticks, period_ticks, sample_id, time_ms))
    {
      return false;
    }

    if (high_ticks <= period_ticks)
    {
      high_ticks = period_ticks - high_ticks;
    }
    return true;
  }

  static const encoder_api::capture_operations k_encoder_cap_normal = { read_capture_normal };
  static const encoder_api::capture_operations k_encoder_cap_inverted = { read_capture_inverted };

  // encoder order expected:
  // encoder0 = front_left
  // encoder1 = front_right
  // encoder2 = rear_left
  // encoder3 = rear_right
  static const encoder_api::encoder_input k_encoders[] =
  {
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_3), &k_encoder_cap_normal },
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_2), &k_encoder_cap_inverted },
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_4), &k_encoder_cap_normal },
    { static_cast<void *>(k_tim2), static_cast<std::uint8_t>(TIM_CHANNEL_1), &k_encoder_cap_inverted }
  };

  static constexpr std::size_t k_encoder_count = sizeof(k_encoders) / sizeof(k_encoders[0]);
}

void board_stm32g474re_agv_init_encoders(void)
{
  encoder_api::init(k_encoders, k_encoder_count);
}
