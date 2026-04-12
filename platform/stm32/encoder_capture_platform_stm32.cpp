#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  struct encoder_capture_state
  {
    TIM_HandleTypeDef *owner;
    std::uint32_t rise_ticks;
    std::uint32_t fall_ticks;
    std::uint32_t high_ticks;
    std::uint32_t period_ticks;
    std::uint32_t time_ms;
    bool started;
    bool waiting_for_falling;
    bool have_rise;
    bool have_fall_after_rise;
    bool valid;
  };

  static volatile encoder_capture_state g_encoder_capture[4] = {};

  static bool channel_to_index(std::uint32_t tim_channel, std::size_t &index)
  {
    switch (tim_channel)
    {
      case TIM_CHANNEL_1:
        index = 0U;
        return true;
      case TIM_CHANNEL_2:
        index = 1U;
        return true;
      case TIM_CHANNEL_3:
        index = 2U;
        return true;
      case TIM_CHANNEL_4:
        index = 3U;
        return true;
      default:
        return false;
    }
  }

  static bool active_channel_to_index_and_channel(std::uint32_t active_channel, std::size_t &index, std::uint32_t &tim_channel)
  {
    switch (active_channel)
    {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        tim_channel = TIM_CHANNEL_1;
        index = 0U;
        return true;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        tim_channel = TIM_CHANNEL_2;
        index = 1U;
        return true;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        tim_channel = TIM_CHANNEL_3;
        index = 2U;
        return true;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        tim_channel = TIM_CHANNEL_4;
        index = 3U;
        return true;
      default:
        return false;
    }
  }

  static void reset_capture_state(volatile encoder_capture_state &state, TIM_HandleTypeDef *owner)
  {
    state.owner = owner;
    state.rise_ticks = 0U;
    state.fall_ticks = 0U;
    state.high_ticks = 0U;
    state.period_ticks = 0U;
    state.time_ms = 0U;
    state.started = false;
    state.waiting_for_falling = false;
    state.have_rise = false;
    state.have_fall_after_rise = false;
    state.valid = false;
  }
}

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == nullptr)
  {
    return;
  }

  bool supported_timer = false;
#if defined(TIM2)
  if (htim->Instance == TIM2)
  {
    supported_timer = true;
  }
#endif
#if defined(TIM3)
  if (htim->Instance == TIM3)
  {
    supported_timer = true;
  }
#endif

  if (!supported_timer)
  {
    return;
  }

  std::size_t index = 0U;
  std::uint32_t tim_channel = 0U;
  if (!active_channel_to_index_and_channel(htim->Channel, index, tim_channel))
  {
    return;
  }

  volatile encoder_capture_state &state = g_encoder_capture[index];
  if (state.owner != htim)
  {
    reset_capture_state(state, htim);
    state.started = true;
  }

  const std::uint32_t captured = HAL_TIM_ReadCapturedValue(htim, tim_channel);
  if (!state.waiting_for_falling)
  {
    if (state.have_rise && state.have_fall_after_rise)
    {
      const std::uint32_t period = captured - state.rise_ticks;
      const std::uint32_t high = state.fall_ticks - state.rise_ticks;

      if (period != 0U && high <= period)
      {
        state.period_ticks = period;
        state.high_ticks = high;
        state.time_ms = HAL_GetTick();
        state.valid = true;
      }
    }

    state.rise_ticks = captured;
    state.have_rise = true;
    state.have_fall_after_rise = false;
    state.waiting_for_falling = true;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else
  {
    state.fall_ticks = captured;
    state.have_fall_after_rise = true;
    state.waiting_for_falling = false;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
  }
}

namespace platform_stm32_hal
{
  static bool encoder_read_capture(void *ctx, std::uint8_t channel, std::uint32_t &high_ticks, std::uint32_t &period_ticks, std::uint32_t &time_ms)
  {
    high_ticks = 0U;
    period_ticks = 0U;
    time_ms = HAL_GetTick();

    if (ctx == nullptr)
    {
      return false;
    }

    const std::uint32_t tim_channel = static_cast<std::uint32_t>(channel);
    std::size_t index = 0U;
    if (!channel_to_index(tim_channel, index))
    {
      return false;
    }

    TIM_HandleTypeDef *htim = static_cast<TIM_HandleTypeDef *>(ctx);
    volatile encoder_capture_state &state = g_encoder_capture[index];

    if (state.owner != htim)
    {
      __disable_irq();
      reset_capture_state(state, htim);
      __enable_irq();
    }

    if (!state.started)
    {
      if (HAL_TIM_IC_Start_IT(htim, tim_channel) != HAL_OK)
      {
        return false;
      }
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);

      __disable_irq();
      state.started = true;
      __enable_irq();
    }

    bool valid = false;
    std::uint32_t high = 0U;
    std::uint32_t period = 0U;
    std::uint32_t sample_time = 0U;

    __disable_irq();
    valid = state.valid;
    high = state.high_ticks;
    period = state.period_ticks;
    sample_time = state.time_ms;
    __enable_irq();

    if (!valid || period == 0U || high > period)
    {
      return false;
    }

    high_ticks = high;
    period_ticks = period;
    time_ms = sample_time;
    return true;
  }

  static const encoder_api::capture_operations k_encoder_capture_ops = { encoder_read_capture };

  const encoder_api::capture_operations *get_encoder_capture_ops(void)
  {
    return &k_encoder_capture_ops;
  }
}
