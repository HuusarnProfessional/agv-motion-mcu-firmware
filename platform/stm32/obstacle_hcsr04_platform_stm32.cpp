#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace
{
  constexpr std::size_t k_max_registered_handles = 8U;
  constexpr std::size_t k_max_registered_sensors = 16U;

  struct handle_runtime
  {
    const platform_stm32_hal::obstacle_hcsr04_handle *owner_handle = nullptr;
    obstacle_api::platform_event_callback_fn event_callback = nullptr;
    void *event_context = nullptr;
    bool in_use = false;
  };

  struct sensor_runtime
  {
    const platform_stm32_hal::obstacle_hcsr04_handle *owner_handle = nullptr;
    std::uint8_t channel = 0U;
    std::uint32_t echo_rise_time_us = 0U;
    bool waiting_for_echo_fall = false;
    bool in_use = false;
  };

  handle_runtime global_handle_runtime[k_max_registered_handles] = {};
  sensor_runtime global_sensor_runtime[k_max_registered_sensors] = {};
  const platform_stm32_hal::obstacle_hcsr04_handle *global_alarm_owner_handle = nullptr;

  static bool write_gpio_level(const platform_stm32_hal::gpio_pin_handle &selected_pin, GPIO_PinState level)
  {
    if (selected_pin.port == nullptr)
    {
      return false;
    }

    HAL_GPIO_WritePin(static_cast<GPIO_TypeDef *>(selected_pin.port), selected_pin.pin, level);
    return true;
  }

  static bool read_gpio_level(const platform_stm32_hal::gpio_pin_handle &selected_pin, GPIO_PinState &level_out)
  {
    if (selected_pin.port == nullptr)
    {
      return false;
    }

    level_out = HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef *>(selected_pin.port), selected_pin.pin);
    return true;
  }

  static bool get_selected_handle(void *platform_handle, const platform_stm32_hal::obstacle_hcsr04_handle *&selected_handle_out)
  {
    if (platform_handle == nullptr)
    {
      return false;
    }

    selected_handle_out = static_cast<const platform_stm32_hal::obstacle_hcsr04_handle *>(platform_handle);

    if (selected_handle_out->sensors == nullptr || selected_handle_out->sensor_count == 0U)
    {
      return false;
    }

    return true;
  }

  static bool get_selected_sensor(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle, std::uint8_t channel, const platform_stm32_hal::obstacle_pin_map *&selected_sensor_out)
  {
    if (channel >= selected_handle.sensor_count)
    {
      return false;
    }

    selected_sensor_out = &selected_handle.sensors[channel];
    return true;
  }

  static bool read_time_us_from_cycle_counter(std::uint32_t &time_us_out)
  {
#if defined(DWT) && defined(CoreDebug)
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
      DWT->CYCCNT = 0U;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
      return false;
    }

    const std::uint32_t cycles_per_microsecond = SystemCoreClock / 1000000U;

    if (cycles_per_microsecond == 0U)
    {
      return false;
    }

    time_us_out = DWT->CYCCNT / cycles_per_microsecond;
    return true;
#else
    (void)time_us_out;
    return false;
#endif
  }

  static TIM_HandleTypeDef *get_alarm_timer(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle)
  {
    return static_cast<TIM_HandleTypeDef *>(selected_handle.alarm_timer_handle);
  }

  static handle_runtime *find_handle_runtime(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle)
  {
    for (std::size_t handle_index = 0U; handle_index < k_max_registered_handles; ++handle_index)
    {
      handle_runtime &selected_handle_runtime = global_handle_runtime[handle_index];

      if (selected_handle_runtime.in_use && selected_handle_runtime.owner_handle == &selected_handle)
      {
        return &selected_handle_runtime;
      }
    }

    return nullptr;
  }

  static handle_runtime *get_or_create_handle_runtime(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle)
  {
    handle_runtime *selected_handle_runtime = find_handle_runtime(selected_handle);

    if (selected_handle_runtime != nullptr)
    {
      return selected_handle_runtime;
    }

    for (std::size_t handle_index = 0U; handle_index < k_max_registered_handles; ++handle_index)
    {
      handle_runtime &available_handle_runtime = global_handle_runtime[handle_index];

      if (available_handle_runtime.in_use)
      {
        continue;
      }

      available_handle_runtime = {};
      available_handle_runtime.owner_handle = &selected_handle;
      available_handle_runtime.in_use = true;
      return &available_handle_runtime;
    }

    return nullptr;
  }

  static sensor_runtime *find_sensor_runtime(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle, std::uint8_t channel)
  {
    for (std::size_t sensor_index = 0U; sensor_index < k_max_registered_sensors; ++sensor_index)
    {
      sensor_runtime &selected_sensor_runtime = global_sensor_runtime[sensor_index];

      if (selected_sensor_runtime.in_use && selected_sensor_runtime.owner_handle == &selected_handle && selected_sensor_runtime.channel == channel)
      {
        return &selected_sensor_runtime;
      }
    }

    return nullptr;
  }

  static sensor_runtime *get_or_create_sensor_runtime(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle, std::uint8_t channel)
  {
    sensor_runtime *selected_sensor_runtime = find_sensor_runtime(selected_handle, channel);

    if (selected_sensor_runtime != nullptr)
    {
      return selected_sensor_runtime;
    }

    for (std::size_t sensor_index = 0U; sensor_index < k_max_registered_sensors; ++sensor_index)
    {
      sensor_runtime &available_sensor_runtime = global_sensor_runtime[sensor_index];

      if (available_sensor_runtime.in_use)
      {
        continue;
      }

      available_sensor_runtime = {};
      available_sensor_runtime.owner_handle = &selected_handle;
      available_sensor_runtime.channel = channel;
      available_sensor_runtime.in_use = true;
      return &available_sensor_runtime;
    }

    return nullptr;
  }

  static void notify_platform_event(const platform_stm32_hal::obstacle_hcsr04_handle &selected_handle, const obstacle_api::ultrasonic_platform_event &event)
  {
    handle_runtime *selected_handle_runtime = find_handle_runtime(selected_handle);

    if (selected_handle_runtime == nullptr || selected_handle_runtime->event_callback == nullptr)
    {
      return;
    }

    selected_handle_runtime->event_callback(selected_handle_runtime->event_context, &event);
  }
  
  static void handle_gpio_exti(void *callback_context, std::uint16_t gpio_pin)
  {
    sensor_runtime *selected_sensor_runtime = static_cast<sensor_runtime *>(callback_context);

    if (selected_sensor_runtime == nullptr || !selected_sensor_runtime->in_use || selected_sensor_runtime->owner_handle == nullptr)
    {
      return;
    }

    const platform_stm32_hal::obstacle_pin_map *selected_sensor = nullptr;

    if (!get_selected_sensor(*selected_sensor_runtime->owner_handle, selected_sensor_runtime->channel, selected_sensor) || selected_sensor->echo.pin != gpio_pin)
    {
      return;
    }

    GPIO_PinState echo_level = GPIO_PIN_RESET;

    if (!read_gpio_level(selected_sensor->echo, echo_level))
    {
      return;
    }

    std::uint32_t current_time_us = 0U;

    if (!read_time_us_from_cycle_counter(current_time_us))
    {
      return;
    }

    if (echo_level == GPIO_PIN_SET)
    {
      selected_sensor_runtime->waiting_for_echo_fall = true;
      selected_sensor_runtime->echo_rise_time_us = current_time_us;
      return;
    }

    if (!selected_sensor_runtime->waiting_for_echo_fall)
    {
      return;
    }

    selected_sensor_runtime->waiting_for_echo_fall = false;

    const obstacle_api::ultrasonic_platform_event event = { static_cast<void *>(const_cast<platform_stm32_hal::obstacle_hcsr04_handle *>(selected_sensor_runtime->owner_handle)), selected_sensor_runtime->channel, current_time_us - selected_sensor_runtime->echo_rise_time_us, HAL_GetTick(), obstacle_api::ultrasonic_platform_event_type::capture_ready };
    notify_platform_event(*selected_sensor_runtime->owner_handle, event);
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (global_alarm_owner_handle == nullptr)
  {
    return;
  }

  TIM_HandleTypeDef *selected_timer = get_alarm_timer(*global_alarm_owner_handle);

  if (selected_timer == nullptr || selected_timer != htim)
  {
    return;
  }

  HAL_TIM_Base_Stop_IT(selected_timer);

  const platform_stm32_hal::obstacle_hcsr04_handle *selected_handle = global_alarm_owner_handle;
  global_alarm_owner_handle = nullptr;

  const obstacle_api::ultrasonic_platform_event event = { static_cast<void *>(const_cast<platform_stm32_hal::obstacle_hcsr04_handle *>(selected_handle)), 0U, 0U, HAL_GetTick(), obstacle_api::ultrasonic_platform_event_type::alarm_elapsed };
  notify_platform_event(*selected_handle, event);
}

namespace platform_stm32_hal
{
  bool obstacle_register_event_callback(void *platform_handle, obstacle_api::platform_event_callback_fn event_callback, void *event_context)
  {
    const obstacle_hcsr04_handle *selected_handle = nullptr;

    if (!get_selected_handle(platform_handle, selected_handle) || event_callback == nullptr)
    {
      return false;
    }

    handle_runtime *selected_handle_runtime = get_or_create_handle_runtime(*selected_handle);

    if (selected_handle_runtime == nullptr)
    {
      return false;
    }

    selected_handle_runtime->event_callback = event_callback;
    selected_handle_runtime->event_context = event_context;

    for (std::uint8_t channel = 0U; channel < selected_handle->sensor_count; ++channel)
    {
      sensor_runtime *selected_sensor_runtime = get_or_create_sensor_runtime(*selected_handle, channel);

      if (selected_sensor_runtime == nullptr)
      {
        return false;
      }

      const obstacle_pin_map *selected_sensor = nullptr;

      if (!get_selected_sensor(*selected_handle, channel, selected_sensor))
      {
        return false;
      }

      if (selected_sensor->echo.port != nullptr && selected_sensor->echo.pin != 0U && !register_gpio_exti_callback(selected_sensor->echo.pin, handle_gpio_exti, selected_sensor_runtime))
      {
        return false;
      }
    }

    return true;
  }

  bool obstacle_set_trigger_level(void *platform_handle, std::uint8_t channel, bool is_high)
  {
    const obstacle_hcsr04_handle *selected_handle = nullptr;

    if (!get_selected_handle(platform_handle, selected_handle))
    {
      return false;
    }

    const obstacle_pin_map *selected_sensor = nullptr;

    if (!get_selected_sensor(*selected_handle, channel, selected_sensor))
    {
      return false;
    }

    GPIO_PinState level = GPIO_PIN_RESET;

    if (is_high)
    {
      level = GPIO_PIN_SET;
    }

    return write_gpio_level(selected_sensor->trigger, level);
  }

  bool obstacle_schedule_alarm_us(void *platform_handle, std::uint32_t delay_us)
  {
    const obstacle_hcsr04_handle *selected_handle = nullptr;

    if (!get_selected_handle(platform_handle, selected_handle) || delay_us == 0U || delay_us > 65535U)
    {
      return false;
    }

    TIM_HandleTypeDef *selected_timer = get_alarm_timer(*selected_handle);

    if (selected_timer == nullptr)
    {
      return false;
    }

    HAL_TIM_Base_Stop_IT(selected_timer);
    __HAL_TIM_DISABLE(selected_timer);
    __HAL_TIM_SET_COUNTER(selected_timer, 0U);
    __HAL_TIM_SET_AUTORELOAD(selected_timer, delay_us - 1U);
    __HAL_TIM_CLEAR_FLAG(selected_timer, TIM_FLAG_UPDATE);
    global_alarm_owner_handle = selected_handle;
    return HAL_TIM_Base_Start_IT(selected_timer) == HAL_OK;
  }

  bool obstacle_cancel_alarm(void *platform_handle)
  {
    const obstacle_hcsr04_handle *selected_handle = nullptr;

    if (!get_selected_handle(platform_handle, selected_handle))
    {
      return false;
    }

    TIM_HandleTypeDef *selected_timer = get_alarm_timer(*selected_handle);

    if (selected_timer == nullptr)
    {
      return false;
    }

    HAL_TIM_Base_Stop_IT(selected_timer);

    if (global_alarm_owner_handle == selected_handle)
    {
      global_alarm_owner_handle = nullptr;
    }

    return true;
  }

  bool obstacle_read_time_ms(void *platform_handle, std::uint32_t &time_ms_out)
  {
    (void)platform_handle;
    time_ms_out = HAL_GetTick();
    return true;
  }
}
