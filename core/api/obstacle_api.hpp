#pragma once

#include <cstddef>
#include <cstdint>

namespace obstacle_api
{
  enum class obstacle_status : std::uint8_t
  {
    ok = 0,
    timeout,
    stale,
    no_signal,
    out_of_range,
    invalid_id
  };

  struct obstacle_sample
  {
    std::uint32_t distance_mm;
    std::uint32_t time_ms;
    obstacle_status status;
  };

  enum class ultrasonic_platform_event_type : std::uint8_t
  {
    alarm_elapsed = 0,
    capture_ready
  };

  struct ultrasonic_platform_event
  {
    void *platform_handle;
    std::uint8_t channel;
    std::uint32_t pulse_width_us;
    std::uint32_t time_ms;
    ultrasonic_platform_event_type type;
  };

  using platform_event_callback_fn = void (*)(void *event_context, const ultrasonic_platform_event *event);
  using register_event_callback_fn = bool (*)(void *platform_handle, platform_event_callback_fn event_callback, void *event_context);
  using set_trigger_level_fn = bool (*)(void *platform_handle, std::uint8_t channel, bool is_high);
  using schedule_alarm_us_fn = bool (*)(void *platform_handle, std::uint32_t delay_us);
  using cancel_alarm_fn = bool (*)(void *platform_handle);
  using read_time_ms_fn = bool (*)(void *platform_handle, std::uint32_t &time_ms_out);

  struct ultrasonic_operations
  {
    register_event_callback_fn register_event_callback;
    set_trigger_level_fn set_trigger_level;
    schedule_alarm_us_fn schedule_alarm_us;
    cancel_alarm_fn cancel_alarm;
    read_time_ms_fn read_time_ms;
  };

  struct obstacle_input
  {
    void *platform_handle;
    std::uint8_t channel;
    std::uint32_t echo_timeout_us;
    const ultrasonic_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const obstacle_input *inputs, std::size_t count);
  using backend_read_sample_fn = bool (*)(std::uint8_t sensor_id, obstacle_sample &out);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_read_sample_fn read_sample_fn;
  };

  void init(const obstacle_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t sensor_id, obstacle_sample &out);
}

