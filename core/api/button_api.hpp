#pragma once

#include <cstddef>
#include <cstdint>

namespace button_api
{
  enum class button_status : std::uint8_t
  {
    ok = 0,
    no_signal,
    invalid_id
  };

  struct button_sample
  {
    bool is_pressed;
    bool was_pressed_since_last_read;
    bool was_released_since_last_read;
    std::uint32_t time_ms;
    button_status status;
  };

  enum class gpio_event_status : std::uint8_t
  {
    none = 0,
    ok,
    invalid_id
  };

  struct gpio_event
  {
    bool is_high;
    std::uint32_t time_ms;
    gpio_event_status status;
  };

  using register_event_source_fn = bool (*)(void *platform_handle);
  using read_gpio_level_fn = bool (*)(void *platform_handle, bool &is_high_out);
  using read_gpio_event_fn = bool (*)(void *platform_handle, gpio_event &event_out);

  struct gpio_operations
  {
    register_event_source_fn register_event_source;
    read_gpio_level_fn read_gpio_level;
    read_gpio_event_fn read_gpio_event;
  };

  struct button_input
  {
    void *platform_handle;
    bool pressed_is_high;
    const gpio_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const button_input *inputs, std::size_t count);
  using backend_read_sample_fn = bool (*)(std::uint8_t button_id, button_sample &out);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_read_sample_fn read_sample_fn;
  };

  void init(const button_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t button_id, button_sample &out);
}
