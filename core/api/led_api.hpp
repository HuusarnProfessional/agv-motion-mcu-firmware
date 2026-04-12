#pragma once

#include <cstddef>
#include <cstdint>

namespace led_api
{
  enum class led_status : std::uint8_t
  {
    ok = 0,
    no_signal,
    invalid_id
  };

  struct led_state
  {
    bool is_on;
    led_status status;
  };

  using write_gpio_level_fn = bool (*)(void *platform_handle, bool is_high);
  using read_gpio_level_fn = bool (*)(void *platform_handle, bool &is_high_out);

  struct gpio_operations
  {
    write_gpio_level_fn write_gpio_level;
    read_gpio_level_fn read_gpio_level;
  };

  struct led_output
  {
    void *platform_handle;
    bool active_is_high;
    const gpio_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const led_output *outputs, std::size_t count);
  using backend_set_fn = bool (*)(std::uint8_t led_id, bool is_on);
  using backend_toggle_fn = bool (*)(std::uint8_t led_id);
  using backend_read_state_fn = bool (*)(std::uint8_t led_id, led_state &out);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_set_fn set_fn;
    backend_toggle_fn toggle_fn;
    backend_read_state_fn read_state_fn;
  };

  void init(const led_output *outputs, std::size_t count);
  bool set(std::uint8_t led_id, bool is_on);
  bool toggle(std::uint8_t led_id);
  bool read_state(std::uint8_t led_id, led_state &out);
}
