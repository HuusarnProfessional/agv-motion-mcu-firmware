#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/led_api.hpp"

namespace led_gpio_impl
{
  enum class state_status : std::uint8_t
  {
    ok = 0,
    no_signal,
    invalid_id
  };

  struct state
  {
    bool is_on;
    state_status status;
  };

  void init(const led_api::led_output *outputs, std::size_t count);
  bool set(std::uint8_t led_id, bool is_on);
  bool toggle(std::uint8_t led_id);
  bool read_state(std::uint8_t led_id, state &out);
}
