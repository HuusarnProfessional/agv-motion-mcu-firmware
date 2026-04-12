#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/button_api.hpp"

namespace button_gpio_impl
{
  enum class sample_status : std::uint8_t
  {
    ok = 0,
    no_signal,
    invalid_id
  };

  struct sample
  {
    bool is_pressed;
    bool was_pressed_since_last_read;
    bool was_released_since_last_read;
    std::uint32_t time_ms;
    sample_status status;
  };

  void init(const button_api::button_input *inputs, std::size_t count);
  bool read_sample(std::uint8_t button_id, sample &out);
}
