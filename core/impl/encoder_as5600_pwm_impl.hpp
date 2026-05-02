#pragma once

#include <cstddef>
#include <cstdint>
#include "core/api/encoder_api.hpp"

namespace encoder_as5600_pwm_impl
{

  //state for encoder
  enum class sample_status : std::uint8_t
  {
    ok =0,
    no_signal,
    stale,
    invalid_duty,
    invalid_id
  };
  struct sample
  {
    std::uint16_t angle_raw_12bit;
    std::uint32_t sample_id;
    std::uint32_t time_ms;
    sample_status status;
  };


  
  
  void init(const encoder_api::encoder_input *encoders, std::size_t count);
  bool read_sample(std::uint8_t encoder_id, sample &out);

}
