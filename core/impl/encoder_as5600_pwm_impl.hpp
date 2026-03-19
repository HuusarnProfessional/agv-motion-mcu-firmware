#pragma once

#include <cstddef>
#include <cstdint>
#include "core/api/encoder_api.hpp"

namespace encoder_as5600_pwm_impl
{
  // TODO (future encoder_as5600_i2c_impl):
  // When configuring AS5600 output mode through I2C (no burn/OTP),
  // write CONF low byte at register 0x08 with an 8-bit write.
  // OUTS is bits [5:4] (set to 0b10 for PWM), PWMF is bits [7:6]
  // (for example 0b01 for 230 Hz). Avoid relying on a 16-bit write
  // starting at 0x07, since some modules do not update 0x08 reliably
  // in that pattern. 
  // see esp32s3 kod for ref.

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
    std::uint32_t time_ms;
    sample_status status;
  };


  
  
  void init(const encoder_api::encoder_input *encoders, std::size_t count);
  bool read_sample(std::uint8_t encoder_id, sample &out);

}
