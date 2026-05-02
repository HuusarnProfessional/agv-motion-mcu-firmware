#pragma once

#include <cstddef>
#include <cstdint>

namespace encoder_api
{
  
  // Planned for controller layer (not API/impl now):
  // - continuous angle (unwrapped)
  // - angular velocity
  // - reset/rebase policy



  enum class encoder_status : std::uint8_t
  {
    ok =0,
    no_signal,
    stale,
    invalid_duty,
    invalid_id
  };

  struct encoder_sample
  {
    std::uint16_t angle_raw_12bit;
    std::uint32_t angle_mdeg; //single turn 0 -> 360
    std::uint32_t angle_mrad; //single turn 0 -> 2*pi
    std::uint32_t sample_id;
    std::uint32_t time_ms;
    encoder_status status;

  };

  using read_capture_fn = bool (*)(void *platform_handle, std::uint8_t channel, std::uint32_t &high_ticks, std::uint32_t &period_ticks, std::uint32_t &sample_id, std::uint32_t &time_ms);


  struct capture_operations
  {
    read_capture_fn read_capture;
  };

  struct encoder_input
  {
    void *platform_handle;
    std::uint8_t channel;
    const capture_operations *platform_operations;
  };

  using backend_init_fn = void (*)(const encoder_input *encoders, std::size_t count);
  using backend_read_sample_fn = bool (*)(std::uint8_t encoder_id, encoder_sample &out);

  struct backend_operation
  {
    backend_init_fn init_fn;
    backend_read_sample_fn read_sample_fn;
  };

  void init(const encoder_input *encoders, std::size_t count);
  bool read_sample(std::uint8_t encoder_id, encoder_sample &out);
  
}
