#include "core/control/local_positioning/encoder_model/input_storage_encoders.hpp"

namespace
{
  void read_current_wheel_samples_from_encoder_api(encoder_input_storage::wheel_sample_array &out_wheels)
  {
    out_wheels = {};

    for (std::size_t i = 0u; i < encoder_input_storage::k_wheel_count; ++i)
    {
      const std::uint8_t wheel_id = static_cast<std::uint8_t>(i);
      encoder_api::encoder_sample sample = {};
      encoder_api::read_sample(wheel_id, sample);
      out_wheels[i].angle_raw_12bit = sample.angle_raw_12bit;
      out_wheels[i].status = sample.status;
      out_wheels[i].time_ms = sample.time_ms;
    }
  }
}

namespace encoder_input_storage
{
  void reset(encoder_snapshot &state)
  {
    state = {};
  }

  void sample_from_encoder_api(encoder_snapshot &state, std::uint32_t tick_id)
  {
    if (state.has_current)
    {
      state.previous_wheels = state.current_wheels;
      state.previous_tick_id = state.current_tick_id;
      state.has_previous = true;
    }

    wheel_sample_array current_wheels = {};
    read_current_wheel_samples_from_encoder_api(current_wheels);

    state.current_wheels = current_wheels;
    state.current_tick_id = tick_id;
    state.has_current = true;
  }

  bool read_previous(const encoder_snapshot &state, wheel_sample_array &out_wheels, std::uint32_t &out_tick_id)
  {
    if (!state.has_previous)
    {
      return false;
    }

    out_wheels = state.previous_wheels;
    out_tick_id = state.previous_tick_id;
    return true;
  }

  bool read_current(const encoder_snapshot &state, wheel_sample_array &out_wheels, std::uint32_t &out_tick_id)
  {
    if (!state.has_current)
    {
      return false;
    }

    out_wheels = state.current_wheels;
    out_tick_id = state.current_tick_id;
    return true;
  }
}
