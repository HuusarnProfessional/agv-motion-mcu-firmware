#include <array>
#include <cstddef>
#include <cstdint>

#include "core/api/encoder_api.hpp"
#include "core/control/local_positioning/encoder_model/delta_estimation_encoders.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include "core/mechanical_config/mechanical_config.hpp"

namespace
{
  using scripted_tick_samples = std::array<encoder_api::encoder_sample, encoder_input_storage::k_wheel_count>;

  std::array<scripted_tick_samples, 3u> g_scripted_samples = {};
  std::size_t g_active_tick_index = 0u;

  void fill_tick_samples(std::size_t tick_index, std::uint16_t angle_raw_12bit, std::uint32_t sample_id, std::uint32_t time_ms, encoder_api::encoder_status status)
  {
    for (std::size_t i = 0u; i < encoder_input_storage::k_wheel_count; ++i)
    {
      g_scripted_samples[tick_index][i].angle_raw_12bit = angle_raw_12bit;
      g_scripted_samples[tick_index][i].angle_mdeg = 0u;
      g_scripted_samples[tick_index][i].angle_mrad = 0u;
      g_scripted_samples[tick_index][i].sample_id = sample_id;
      g_scripted_samples[tick_index][i].time_ms = time_ms;
      g_scripted_samples[tick_index][i].status = status;
    }
  }
}

namespace encoder_api
{
  void init(const encoder_input *encoders, std::size_t count)
  {
    (void)encoders;
    (void)count;
  }

  bool read_sample(std::uint8_t encoder_id, encoder_sample &out)
  {
    if (encoder_id >= encoder_input_storage::k_wheel_count)
    {
      out = {};
      out.status = encoder_status::invalid_id;
      return false;
    }

    out = g_scripted_samples[g_active_tick_index][encoder_id];
    return true;
  }
}

int main()
{
  fill_tick_samples(0u, 100u, 1u, 10u, encoder_api::encoder_status::ok);
  fill_tick_samples(1u, 200u, 2u, 20u, encoder_api::encoder_status::ok);
  fill_tick_samples(2u, 200u, 2u, 20u, encoder_api::encoder_status::ok);

  encoder_motion::state state = {};
  delta_estimation_encoders::set_drivetrain_config(mechanical_config::drivetrain{});
  encoder_motion::reset(state);

  g_active_tick_index = 0u;
  encoder_motion::tick(state, 1u);

  if (state.encoder_delta_snapshot.has_delta)
  {
    return 1;
  }

  if (state.encoder_confidence_snapshot.has_confidence)
  {
    return 2;
  }

  if (state.encoder_motion_snapshot.has_motion_model)
  {
    return 3;
  }

  g_active_tick_index = 1u;
  encoder_motion::tick(state, 2u);

  if (!state.encoder_delta_snapshot.has_delta)
  {
    return 4;
  }

  if (!state.encoder_confidence_snapshot.has_confidence)
  {
    return 5;
  }

  if (!state.encoder_motion_snapshot.has_motion_model)
  {
    return 6;
  }

  if (state.encoder_motion_snapshot.translation == 0)
  {
    return 7;
  }

  g_active_tick_index = 2u;
  encoder_motion::tick(state, 3u);

  if (state.encoder_delta_snapshot.has_delta)
  {
    return 8;
  }

  if (state.encoder_confidence_snapshot.has_confidence)
  {
    return 9;
  }

  if (state.encoder_motion_snapshot.has_motion_model)
  {
    return 10;
  }

  if (state.encoder_motion_snapshot.translation != 0)
  {
    return 11;
  }

  if (state.encoder_input_snapshot.current_tick_id != 2u)
  {
    return 12;
  }

  if (state.encoder_input_snapshot.current_wheels[0u].sample_id != 2u)
  {
    return 13;
  }

  return 0;
}
