#pragma once

#include <cstdint>

#include "core/api/encoder_api.hpp"

namespace rc_encoder_sample_guard_ai
{
  struct config
  {
    bool enabled = true;
    bool reject_zero_glitch = true;
    std::uint16_t zero_edge_window_raw = 96U;
    std::uint32_t jump_check_max_dt_ms = 220U;
    std::int32_t max_delta_raw_short_dt = 700;
    std::uint32_t rebase_gap_ms = 500U;
    std::uint8_t max_bad_run_before_no_signal = 3U;
  };

  struct channel_state
  {
    bool have_prev = false;
    std::uint16_t prev_raw = 0U;
    std::uint32_t prev_time_ms = 0U;

    bool have_last_good = false;
    std::uint16_t last_good_raw = 0U;
    std::uint32_t last_good_mdeg = 0U;
    std::uint32_t last_good_mrad = 0U;

    std::uint8_t bad_run = 0U;
  };

  inline void reset(channel_state &state)
  {
    state.have_prev = false;
    state.prev_raw = 0U;
    state.prev_time_ms = 0U;
    state.have_last_good = false;
    state.last_good_raw = 0U;
    state.last_good_mdeg = 0U;
    state.last_good_mrad = 0U;
    state.bad_run = 0U;
  }

  inline std::int32_t wrapped_delta_raw12(std::uint16_t prev_raw, std::uint16_t curr_raw)
  {
    std::int32_t d = static_cast<std::int32_t>(curr_raw) - static_cast<std::int32_t>(prev_raw);
    if (d > 2048)
    {
      d -= 4096;
    }
    else if (d < -2048)
    {
      d += 4096;
    }
    return d;
  }

  inline std::int32_t abs_i32(std::int32_t v)
  {
    return (v < 0) ? -v : v;
  }

  inline bool is_near_zero_edge(std::uint16_t raw, std::uint16_t edge_window_raw)
  {
    return raw <= edge_window_raw || raw >= static_cast<std::uint16_t>(4096U - edge_window_raw);
  }

  inline void cache_last_good(channel_state &state, const encoder_api::encoder_sample &sample)
  {
    state.have_last_good = true;
    state.last_good_raw = sample.angle_raw_12bit;
    state.last_good_mdeg = sample.angle_mdeg;
    state.last_good_mrad = sample.angle_mrad;
  }

  inline void replace_with_last_good(channel_state &state, encoder_api::encoder_sample &sample)
  {
    if (!state.have_last_good)
    {
      return;
    }

    sample.angle_raw_12bit = state.last_good_raw;
    sample.angle_mdeg = state.last_good_mdeg;
    sample.angle_mrad = state.last_good_mrad;
  }

  inline bool sanitize(channel_state &state,
                       const config &cfg,
                       encoder_api::encoder_sample &sample)
  {
    if (!cfg.enabled)
    {
      if (sample.status == encoder_api::encoder_status::ok)
      {
        state.have_prev = true;
        state.prev_raw = sample.angle_raw_12bit;
        state.prev_time_ms = sample.time_ms;
        state.bad_run = 0U;
        cache_last_good(state, sample);
        return true;
      }

      state.bad_run = static_cast<std::uint8_t>(state.bad_run + 1U);
      return false;
    }

    if (sample.status != encoder_api::encoder_status::ok)
    {
      state.bad_run = static_cast<std::uint8_t>(state.bad_run + 1U);
      replace_with_last_good(state, sample);
      sample.status = (state.bad_run > cfg.max_bad_run_before_no_signal)
                    ? encoder_api::encoder_status::no_signal
                    : encoder_api::encoder_status::stale;
      return false;
    }

    if (!state.have_prev)
    {
      state.have_prev = true;
      state.prev_raw = sample.angle_raw_12bit;
      state.prev_time_ms = sample.time_ms;
      state.bad_run = 0U;
      cache_last_good(state, sample);
      return true;
    }

    const std::uint32_t dt_ms = sample.time_ms - state.prev_time_ms;
    const std::int32_t d_raw = wrapped_delta_raw12(state.prev_raw, sample.angle_raw_12bit);
    bool reject = false;

    if (cfg.reject_zero_glitch &&
        sample.angle_raw_12bit == 0U &&
        !is_near_zero_edge(state.prev_raw, cfg.zero_edge_window_raw))
    {
      reject = true;
    }

    if (!reject &&
        dt_ms <= cfg.jump_check_max_dt_ms &&
        abs_i32(d_raw) > cfg.max_delta_raw_short_dt)
    {
      reject = true;
    }

    if (reject)
    {
      state.bad_run = static_cast<std::uint8_t>(state.bad_run + 1U);
      replace_with_last_good(state, sample);
      sample.status = (state.bad_run > cfg.max_bad_run_before_no_signal)
                    ? encoder_api::encoder_status::no_signal
                    : encoder_api::encoder_status::stale;
      return false;
    }

    // Rebase after long gaps (sample still accepted), avoids large dt artifacts.
    if (dt_ms > cfg.rebase_gap_ms)
    {
      state.prev_raw = sample.angle_raw_12bit;
      state.prev_time_ms = sample.time_ms;
      state.bad_run = 0U;
      cache_last_good(state, sample);
      return true;
    }

    state.prev_raw = sample.angle_raw_12bit;
    state.prev_time_ms = sample.time_ms;
    state.bad_run = 0U;
    cache_last_good(state, sample);
    return true;
  }
}
