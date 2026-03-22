#include "core/control/local_positioning/encoder_model/delta_estimation_encoders.hpp"

namespace
{
  constexpr std::int64_t k_encoder_full_scale = 4096;
  constexpr std::int64_t k_encoder_half_scale = 2048;
  constexpr std::int64_t k_pi_x_1000 = 3142;

  mechanical_config::drivetrain g_drivetrain_config = {};

  std::int64_t compute_raw_delta_12bit(std::uint16_t previous_raw, std::uint16_t current_raw)
  {
    std::int64_t delta = static_cast<std::int64_t>(current_raw) - static_cast<std::int64_t>(previous_raw);

    //rollover/jump check
    if (delta > k_encoder_half_scale)
    {
      delta -= k_encoder_full_scale;
    }
    else if (delta < -k_encoder_half_scale)
    {
      delta += k_encoder_full_scale;
    }

    return delta;
  }

  std::uint32_t compute_delta_time_ms(std::uint32_t previous_time_ms, std::uint32_t current_time_ms)
  {
    if (current_time_ms >= previous_time_ms)
    {
      return current_time_ms - previous_time_ms;
    }

    return 0u;
  }

  std::uint8_t status_rank(encoder_api::encoder_status status)
  {
    switch (status)
    {
      case encoder_api::encoder_status::ok:
        return 0u;
      case encoder_api::encoder_status::stale:
        return 1u;
      case encoder_api::encoder_status::no_signal:
      case encoder_api::encoder_status::invalid_duty:
      case encoder_api::encoder_status::invalid_id:
        return 2u;
      default:
        return 2u;
    }
  }

  encoder_api::encoder_status inherit_worst_api_status(encoder_api::encoder_status previous_status, encoder_api::encoder_status current_status)
  {
    const std::uint8_t previous_rank = status_rank(previous_status);
    const std::uint8_t current_rank = status_rank(current_status);

    if (current_rank > previous_rank)
    {
      return current_status;
    }

    if (current_rank < previous_rank)
    {
      return previous_status;
    }

    if (static_cast<std::uint8_t>(current_status) >= static_cast<std::uint8_t>(previous_status))
    {
      return current_status;
    }

    return previous_status;
  }

  delta_estimation_encoders::delta_status map_to_delta_status(encoder_api::encoder_status status)
  {
    switch (status)
    {
      case encoder_api::encoder_status::ok:
        return delta_estimation_encoders::delta_status::ok;

      case encoder_api::encoder_status::stale:
        return delta_estimation_encoders::delta_status::stale;

      case encoder_api::encoder_status::no_signal:
      case encoder_api::encoder_status::invalid_duty:
      case encoder_api::encoder_status::invalid_id:
        return delta_estimation_encoders::delta_status::bad;

      default:
        return delta_estimation_encoders::delta_status::bad;
    }
  }

  bool is_bad_status(delta_estimation_encoders::delta_status status)
  {
    if (status == delta_estimation_encoders::delta_status::bad)
    {
      return true;
    }

    return false;
  }

  std::int64_t compute_delta_distance_um(std::int64_t delta_raw_12bit, const mechanical_config::drivetrain &drivetrain_config)
  {
    if (drivetrain_config.wheel_diameter_mm <= 0)
    {
      return 0;
    }

    const std::int64_t delta_counts = delta_raw_12bit;
    const std::int64_t wheel_diameter_mm = drivetrain_config.wheel_diameter_mm;
    const std::int64_t wheel_circumference_scale = k_pi_x_1000;
    const std::int64_t counts_per_revolution = k_encoder_full_scale;
    const std::int64_t numerator = delta_counts * wheel_diameter_mm * wheel_circumference_scale;
    const std::int64_t denominator = counts_per_revolution;
    return numerator / denominator;
  }

}

namespace delta_estimation_encoders
{
  void reset(delta_snapshot &state)
  {
    state = {};
  }

  void set_drivetrain_config(const mechanical_config::drivetrain &drivetrain_config)
  {
    g_drivetrain_config = drivetrain_config;
  }

  bool estimate_from_encoder_snapshot(const encoder_input_storage::encoder_snapshot &encoder_snapshot, delta_snapshot &out)
  {
    out = {};

    if (!encoder_snapshot.has_previous || !encoder_snapshot.has_current)
    {
      return false;
    }

    out.previous_tick_id = encoder_snapshot.previous_tick_id;
    out.current_tick_id = encoder_snapshot.current_tick_id;

    for (std::size_t i = 0u; i < encoder_input_storage::k_wheel_count; ++i)
    {
      const auto &previous_wheel = encoder_snapshot.previous_wheels[i];
      const auto &current_wheel = encoder_snapshot.current_wheels[i];
      const encoder_api::encoder_status inherited_api_status = inherit_worst_api_status(previous_wheel.status, current_wheel.status);

      out.wheel_deltas[i].delta_raw_12bit = compute_raw_delta_12bit(previous_wheel.angle_raw_12bit, current_wheel.angle_raw_12bit);
      out.wheel_deltas[i].delta_time_ms = compute_delta_time_ms(previous_wheel.time_ms, current_wheel.time_ms);
      out.wheel_deltas[i].status = map_to_delta_status(inherited_api_status);

      if (is_bad_status(out.wheel_deltas[i].status))
      {
        out.wheel_delta_motion[i].wheel_delta_distance_um = 0;
        continue;
      }

      const std::int64_t wheel_delta_distance_um = compute_delta_distance_um(out.wheel_deltas[i].delta_raw_12bit, g_drivetrain_config);
      out.wheel_delta_motion[i].wheel_delta_distance_um = wheel_delta_distance_um;
    }

    out.has_delta = true;
    return true;
  }
}
