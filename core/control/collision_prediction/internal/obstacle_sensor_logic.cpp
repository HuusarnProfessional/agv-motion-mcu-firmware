#include "core/control/collision_prediction/internal/obstacle_sensor_logic.hpp"

namespace
{
  bool is_sample_fresh(std::uint32_t now_ms, const obstacle_api::obstacle_sample &sample);
  std::uint32_t compute_required_distance_mm(std::uint32_t approach_speed_mm_s);
  bool is_raw_distance_hazard(const obstacle_sensor_logic::input &evaluation_input, const obstacle_sensor_logic::output &out);
  bool is_filtered_distance_hazard(const obstacle_sensor_logic::output &out);
  void update_hazard_confirmation(obstacle_sensor_logic::state &sensor_state, bool has_raw_hazard, bool has_filtered_hazard);

  bool is_valid_sample(const obstacle_api::obstacle_sample &sample)
  {
    if (sample.status != obstacle_api::obstacle_status::ok)
    {
      return false;
    }

    if (sample.distance_mm == 0u)
    {
      return false;
    }

    return true;
  }

  bool prepare_evaluation(const obstacle_sensor_logic::input &evaluation_input, obstacle_sensor_logic::output &out)
  {
    out = {};
    out.is_relevant = false;
    out.has_valid_sample = is_valid_sample(evaluation_input.sample);
    out.is_sample_fresh = false;
    out.required_distance_mm = compute_required_distance_mm(evaluation_input.approach_speed_mm_s);

    if (!evaluation_input.config.is_enabled)
    {
      return false;
    }

    if (evaluation_input.config.is_masked)
    {
      return false;
    }

    if (evaluation_input.approach_speed_mm_s > static_cast<std::uint32_t>(collision_tuning::k_min_approach_speed_for_obstacle_check_mm_s))
    {
      out.is_relevant = true;
    }

    if (out.has_valid_sample)
    {
      if (is_sample_fresh(evaluation_input.now_ms, evaluation_input.sample))
      {
        out.is_sample_fresh = true;
      }
    }

    if (!out.has_valid_sample || !out.is_sample_fresh)
    {
      return false;
    }

    return true;
  }

  bool is_sample_fresh(std::uint32_t now_ms, const obstacle_api::obstacle_sample &sample)
  {
    if (sample.time_ms > now_ms)
    {
      return false;
    }

    if (now_ms - sample.time_ms > collision_tuning::k_sample_stale_timeout_ms)
    {
      return false;
    }

    return true;
  }

  bool is_raw_distance_hazard(const obstacle_sensor_logic::input &evaluation_input, const obstacle_sensor_logic::output &out)
  {
    if (!out.is_relevant)
    {
      return false;
    }

    if (evaluation_input.sample.distance_mm <= out.required_distance_mm)
    {
      return true;
    }

    return false;
  }

  bool is_filtered_distance_hazard(const obstacle_sensor_logic::output &out)
  {
    if (!out.is_relevant)
    {
      return false;
    }

    if (out.filtered_distance_mm <= out.required_distance_mm)
    {
      return true;
    }

    return false;
  }

  void update_hazard_confirmation(obstacle_sensor_logic::state &sensor_state, bool has_raw_hazard, bool has_filtered_hazard)
  {
    if (!has_raw_hazard || !has_filtered_hazard)
    {
      sensor_state.consecutive_hazard_sample_count = 0u;
      return;
    }

    if (sensor_state.consecutive_hazard_sample_count < collision_tuning::k_required_consecutive_hazard_samples)
    {
      sensor_state.consecutive_hazard_sample_count += 1u;
    }
  }

  std::uint32_t apply_low_pass_filter(std::uint32_t raw_distance_mm, const obstacle_sensor_logic::state &sensor_state)
  {
    if (!sensor_state.has_filtered_distance)
    {
      return raw_distance_mm;
    }

    if (raw_distance_mm < sensor_state.filtered_distance_mm)
    {
      return (600u * raw_distance_mm + 400u * sensor_state.filtered_distance_mm + 500u) / 1000u;
    }

    return (200u * raw_distance_mm + 800u * sensor_state.filtered_distance_mm + 500u) / 1000u;
  }

  std::uint32_t compute_required_distance_mm(std::uint32_t approach_speed_mm_s)
  {
    const std::uint64_t latency_distance_mm = (static_cast<std::uint64_t>(approach_speed_mm_s) * collision_tuning::k_total_system_latency_ms + 999u) / 1000u;
    const std::uint64_t brake_distance_mm = (static_cast<std::uint64_t>(approach_speed_mm_s) * approach_speed_mm_s + static_cast<std::uint64_t>(2u) * collision_tuning::k_min_brake_mm_s2 - 1u) / (static_cast<std::uint64_t>(2u) * collision_tuning::k_min_brake_mm_s2);
    return static_cast<std::uint32_t>(collision_tuning::k_static_margin_mm + latency_distance_mm + brake_distance_mm);
  }
}

namespace obstacle_sensor_logic
{
  void reset(state &sensor_state)
  {
    sensor_state = {};
  }

  void evaluate(state &sensor_state, const input &evaluation_input, output &out)
  {
    if (!prepare_evaluation(evaluation_input, out))
    {
      sensor_state.consecutive_hazard_sample_count = 0u;
      return;
    }

    sensor_state.filtered_distance_mm = apply_low_pass_filter(evaluation_input.sample.distance_mm, sensor_state);
    sensor_state.has_filtered_distance = true;
    sensor_state.has_seen_valid_sample = true;
    sensor_state.last_valid_sample_time_ms = evaluation_input.sample.time_ms;
    out.filtered_distance_mm = sensor_state.filtered_distance_mm;
    const bool has_raw_hazard = is_raw_distance_hazard(evaluation_input, out);
    const bool has_filtered_hazard = is_filtered_distance_hazard(out);
    update_hazard_confirmation(sensor_state, has_raw_hazard, has_filtered_hazard);

    if (sensor_state.consecutive_hazard_sample_count >= collision_tuning::k_required_consecutive_hazard_samples)
    {
      out.hazard_detected = true;
    }
  }
}
