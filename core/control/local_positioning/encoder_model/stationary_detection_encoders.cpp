#include "core/control/local_positioning/encoder_model/stationary_detection_encoders.hpp"

#include <cstddef>

#include "core/control/local_positioning/encoder_model/encoder_model_tuning.hpp"
#include "core/mechanical_config/mechanical_config.hpp"

namespace
{
  const encoder_model_tuning::stationary_detection_tuning tuning = {};
  const mechanical_config::drivetrain drivetrain = {};

  constexpr std::size_t k_front_left_index = 0u;
  constexpr std::size_t k_front_right_index = 1u;
  constexpr std::size_t k_rear_left_index = 2u;
  constexpr std::size_t k_rear_right_index = 3u;
  constexpr std::int64_t k_um_per_mm = 1000LL;
  constexpr std::int64_t k_urad_per_rad = 1000000LL;
  constexpr std::int64_t k_max_pending_multiplier = 4LL;

  std::int64_t abs_i64(std::int64_t value)
  {
    if (value < 0)
    {
      return -value;
    }

    return value;
  }

  std::int64_t clamp_i64(std::int64_t value, std::int64_t lower_bound, std::int64_t upper_bound)
  {
    if (value < lower_bound)
    {
      return lower_bound;
    }

    if (value > upper_bound)
    {
      return upper_bound;
    }

    return value;
  }

  bool has_usable_delta(delta_estimation_encoders::delta_status status)
  {
    return status != delta_estimation_encoders::delta_status::bad;
  }

  std::int64_t &pending_for_wheel(stationary_detection_encoders::state &classifier_state, std::size_t wheel_index)
  {
    if (wheel_index == k_front_left_index)
    {
      return classifier_state.pending_fl_um;
    }

    if (wheel_index == k_front_right_index)
    {
      return classifier_state.pending_fr_um;
    }

    if (wheel_index == k_rear_left_index)
    {
      return classifier_state.pending_rl_um;
    }

    return classifier_state.pending_rr_um;
  }

  void clear_pending(stationary_detection_encoders::state &classifier_state)
  {
    classifier_state.pending_fl_um = 0;
    classifier_state.pending_fr_um = 0;
    classifier_state.pending_rl_um = 0;
    classifier_state.pending_rr_um = 0;
  }

  std::int64_t average_valid_pending(std::int64_t first_pending_um, std::int64_t second_pending_um, bool has_first, bool has_second, bool &has_average)
  {
    has_average = false;

    if (has_first && has_second)
    {
      has_average = true;
      return (first_pending_um + second_pending_um) / 2;
    }

    if (has_first)
    {
      has_average = true;
      return first_pending_um;
    }

    if (has_second)
    {
      has_average = true;
      return second_pending_um;
    }

    return 0;
  }

  std::int64_t compute_rotation_abs_urad(std::int64_t left_pending_um, std::int64_t right_pending_um)
  {
    const std::int64_t wheel_separation_um = static_cast<std::int64_t>(drivetrain.wheel_separation_mm) * k_um_per_mm;

    if (wheel_separation_um == 0)
    {
      return 0;
    }

    return abs_i64((right_pending_um - left_pending_um) * k_urad_per_rad / wheel_separation_um);
  }

  std::int64_t compute_max_pending_wheel_um(void)
  {
    const std::int64_t wheel_separation_um = static_cast<std::int64_t>(drivetrain.wheel_separation_mm) * k_um_per_mm;
    std::int64_t start_motion_rotation_threshold_um = 0;

    if (wheel_separation_um != 0)
    {
      start_motion_rotation_threshold_um =
        (tuning.start_motion_rotation_threshold_urad * wheel_separation_um) / k_urad_per_rad;
    }

    std::int64_t dominant_threshold_um = tuning.start_motion_translation_threshold_um;

    if (start_motion_rotation_threshold_um > dominant_threshold_um)
    {
      dominant_threshold_um = start_motion_rotation_threshold_um;
    }

    return dominant_threshold_um * k_max_pending_multiplier;
  }

  void clamp_pending(stationary_detection_encoders::state &classifier_state)
  {
    const std::int64_t max_pending_wheel_um = compute_max_pending_wheel_um();
    const std::int64_t min_pending_wheel_um = -max_pending_wheel_um;

    classifier_state.pending_fl_um = clamp_i64(classifier_state.pending_fl_um, min_pending_wheel_um, max_pending_wheel_um);
    classifier_state.pending_fr_um = clamp_i64(classifier_state.pending_fr_um, min_pending_wheel_um, max_pending_wheel_um);
    classifier_state.pending_rl_um = clamp_i64(classifier_state.pending_rl_um, min_pending_wheel_um, max_pending_wheel_um);
    classifier_state.pending_rr_um = clamp_i64(classifier_state.pending_rr_um, min_pending_wheel_um, max_pending_wheel_um);
  }
}

namespace stationary_detection_encoders
{
  void reset(state &classifier_state)
  {
    classifier_state = {};
  }

  bool classify_delta(state &classifier_state, const delta_estimation_encoders::delta_snapshot &input_delta, snapshot &out)
  {
    out = {};

    if (!input_delta.has_delta)
    {
      return false;
    }

    for (std::size_t i = 0u; i < input_delta.wheel_deltas.size(); ++i)
    {
      if (!has_usable_delta(input_delta.wheel_deltas[i].status))
      {
        continue;
      }

      pending_for_wheel(classifier_state, i) += input_delta.wheel_delta_motion[i].wheel_delta_distance_um;
    }

    const bool has_fl = has_usable_delta(input_delta.wheel_deltas[k_front_left_index].status);
    const bool has_fr = has_usable_delta(input_delta.wheel_deltas[k_front_right_index].status);
    const bool has_rl = has_usable_delta(input_delta.wheel_deltas[k_rear_left_index].status);
    const bool has_rr = has_usable_delta(input_delta.wheel_deltas[k_rear_right_index].status);

    bool has_left_pending = false;
    bool has_right_pending = false;
    const std::int64_t left_pending_um =
      average_valid_pending(classifier_state.pending_fl_um, classifier_state.pending_rl_um, has_fl, has_rl, has_left_pending);
    const std::int64_t right_pending_um =
      average_valid_pending(classifier_state.pending_fr_um, classifier_state.pending_rr_um, has_fr, has_rr, has_right_pending);

    if (!has_left_pending && !has_right_pending)
    {
      return false;
    }

    std::int64_t translation_abs_um = 0;

    if (has_left_pending && has_right_pending)
    {
      translation_abs_um = abs_i64((left_pending_um + right_pending_um) / 2);
    }
    else if (has_left_pending)
    {
      translation_abs_um = abs_i64(left_pending_um);
    }
    else
    {
      translation_abs_um = abs_i64(right_pending_um);
    }

    std::int64_t rotation_abs_urad = 0;

    if (has_left_pending && has_right_pending)
    {
      rotation_abs_urad = compute_rotation_abs_urad(left_pending_um, right_pending_um);
    }

    const std::int64_t translation_threshold_um =
      classifier_state.is_moving ? tuning.stay_motion_translation_threshold_um : tuning.start_motion_translation_threshold_um;
    const std::int64_t rotation_threshold_urad =
      classifier_state.is_moving ? tuning.stay_motion_rotation_threshold_urad : tuning.start_motion_rotation_threshold_urad;
    const bool has_meaningful_motion =
      translation_abs_um >= translation_threshold_um || rotation_abs_urad >= rotation_threshold_urad;

    if (has_meaningful_motion)
    {
      out.has_classified_delta = true;
      out.is_stationary = false;
      out.has_motion = true;
      out.classified_delta = input_delta;
      out.classified_delta.wheel_delta_motion[k_front_left_index].wheel_delta_distance_um = classifier_state.pending_fl_um;
      out.classified_delta.wheel_delta_motion[k_front_right_index].wheel_delta_distance_um = classifier_state.pending_fr_um;
      out.classified_delta.wheel_delta_motion[k_rear_left_index].wheel_delta_distance_um = classifier_state.pending_rl_um;
      out.classified_delta.wheel_delta_motion[k_rear_right_index].wheel_delta_distance_um = classifier_state.pending_rr_um;
      out.classified_delta.has_delta = true;

      classifier_state.is_moving = true;
      classifier_state.stationary_tick_count = 0u;
      clear_pending(classifier_state);
      return true;
    }

    if (classifier_state.stationary_tick_count < tuning.stationary_ticks_required)
    {
      classifier_state.stationary_tick_count += 1u;
    }

    if (classifier_state.stationary_tick_count >= tuning.stationary_ticks_required)
    {
      classifier_state.is_moving = false;
    }

    out.is_stationary = !classifier_state.is_moving;
    clamp_pending(classifier_state);
    return false;
  }
}
