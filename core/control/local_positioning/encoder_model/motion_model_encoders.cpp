#include "core/control/local_positioning/encoder_model/motion_model_encoders.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_tuning.hpp"
#include "core/mechanical_config/mechanical_config.hpp"
#include <cmath>

namespace
{
  const encoder_model_tuning::motion_model_tuning tuning = {};
  const mechanical_config::drivetrain drivetrain = {};
  constexpr std::int64_t k_rotate_in_spot_zero_threshold_um = 50;
  constexpr std::int64_t k_rotate_in_spot_active_threshold_um = 200;

  int sign_i64(std::int64_t value)
  {
    if (value > 0)
    {
      return 1;
    }

    if (value < 0)
    {
      return -1;
    }

    return 0;
  }

  std::int64_t abs_i64(std::int64_t value)
  {
    if (value < 0)
    {
      return -value;
    }

    return value;
  }

  bool compute_rotate_in_spot_flag(std::int64_t delta_left_um, std::int64_t delta_right_um)
  {
    const std::int64_t left_abs = abs_i64(delta_left_um);
    const std::int64_t right_abs = abs_i64(delta_right_um);

    if (left_abs <= k_rotate_in_spot_zero_threshold_um && right_abs >= k_rotate_in_spot_active_threshold_um)
    {
      return true;
    }

    if (right_abs <= k_rotate_in_spot_zero_threshold_um && left_abs >= k_rotate_in_spot_active_threshold_um)
    {
      return true;
    }

    if (left_abs < k_rotate_in_spot_active_threshold_um || right_abs < k_rotate_in_spot_active_threshold_um)
    {
      return false;
    }

    const int sign_left = sign_i64(delta_left_um);
    const int sign_right = sign_i64(delta_right_um);

    if (sign_left == 0 || sign_right == 0)
    {
      return false;
    }

    return sign_left == -sign_right;
  }

  void compute_confidences(const confidence_estimation_encoders::confidence_snapshot &confidence_snapshot, std::int64_t delta_fl_um, std::int64_t delta_fr_um, std::int64_t delta_rl_um, std::int64_t delta_rr_um, std::int64_t &out_confidence_slip, std::int64_t &out_confidence_translation, std::int64_t &out_confidence_rotation)
  {
    switch (confidence_snapshot.case_id)
    {
      case confidence_estimation_encoders::confidence_case::all_ok:
        break;

      case confidence_estimation_encoders::confidence_case::one_bad:
        out_confidence_slip = tuning.confidence_rotation_one_bad;
        out_confidence_translation = tuning.confidence_translation_one_bad;
        out_confidence_rotation = tuning.confidence_rotation_one_bad;
        return;

      case confidence_estimation_encoders::confidence_case::two_bad_same_axle:
        out_confidence_slip = tuning.confidence_rotation_two_bad_same_axle;
        out_confidence_translation = tuning.confidence_translation_two_bad_same_axle;
        out_confidence_rotation = tuning.confidence_rotation_two_bad_same_axle;
        return;

      case confidence_estimation_encoders::confidence_case::two_bad_same_side:
        out_confidence_slip = tuning.confidence_rotation_two_bad_same_side;
        out_confidence_translation = tuning.confidence_translation_two_bad_same_side;
        out_confidence_rotation = tuning.confidence_rotation_two_bad_same_side;
        return;

      case confidence_estimation_encoders::confidence_case::two_bad_mixed_axle:
        out_confidence_slip = tuning.confidence_rotation_two_bad_mixed_axle;
        out_confidence_translation = tuning.confidence_translation_two_bad_mixed_axle;
        out_confidence_rotation = tuning.confidence_rotation_two_bad_mixed_axle;
        return;

      case confidence_estimation_encoders::confidence_case::three_bad:
        out_confidence_slip = tuning.confidence_rotation_three_bad;
        out_confidence_translation = tuning.confidence_translation_three_bad;
        out_confidence_rotation = tuning.confidence_rotation_three_bad;
        return;

      case confidence_estimation_encoders::confidence_case::four_bad:
      default:
        out_confidence_slip = tuning.confidence_rotation_four_bad;
        out_confidence_translation = tuning.confidence_translation_four_bad;
        out_confidence_rotation = tuning.confidence_rotation_four_bad;
        return;
    }

    if (tuning.d_ref_um == 0)
    {
      out_confidence_slip = tuning.confidence_rotation_all_ok;
      out_confidence_translation = tuning.confidence_translation_all_ok;
      out_confidence_rotation = tuning.confidence_rotation_all_ok;
      return;
    }
    const std::int64_t d_l = std::abs(delta_fl_um - delta_rl_um);
    const std::int64_t d_r = std::abs(delta_fr_um - delta_rr_um);
    
    const double inv_d_ref = 1.0 / tuning.d_ref_um;
    const double ratio_l = d_l * inv_d_ref;
    const double ratio_r = d_r * inv_d_ref;

    const double c_l = 1.0 / (1.0 + ratio_l * ratio_l);
    const double c_r = 1.0 / (1.0 + ratio_r * ratio_r);

    out_confidence_slip = static_cast<std::int64_t>((c_l * c_r) * tuning.confidence_rotation_all_ok);
    out_confidence_translation = static_cast<std::int64_t>(std::sqrt(c_l * c_r) * tuning.confidence_translation_all_ok);
    out_confidence_rotation = static_cast<std::int64_t>(std::fmin(c_l, c_r) * tuning.confidence_rotation_all_ok);
  }
}

namespace motion_model_encoders
{
  void reset(motion_model_snapshot &state)
  {
    state = {};
  }

  bool estimate_from_encoder(const delta_estimation_encoders::delta_snapshot &delta_snapshot, const confidence_estimation_encoders::confidence_snapshot &confidence_snapshot, motion_model_snapshot &out)
  {
    out = {};

    if (!delta_snapshot.has_delta)
    {
      return false;
    }

    const std::int64_t delta_fl_um = delta_snapshot.wheel_delta_motion[0u].wheel_delta_distance_um;
    const std::int64_t delta_fr_um = delta_snapshot.wheel_delta_motion[1u].wheel_delta_distance_um;
    const std::int64_t delta_rl_um = delta_snapshot.wheel_delta_motion[2u].wheel_delta_distance_um;
    const std::int64_t delta_rr_um = delta_snapshot.wheel_delta_motion[3u].wheel_delta_distance_um;
    compute_confidences(confidence_snapshot, delta_fl_um, delta_fr_um, delta_rl_um, delta_rr_um, out.confidence_slip, out.confidence_translation, out.confidence_rotation);

    if (!confidence_snapshot.can_estimate_translation)
    {
      return false;
    }

    const std::int64_t wheel_separation_um = static_cast<std::int64_t>(drivetrain.wheel_separation_mm) * tuning.um_per_mm;

    
    // motion model math
    std::int64_t numerator = tuning.weight_fl*delta_fl_um + tuning.weight_rl*delta_rl_um;
    std::int64_t denominator  = tuning.weight_fl*confidence_snapshot.meas_enable_fl + tuning.weight_rl*confidence_snapshot.meas_enable_rl;

    bool has_left_delta = false;
    std::int64_t delta_left = 0;
    if (denominator != 0u)
    {
      delta_left = numerator / denominator;
      has_left_delta = true;
    }


    numerator = tuning.weight_fr*delta_fr_um + tuning.weight_rr*delta_rr_um;
    denominator  = tuning.weight_fr*confidence_snapshot.meas_enable_fr + tuning.weight_rr*confidence_snapshot.meas_enable_rr;

    bool has_right_delta = false;
    std::int64_t delta_right = 0;
    if(denominator != 0u)
    {
      delta_right = numerator/denominator;
      has_right_delta = true;
    }

    out.has_left_delta = has_left_delta;
    out.has_right_delta = has_right_delta;
    out.left_delta = delta_left;
    out.right_delta = delta_right;
    out.rotate_in_spot_flag = has_left_delta && has_right_delta && compute_rotate_in_spot_flag(delta_left, delta_right);
    
    if (has_left_delta && has_right_delta)
    {
      out.translation = (delta_left + delta_right) / 2;
    }
    else if (has_left_delta)
    {
      out.translation = delta_left;
    }
    else if (has_right_delta)
    {
      out.translation = delta_right;
    }
    else
    {
      return false;
    }


    if (!confidence_snapshot.can_estimate_rotation)
    {
      out.rotation = 0;
      out.confidence_rotation = 0;
      out.has_motion_model = true;
      return true;
    }
    if (wheel_separation_um == 0u)
    {
      return false;
    }

    

    out.rotation = ((delta_right - delta_left) * 1000000LL) / wheel_separation_um;
    out.has_motion_model = true;
    return true;
  }
}
