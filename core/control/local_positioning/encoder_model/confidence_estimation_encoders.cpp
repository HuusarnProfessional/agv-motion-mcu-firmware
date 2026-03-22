#include "core/control/local_positioning/encoder_model/confidence_estimation_encoders.hpp"

namespace confidence_estimation_encoders
{
  void reset(confidence_snapshot &state)
  {
    state = {};
  }

  bool estimate_from_delta_snapshot(const delta_estimation_encoders::delta_snapshot &delta_snapshot, confidence_snapshot &out)
  {
    out = {};

    if (!delta_snapshot.has_delta)
    {
      return false;
    }

    const delta_estimation_encoders::delta_status status_fl = delta_snapshot.wheel_deltas[0u].status;
    const delta_estimation_encoders::delta_status status_fr = delta_snapshot.wheel_deltas[1u].status;
    const delta_estimation_encoders::delta_status status_rl = delta_snapshot.wheel_deltas[2u].status;
    const delta_estimation_encoders::delta_status status_rr = delta_snapshot.wheel_deltas[3u].status;

    std::size_t first_bad_wheel_id = 0u;
    std::size_t second_bad_wheel_id = 0u;
    std::uint8_t bad_wheel_count = 0u;

    if (status_fl == delta_estimation_encoders::delta_status::bad)
    {
      out.meas_enable_fl = false;
      if (bad_wheel_count == 0u)
      {
        first_bad_wheel_id = 0u;
      }
      else if (bad_wheel_count == 1u)
      {
        second_bad_wheel_id = 0u;
      }

      ++bad_wheel_count;
    }
    else
    {
      out.meas_enable_fl = true;
    }

    if (status_fr == delta_estimation_encoders::delta_status::bad)
    {
      out.meas_enable_fr = false;
      if (bad_wheel_count == 0u)
      {
        first_bad_wheel_id = 1u;
      }
      else if (bad_wheel_count == 1u)
      {
        second_bad_wheel_id = 1u;
      }

      ++bad_wheel_count;
    }
    else
    {
      out.meas_enable_fr = true;
    }

    if (status_rl == delta_estimation_encoders::delta_status::bad)
    {
      out.meas_enable_rl = false;
      if (bad_wheel_count == 0u)
      {
        first_bad_wheel_id = 2u;
      }
      else if (bad_wheel_count == 1u)
      {
        second_bad_wheel_id = 2u;
      }

      ++bad_wheel_count;
    }
    else
    {
      out.meas_enable_rl = true;
    }

    if (status_rr == delta_estimation_encoders::delta_status::bad)
    {
      out.meas_enable_rr = false;
      if (bad_wheel_count == 0u)
      {
        first_bad_wheel_id = 3u;
      }
      else if (bad_wheel_count == 1u)
      {
        second_bad_wheel_id = 3u;
      }

      ++bad_wheel_count;
    }
    else
    {
      out.meas_enable_rr = true;
    }


    switch (bad_wheel_count)
    {
      case 0u:
        out.case_id = confidence_case::all_ok;
        out.can_estimate_translation = true;
        out.can_estimate_rotation = true;
        out.has_confidence = true;
        return true;

      case 1u:
        out.case_id = confidence_case::one_bad;
        out.can_estimate_translation = true;
        out.can_estimate_rotation = true;
        out.has_confidence = true;
        return true;

      case 2u:
      {
        const bool first_on_front_axle = first_bad_wheel_id == 0u || first_bad_wheel_id == 1u;
        const bool second_on_front_axle = second_bad_wheel_id == 0u || second_bad_wheel_id == 1u;
        const bool first_on_left_side = first_bad_wheel_id == 0u || first_bad_wheel_id == 2u;
        const bool second_on_left_side = second_bad_wheel_id == 0u || second_bad_wheel_id == 2u;

        if (first_on_front_axle == second_on_front_axle)
        {
          out.case_id = confidence_case::two_bad_same_axle;
          out.can_estimate_translation = true;
          out.can_estimate_rotation = true;
          out.has_confidence = true;
          return true;
        }

        if (first_on_left_side == second_on_left_side)
        {
          out.case_id = confidence_case::two_bad_same_side;
          out.can_estimate_translation = true;
          out.can_estimate_rotation = false;
          out.has_confidence = true;
          return true;
        }

        out.case_id = confidence_case::two_bad_mixed_axle;
        out.can_estimate_translation = true;
        out.can_estimate_rotation = true;
        out.has_confidence = true;
        return true;
      }

      case 3u:
        out.case_id = confidence_case::three_bad;
        out.can_estimate_translation = true;
        out.can_estimate_rotation = false;
        out.has_confidence = true;
        return true;

      default:
        out.case_id = confidence_case::four_bad;
        out.can_estimate_translation = false;
        out.can_estimate_rotation = false;
        out.has_confidence = true;
        return true;
    }
  }
}
