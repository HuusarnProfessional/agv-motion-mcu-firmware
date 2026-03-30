/*
IMU_CALIBRATION - PLAN (engangsprocedur)

1) imu_calibration ska ta fram tare och transform fran IMU-referens till AGV-referens.

2) Resultatet ska skrivas till imu_api sa att senare imu-samples kan lasas i AGV-referens.

3) Flode:
   - Stoppa motorer.
   - Verifiera med encoder_model att AGV star still.
   - Sampla manga IMU-samples och bygg tare som medelvarde.
   - Kor framat och samla IMU-data i block.
   - Stoppa framat nar blockmedel for acceleration faller under 25% av peak, eller nar encoder nar 500 mm.
   - Stoppa och lat AGV bli still igen.
   - Kor bakat och samla blockmedel pa samma satt.
   - Anvand still + framat + bakat for att losa AGV-plan och riktningsmatris.
   - Skriv tare + matris till imu_api.

4) Pipeline-filen ska bara visa ordningen.
   Hjalpfilerna ska bara gora deljobben.
*/

#include "core/control/imu_calibration/imu_calibration_pipeline.hpp"
#include "core/control/imu_calibration/imu_calibration_drive_and_sample_alignment.hpp"
#include "core/control/imu_calibration/imu_calibration_solve_and_set.hpp"
#include "core/control/imu_calibration/imu_calibration_stop_and_sample_tare.hpp"

namespace imu_calibration
{
  void reset(state &imu_calibration_state)
  {
    imu_calibration_state = {};
  }

  void start(state &imu_calibration_state)
  {
    reset(imu_calibration_state);
    imu_calibration_state.in_progress = true;
    imu_calibration_state.current_step = step::stop_before_tare;
  }

  bool tick(state &imu_calibration_state, const encoder_motion::state &encoder_model_state, std::uint8_t imu_id)
  {
    if (imu_calibration_state.current_step == step::done)
    {
      return true;
    }

    if (!imu_calibration_state.in_progress)
    {
      return false;
    }

    switch (imu_calibration_state.current_step)
    {
      case step::stop_before_tare:
      {
        if (!imu_calibration_state.stop_before_tare_command_sent)
        {
          stop_motor();
          imu_calibration_state.stop_before_tare_command_sent = true;
        }

        if (is_still_from_encoder_model(encoder_model_state))
        {
          imu_calibration_state.current_step = step::build_tare;
        }
        break;
      }

      case step::build_tare:
      {
        if (tick_build_tare_values(imu_calibration_state.tare_step_state, imu_id, imu_calibration_state.tare_values, imu_calibration_state.noise_profile))
        {
          imu_calibration_state.current_step = step::drive_forward;
        }
        break;
      }

      case step::drive_forward:
      {
        if (tick_drive_forward_and_sample(imu_calibration_state.forward_step_state, encoder_model_state, imu_id, imu_calibration_state.tare_values, imu_calibration_state.forward_values))
        {
          imu_calibration_state.current_step = step::stop_before_backward;
        }
        break;
      }

      case step::stop_before_backward:
      {
        if (!imu_calibration_state.stop_before_backward_command_sent)
        {
          stop_motor();
          imu_calibration_state.stop_before_backward_command_sent = true;
        }

        if (is_still_from_encoder_model(encoder_model_state))
        {
          imu_calibration_state.current_step = step::drive_backward;
        }
        break;
      }

      case step::drive_backward:
      {
        if (tick_drive_backward_and_sample(imu_calibration_state.backward_step_state, encoder_model_state, imu_id, imu_calibration_state.tare_values, imu_calibration_state.backward_values))
        {
          imu_calibration_state.current_step = step::solve_and_set;
        }
        break;
      }

      case step::solve_and_set:
      {
        build_mean_values_from_drive_samples(imu_calibration_state.forward_values, imu_calibration_state.forward_mean_values);
        build_mean_values_from_drive_samples(imu_calibration_state.backward_values, imu_calibration_state.backward_mean_values);

        if (imu_calibration_state.tare_values.has_tare &&
            imu_calibration_state.forward_mean_values.has_mean &&
            imu_calibration_state.backward_mean_values.has_mean)
        {
          solve_alignment_matrix(imu_calibration_state.forward_mean_values, imu_calibration_state.backward_mean_values, imu_calibration_state.tare_values);
          set_calibration_profile_to_imu(imu_id, imu_calibration_state.tare_values, imu_calibration_state.noise_profile);
        }

        imu_calibration_state.current_step = step::done;
        imu_calibration_state.in_progress = false;
        return imu_calibration_state.tare_values.has_tare &&
               imu_calibration_state.forward_mean_values.has_mean &&
               imu_calibration_state.backward_mean_values.has_mean;
      }

      case step::done:
      {
        return true;
      }

      case step::idle:
      default:
      {
        break;
      }
    }

    return false;
  }
}
