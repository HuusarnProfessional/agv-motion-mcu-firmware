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

namespace
{
  struct pipeline_state
  {
    bool in_progress = false;
    bool start_requested = false;
    bool clear_requested = false;
    imu_calibration::step current_step = imu_calibration::step::idle;
    bool stop_before_tare_command_sent = false;
    bool stop_before_backward_command_sent = false;
    imu_tare_step_state tare_step_state = {};
    imu_drive_sample_step_state forward_step_state = {};
    imu_drive_sample_step_state backward_step_state = {};
    imu_api::imu_tare_values tare_values = {};
    imu_api::imu_noise_profile noise_profile = {};
    imu_drive_sample_values forward_values = {};
    imu_drive_sample_values backward_values = {};
    imu_drive_sample_mean_values forward_mean_values = {};
    imu_drive_sample_mean_values backward_mean_values = {};
  };

  pipeline_state g_pipeline_state = {};
}

namespace imu_calibration
{
  void init(void)
  {
    g_pipeline_state = {};
  }

  void start(void)
  {
    init();
    g_pipeline_state.in_progress = true;
    g_pipeline_state.current_step = step::stop_before_tare;
  }

  void request_start(void)
  {
    g_pipeline_state.start_requested = true;
  }

  void request_clear(void)
  {
    g_pipeline_state.clear_requested = true;
  }

  bool consume_start_request(void)
  {
    const bool has_request = g_pipeline_state.start_requested;
    g_pipeline_state.start_requested = false;
    return has_request;
  }

  bool consume_clear_request(void)
  {
    const bool has_request = g_pipeline_state.clear_requested;
    g_pipeline_state.clear_requested = false;
    return has_request;
  }

  bool tick(const encoder_motion::state &encoder_model_state, std::uint8_t imu_id)
  {
    if (g_pipeline_state.current_step == step::done)
    {
      return true;
    }

    if (!g_pipeline_state.in_progress)
    {
      return false;
    }

    switch (g_pipeline_state.current_step)
    {
      case step::stop_before_tare:
      {
        if (!g_pipeline_state.stop_before_tare_command_sent)
        {
          stop_motor();
          g_pipeline_state.stop_before_tare_command_sent = true;
        }

        if (is_still_from_encoder_model(encoder_model_state))
        {
          g_pipeline_state.current_step = step::build_tare;
        }
        break;
      }

      case step::build_tare:
      {
        if (tick_build_tare_values(g_pipeline_state.tare_step_state, imu_id, g_pipeline_state.tare_values, g_pipeline_state.noise_profile))
        {
          g_pipeline_state.current_step = step::drive_forward;
        }
        break;
      }

      case step::drive_forward:
      {
        if (tick_drive_forward_and_sample(g_pipeline_state.forward_step_state, encoder_model_state, imu_id, g_pipeline_state.tare_values, g_pipeline_state.forward_values))
        {
          g_pipeline_state.current_step = step::stop_before_backward;
        }
        break;
      }

      case step::stop_before_backward:
      {
        if (!g_pipeline_state.stop_before_backward_command_sent)
        {
          stop_motor();
          g_pipeline_state.stop_before_backward_command_sent = true;
        }

        if (is_still_from_encoder_model(encoder_model_state))
        {
          g_pipeline_state.current_step = step::drive_backward;
        }
        break;
      }

      case step::drive_backward:
      {
        if (tick_drive_backward_and_sample(g_pipeline_state.backward_step_state, encoder_model_state, imu_id, g_pipeline_state.tare_values, g_pipeline_state.backward_values))
        {
          g_pipeline_state.current_step = step::solve_and_set;
        }
        break;
      }

      case step::solve_and_set:
      {
        build_mean_values_from_drive_samples(g_pipeline_state.forward_values, g_pipeline_state.forward_mean_values);
        build_mean_values_from_drive_samples(g_pipeline_state.backward_values, g_pipeline_state.backward_mean_values);

        if (g_pipeline_state.tare_values.has_tare &&
            g_pipeline_state.forward_mean_values.has_mean &&
            g_pipeline_state.backward_mean_values.has_mean)
        {
          solve_alignment_matrix(g_pipeline_state.forward_mean_values, g_pipeline_state.backward_mean_values, g_pipeline_state.tare_values);
          set_calibration_profile_to_imu(imu_id, g_pipeline_state.tare_values, g_pipeline_state.noise_profile);
        }

        g_pipeline_state.current_step = step::done;
        g_pipeline_state.in_progress = false;
        return g_pipeline_state.tare_values.has_tare &&
               g_pipeline_state.forward_mean_values.has_mean &&
               g_pipeline_state.backward_mean_values.has_mean;
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
