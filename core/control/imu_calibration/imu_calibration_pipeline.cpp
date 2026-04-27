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
    bool success = false;
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

  bool finish_pipeline(bool success)
  {
    stop_motor();
    g_pipeline_state.success = success;
    g_pipeline_state.current_step = imu_calibration::step::done;
    g_pipeline_state.in_progress = false;
    return success;
  }

  bool tick_stop_before_tare(const encoder_motion::state &encoder_model_state)
  {
    if (!g_pipeline_state.stop_before_tare_command_sent)
    {
      stop_motor();
      g_pipeline_state.stop_before_tare_command_sent = true;
    }

    if (is_still_from_encoder_model(encoder_model_state))
    {
      g_pipeline_state.current_step = imu_calibration::step::build_tare;
    }

    return false;
  }

  bool tick_build_tare(std::uint8_t imu_id)
  {
    if (!tick_build_tare_values(g_pipeline_state.tare_step_state, imu_id, g_pipeline_state.tare_values, g_pipeline_state.noise_profile))
    {
      return false;
    }

    if (g_pipeline_state.tare_step_state.failed)
    {
      return finish_pipeline(false);
    }

    g_pipeline_state.current_step = imu_calibration::step::drive_forward;
    return false;
  }

  bool tick_drive_forward(const encoder_motion::state &encoder_model_state, std::uint8_t imu_id)
  {
    if (!tick_drive_forward_and_sample(g_pipeline_state.forward_step_state, encoder_model_state, imu_id, g_pipeline_state.tare_values, g_pipeline_state.forward_values))
    {
      return false;
    }

    if (g_pipeline_state.forward_step_state.failed)
    {
      return finish_pipeline(false);
    }

    g_pipeline_state.current_step = imu_calibration::step::stop_before_backward;
    return false;
  }

  bool tick_stop_before_backward(const encoder_motion::state &encoder_model_state)
  {
    if (!g_pipeline_state.stop_before_backward_command_sent)
    {
      stop_motor();
      g_pipeline_state.stop_before_backward_command_sent = true;
    }

    if (is_still_from_encoder_model(encoder_model_state))
    {
      g_pipeline_state.current_step = imu_calibration::step::drive_backward;
    }

    return false;
  }

  bool tick_drive_backward(const encoder_motion::state &encoder_model_state, std::uint8_t imu_id)
  {
    if (!tick_drive_backward_and_sample(g_pipeline_state.backward_step_state, encoder_model_state, imu_id, g_pipeline_state.tare_values, g_pipeline_state.backward_values))
    {
      return false;
    }

    if (g_pipeline_state.backward_step_state.failed)
    {
      return finish_pipeline(false);
    }

    g_pipeline_state.current_step = imu_calibration::step::solve_and_set;
    return false;
  }

  bool tick_solve_and_set(std::uint8_t imu_id)
  {
    build_mean_values_from_drive_samples(g_pipeline_state.forward_values, g_pipeline_state.forward_mean_values);
    build_mean_values_from_drive_samples(g_pipeline_state.backward_values, g_pipeline_state.backward_mean_values);

    if (!g_pipeline_state.tare_values.has_tare)
    {
      return finish_pipeline(false);
    }

    if (!g_pipeline_state.forward_mean_values.has_mean)
    {
      return finish_pipeline(false);
    }

    if (!g_pipeline_state.backward_mean_values.has_mean)
    {
      return finish_pipeline(false);
    }

    if (!solve_alignment_matrix(g_pipeline_state.forward_mean_values, g_pipeline_state.backward_mean_values, g_pipeline_state.tare_values))
    {
      return finish_pipeline(false);
    }

    if (!set_calibration_profile_to_imu(imu_id, g_pipeline_state.tare_values, g_pipeline_state.noise_profile))
    {
      return finish_pipeline(false);
    }

    return finish_pipeline(true);
  }

  void reset_pipeline_state(void)
  {
    g_pipeline_state = {};
  }
}

namespace imu_calibration
{
  void init(void)
  {
    reset_pipeline_state();
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

  bool is_in_progress(void)
  {
    if (g_pipeline_state.current_step == step::idle)
    {
      return false;
    }

    if (g_pipeline_state.current_step == step::done)
    {
      return false;
    }

    return true;
  }

  void clear_pipeline_state(void)
  {
    reset_pipeline_state();
  }

  bool tick(const encoder_motion::state &encoder_model_state, std::uint8_t imu_id)
  {
    if (g_pipeline_state.current_step == step::done)
    {
      return g_pipeline_state.success;
    }

    if (!g_pipeline_state.in_progress)
    {
      return false;
    }

    switch (g_pipeline_state.current_step)
    {
      case step::stop_before_tare:
      {
        return tick_stop_before_tare(encoder_model_state);
      }
      case step::build_tare:
      {
        return tick_build_tare(imu_id);
      }
      case step::drive_forward:
      {
        return tick_drive_forward(encoder_model_state, imu_id);
      }
      case step::stop_before_backward:
      {
        return tick_stop_before_backward(encoder_model_state);
      }
      case step::drive_backward:
      {
        return tick_drive_backward(encoder_model_state, imu_id);
      }
      case step::solve_and_set:
      {
        return tick_solve_and_set(imu_id);
      }
      case step::done:
      {
        return g_pipeline_state.success;
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
