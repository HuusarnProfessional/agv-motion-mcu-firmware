#pragma once

#include <cstdint>

#include "core/api/imu_api.hpp"
#include "core/control/imu_calibration/imu_calibration_drive_and_sample_alignment.hpp"
#include "core/control/imu_calibration/imu_calibration_solve_and_set.hpp"
#include "core/control/imu_calibration/imu_calibration_stop_and_sample_tare.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"

namespace imu_calibration
{
  enum class step : std::uint8_t
  {
    idle = 0,
    stop_before_tare,
    build_tare,
    drive_forward,
    stop_before_backward,
    drive_backward,
    solve_and_set,
    done
  };

  struct state
  {
    bool in_progress = false;
    step current_step = step::idle;
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

  void reset(state &imu_calibration_state);
  void start(state &imu_calibration_state);
  bool tick(state &imu_calibration_state, const encoder_motion::state &encoder_model_state, std::uint8_t imu_id);
}
