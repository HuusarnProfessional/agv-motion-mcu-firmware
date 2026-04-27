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

  void init(void);
  void start(void);
  bool tick(const encoder_motion::state &encoder_model_state, std::uint8_t imu_id);
  void request_start(void);
  void request_clear(void);
  bool consume_start_request(void);
  bool consume_clear_request(void);
}
