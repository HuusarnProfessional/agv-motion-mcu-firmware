#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include "core/control/local_positioning/imu_model/imu_model_pipeline.hpp"
#include "core/control/local_positioning/local_positioning.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_pipeline.hpp"
#include "core/control/local_positioning/test/local_position_model_debug.hpp"

namespace local_positioning_pipeline
{
  void init(void);
  void tick(std::uint32_t now_ms);
  bool request_position_correction(const local_positioning::external_correction_request &request);
  void read_snapshot(local_positioning::snapshot &out);
  void read_model_debug_snapshot(local_position_model_debug::snapshot &out);
  void read_encoder_motion_state(encoder_motion::state &out);
  void read_imu_state(local_positioning_imu::state &out);
  std::uint8_t read_imu_id(void);
}
