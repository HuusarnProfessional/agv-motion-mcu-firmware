#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/motion_model_encoders.hpp"
#include "core/control/local_positioning/imu_model/motion_model_imu.hpp"
#include "core/control/local_positioning/local_positioning.hpp"
#include "core/control/local_positioning/sensorfusion/sensorfusion_pipeline.hpp"

namespace local_position_model_debug
{
  struct snapshot
  {
    bool has_encoder_motion = false;
    bool has_imu_motion = false;
    bool has_fused_translation = false;
    bool has_fused_rotation = false;
    std::int64_t encoder_translation_um = 0;
    std::int64_t encoder_rotation_urad = 0;
    std::int64_t encoder_translation_sum_um = 0;
    std::int64_t encoder_rotation_sum_urad = 0;
    std::int64_t imu_translation_um = 0;
    std::int64_t imu_rotation_urad = 0;
    std::int64_t imu_translation_sum_um = 0;
    std::int64_t imu_rotation_sum_urad = 0;
    std::int64_t fused_translation_um = 0;
    std::int64_t fused_rotation_urad = 0;
    std::int64_t fused_translation_sum_um = 0;
    std::int64_t fused_rotation_sum_urad = 0;
    std::int64_t local_x_um = 0;
    std::int64_t local_y_um = 0;
    std::int32_t local_heading_urad = 0;
    std::uint32_t local_update_id = 0u;
    std::uint32_t time_ms = 0u;
  };

  struct state
  {
    snapshot output_snapshot = {};
  };

  void reset(state &debug_state);
  void tick(state &debug_state,
            const motion_model_encoders::motion_model_snapshot &encoder_motion_snapshot,
            const motion_model_imu::motion_model_snapshot &imu_motion_snapshot,
            const local_positioning_sensorfusion::state &sensorfusion_state,
            const local_positioning::state &local_positioning_state,
            std::uint32_t now_ms);
}
