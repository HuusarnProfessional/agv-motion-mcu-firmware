#pragma once

#include <cstdint>

#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include "core/control/local_positioning/imu_model/imu_model_pipeline.hpp"
#include "core/control/local_positioning/local_positioning.hpp"

namespace motion_primitives
{
  enum class primitive_id : std::uint8_t
  {
    none = 0u,
    pause,
    drive_forward,
    rotate_delta
  };

  struct pause_request
  {
    std::uint32_t duration_ms = 0u;
  };

  struct drive_forward_request
  {
    std::int16_t velocity_mm_s = 0;
    std::int64_t target_distance_um = 0;
  };

  struct rotate_delta_request
  {
    std::int16_t linear_velocity_mm_s = 0;
    std::int16_t yaw_rate_mdeg_s = 0;
    std::int64_t target_rotation_urad = 0;
    bool has_rotation_drive_tuning = false;
    std::int32_t rotation_min_drive_u = 0;
    std::int32_t rotation_startup_drive_u = 0;
  };

  struct request
  {
    primitive_id primitive_id_value = primitive_id::none;
    pause_request pause = {};
    drive_forward_request drive_forward = {};
    rotate_delta_request rotate_delta = {};
  };

  struct input_snapshot
  {
    local_positioning::snapshot pose = {};
    encoder_motion::state encoder_state = {};
    local_positioning_imu::state imu_state = {};
  };

  struct snapshot
  {
    bool running = false;
    bool complete = false;
    bool success = false;
    bool timed_out = false;
    primitive_id active_primitive_id = primitive_id::none;
    std::uint32_t start_time_ms = 0u;
    std::uint32_t end_time_ms = 0u;
    std::uint32_t stop_time_ms = 0u;
    std::uint32_t still_since_ms = 0u;
    local_positioning::snapshot start_pose = {};
    local_positioning::snapshot phase_start_pose = {};
  };

  void init(void);
  bool start(const request &primitive_request, std::uint32_t now_ms, const local_positioning::snapshot &current_pose);
  void tick(std::uint32_t now_ms, const input_snapshot &input);
  void stop(std::uint32_t now_ms);
  void read_snapshot(snapshot &out);
}
