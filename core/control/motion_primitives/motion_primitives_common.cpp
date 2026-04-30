#include "core/control/motion_primitives/motion_primitives_common.hpp"

#include "core/control/drive_control/drive_control_pipeline.hpp"
#include "core/control/motion_primitives/motion_primitives_tuning.hpp"

namespace motion_primitives_common
{
  bool has_elapsed(std::uint32_t now_ms, std::uint32_t target_ms)
  {
    return static_cast<std::int32_t>(now_ms - target_ms) >= 0;
  }

  std::int64_t abs_i64(std::int64_t value)
  {
    if (value < 0)
    {
      return -value;
    }

    return value;
  }

  void send_motion_command(bool drive_enabled, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s, std::uint32_t now_ms)
  {
    middleware_incoming_payloads::motion_command_payload_data command = {};
    command.drive_enabled = drive_enabled;
    command.linear_velocity_mm_s = linear_velocity_mm_s;
    command.yaw_rate_mdeg_s = yaw_rate_mdeg_s;
    command.received_time_ms = now_ms;
    drive_control::set_motion_command(command);
  }

  void stop_motion(std::uint32_t now_ms)
  {
    send_motion_command(false, 0, 0, now_ms);
  }

  std::int64_t squared_distance_um(const local_positioning::snapshot &start_pose, const local_positioning::snapshot &current_pose)
  {
    const std::int64_t dx_um = current_pose.x_um - start_pose.x_um;
    const std::int64_t dy_um = current_pose.y_um - start_pose.y_um;
    return dx_um * dx_um + dy_um * dy_um;
  }

  bool has_reached_forward_target(const local_positioning::snapshot &start_pose, const local_positioning::snapshot &current_pose, std::int64_t target_distance_um)
  {
    const std::int64_t distance_squared_um2 = squared_distance_um(start_pose, current_pose);
    const std::int64_t target_squared_um2 = target_distance_um * target_distance_um;
    return distance_squared_um2 >= target_squared_um2;
  }

  bool has_reached_rotation_target(const local_positioning::snapshot &start_pose, const local_positioning::snapshot &current_pose, std::int64_t target_rotation_urad)
  {
    const std::int64_t heading_delta_urad = static_cast<std::int64_t>(current_pose.heading_urad) - static_cast<std::int64_t>(start_pose.heading_urad);
    return heading_delta_urad >= target_rotation_urad;
  }

  bool is_settled(const encoder_motion::state &encoder_state, const local_positioning_imu::state &imu_state)
  {
    const std::int64_t encoder_translation_abs = abs_i64(encoder_state.encoder_motion_snapshot.translation);
    const std::int64_t imu_translation_abs = abs_i64(imu_state.motion_snapshot.translation);
    const std::int64_t imu_yaw_abs = abs_i64(imu_state.delta_snapshot.delta_rotation_urad);

    return encoder_translation_abs <= motion_primitives_tuning::k_settling_encoder_translation_threshold_um &&
           imu_translation_abs <= motion_primitives_tuning::k_settling_imu_translation_threshold_um &&
           imu_yaw_abs <= motion_primitives_tuning::k_settling_imu_yaw_threshold_urad;
  }

  void begin_settling(state &primitive_state, const local_positioning::snapshot &current_pose, std::uint32_t now_ms)
  {
    stop_motion(now_ms);
    primitive_state.active_phase = phase::settling;
    primitive_state.snapshot.phase_start_pose = current_pose;
    primitive_state.snapshot.stop_time_ms = now_ms + motion_primitives_tuning::k_settling_max_duration_ms;
    primitive_state.snapshot.still_since_ms = 0u;
  }

  void finish(state &primitive_state, bool success, bool timed_out, std::uint32_t now_ms)
  {
    stop_motion(now_ms);
    primitive_state.snapshot.running = false;
    primitive_state.snapshot.complete = true;
    primitive_state.snapshot.success = success;
    primitive_state.snapshot.timed_out = timed_out;
    primitive_state.snapshot.end_time_ms = now_ms;
    primitive_state.active_phase = phase::done;
  }

  void begin_active_phase(state &primitive_state, const local_positioning::snapshot &current_pose)
  {
    primitive_state.active_phase = phase::active;
    primitive_state.snapshot.phase_start_pose = current_pose;
  }
}
