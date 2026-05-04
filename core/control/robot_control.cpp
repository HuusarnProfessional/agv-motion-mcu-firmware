#include "core/control/robot_control.hpp"

#include "core/api/imu_api.hpp"
#include "core/control/collision_prediction/collision_prediction_pipeline.hpp"
#include "core/control/drive_control/drive_control_pipeline.hpp"
#include "core/control/imu_calibration/imu_calibration_pipeline.hpp"
#include "core/control/heartbeat/heartbeat_pipeline.hpp"
#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/control/motion_primitives/motion_primitives_pipeline.hpp"
#include "core/control/safe_guard/safe_guard_pipeline.hpp"
#include "core/middleware/incoming_payloads/incoming_middleware_pipeline.hpp"
#include "core/middleware/outgoing_payloads/outgoing_middleware_pipeline.hpp"

namespace robot_control
{
  namespace
  {
    constexpr std::uint8_t k_comm_uart_id = 0u;

    enum class active_controller : std::uint8_t
    {
      safe_guard = 0u,
      imu_calibration,
      drive
    };

    active_controller select_active_controller(void)
    {
      if (safe_guard::is_latched())
      {
        return active_controller::safe_guard;
      }

      if (imu_calibration::is_in_progress())
      {
        return active_controller::imu_calibration;
      }

      return active_controller::drive;
    }
  }

  void init(void)
  {
    incoming_middleware_pipeline::init(k_comm_uart_id);
    outgoing_middleware_pipeline::init();
    local_positioning_pipeline::init();
    collision_prediction::init();
    heartbeat::init();
    safe_guard::init();
    drive_control::init();
    imu_calibration::init();
    motion_primitives_pipeline::init();
  }

  void tick(std::uint32_t now_ms)
  {
    encoder_motion::state encoder_motion_state = {};
    collision_prediction::snapshot collision_prediction_snapshot = {};
    const std::uint8_t imu_id = local_positioning_pipeline::read_imu_id();
    incoming_middleware_pipeline::tick(now_ms);

    if (imu_calibration::consume_clear_request())
    {
      imu_api::clear_calibration(imu_id);
      imu_calibration::clear_pipeline_state();
    }

    if (imu_calibration::consume_start_request())
    {
      imu_calibration::start();
    }

    local_positioning_pipeline::tick(now_ms);
    local_positioning_pipeline::read_encoder_motion_state(encoder_motion_state);
    collision_prediction::tick(now_ms);
    heartbeat::tick(now_ms);
    collision_prediction::read_snapshot(collision_prediction_snapshot);

    if (collision_prediction_snapshot.collision_blocked)
    {
      safe_guard::trip();
    }

    if (heartbeat::is_timed_out())
    {
      safe_guard::trip();
    }

    safe_guard::tick(now_ms);
    const active_controller controller = select_active_controller();

    if (controller == active_controller::imu_calibration)
    {
      motion_primitives_pipeline::stop(now_ms);
      imu_calibration::tick(encoder_motion_state, imu_id);
    }
    else if (controller == active_controller::drive)
    {
      motion_primitives_pipeline::tick(now_ms);
      drive_control::tick(now_ms);
    }
    else
    {
      motion_primitives_pipeline::stop(now_ms);
    }

    outgoing_middleware_pipeline::tick(now_ms);
  }

  void apply_comm_drive_defaults(const comm_drive_defaults &defaults)
  {
    (void)defaults;
  }
}
