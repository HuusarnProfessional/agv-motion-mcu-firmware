#include "core/control/robot_control.hpp"

#include "core/api/imu_api.hpp"
#include "core/control/collision_prediction/collision_prediction_pipeline.hpp"
#include "core/control/drive_control/drive_control_pipeline.hpp"
#include "core/control/imu_calibration/imu_calibration_pipeline.hpp"
#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/middleware/incoming_middleware_pipeline.hpp"
#include "core/middleware/outgoing_middleware_pipeline.hpp"

namespace robot_control
{
  namespace
  {
    constexpr std::uint8_t k_comm_uart_id = 0U;
  }

  void init(void)
  {
    incoming_middleware_pipeline::init(k_comm_uart_id);
    outgoing_middleware_pipeline::init();
    local_positioning_pipeline::init();
    collision_prediction::init();
    drive_control::init();
    imu_calibration::init();
  }

  void tick(std::uint32_t now_ms)
  {
    encoder_motion::state encoder_motion_state = {};
    incoming_middleware_pipeline::tick(now_ms);

    if (imu_calibration::consume_clear_request())
    {
      imu_api::clear_calibration(local_positioning_pipeline::read_imu_id());
      imu_calibration::init();
    }

    if (imu_calibration::consume_start_request())
    {
      imu_calibration::start();
    }

    local_positioning_pipeline::tick(now_ms);
    local_positioning_pipeline::read_encoder_motion_state(encoder_motion_state);
    imu_calibration::tick(encoder_motion_state, local_positioning_pipeline::read_imu_id());
    collision_prediction::tick(now_ms);
    drive_control::tick(now_ms);
    outgoing_middleware_pipeline::tick(now_ms);
  }

  void apply_comm_drive_defaults(const comm_drive_defaults &defaults)
  {
    (void)defaults;
  }
}
