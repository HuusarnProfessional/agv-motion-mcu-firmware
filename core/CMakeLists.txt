add_library(agv_core STATIC
  api/comm_uart_api.cpp
  api/motor_api.cpp
  api/obstacle_api.cpp
  api/voltage_monitor_api.cpp
  middleware/middleware_shared_state.cpp
  middleware/incoming_middleware_pipeline.cpp
  middleware/outgoing_middleware_pipeline.cpp
  middleware/outgoing_payloads/local_position_payload.cpp
  middleware/outgoing_payloads/safety_status_payload.cpp
  middleware/outgoing_payloads/power_status_payload.cpp
  middleware/outgoing_payloads/encoder_debug_payload.cpp
  middleware/outgoing_payloads/imu_debug_payload.cpp
  middleware/outgoing_payloads/obstacle_debug_payload.cpp
  middleware/outgoing_payloads/voltage_debug_payload.cpp
  middleware/incoming_payloads/motion_command_payload.cpp
  middleware/incoming_payloads/start_imu_calibration_payload.cpp
  middleware/incoming_payloads/clear_imu_calibration_payload.cpp
  middleware/incoming_payloads/debug_stream_control_payload.cpp
  middleware/incoming_payloads/obstacle_safety_control_payload.cpp
  control/local_positioning/encoder_model/encoder_model_pipeline.cpp
  control/local_positioning/encoder_model/confidence_estimation_encoders.cpp
  control/local_positioning/encoder_model/delta_estimation_encoders.cpp
  control/local_positioning/encoder_model/input_storage_encoders.cpp
  control/local_positioning/encoder_model/motion_model_encoders.cpp
  control/local_positioning/imu_model/delta_estimation_imu.cpp
  control/local_positioning/imu_model/input_storage_imu.cpp
  control/local_positioning/imu_model/stationary_detection_imu.cpp
  control/local_positioning/imu_model/motion_model_imu.cpp
  control/local_positioning/imu_model/imu_model_pipeline.cpp
  control/local_positioning/sensorfusion/sensorfusion_pipeline.cpp
  control/local_positioning/local_positioning.cpp
  control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.cpp
  control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.cpp
  control/local_positioning/sensorfusion/rotation/imu/sensorfusion_rotation_imu.cpp
  control/local_positioning/sensorfusion/translation/sensorfusion_translation.cpp
  control/local_positioning/sensorfusion/translation/encoder/sensorfusion_translation_encoder.cpp
  control/local_positioning/sensorfusion/translation/imu/sensorfusion_translation_imu.cpp
  control/imu_calibration/imu_calibration_pipeline.cpp
  control/imu_calibration/imu_calibration_drive_and_sample_alignment.cpp
  control/imu_calibration/imu_calibration_solve_and_set.cpp
  control/imu_calibration/imu_calibration_stop_and_sample_tare.cpp
  control/collision_prediction/collision_prediction_pipeline.cpp
  control/drive_control/drive_control_pipeline.cpp
  control/robot_control.cpp
  control/local_positioning/local_positioning_pipeline.cpp
  impl/comm_uart_impl.cpp
  impl/motor_drv8871_impl.cpp
  impl/obstacle_hcsr04_impl.cpp
  impl/voltage_monitor_adc_impl.cpp
  state/agv_state.cpp
)

target_include_directories(agv_core PUBLIC
  ${PROJECT_SOURCE_DIR}
)

set_target_properties(agv_core PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)
