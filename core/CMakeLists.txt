add_library(agv_core STATIC
  api/comm_uart_api.cpp
  api/motor_api.cpp
  api/obstacle_api.cpp
  api/voltage_monitor_api.cpp
  control/controller_esp32_wroom32_demo.cpp
  control/local_positioning/encoder_model/encoder_model_pipeline.cpp
  control/local_positioning/encoder_model/confidence_estimation_encoders.cpp
  control/local_positioning/encoder_model/delta_estimation_encoders.cpp
  control/local_positioning/encoder_model/input_storage_encoders.cpp
  control/local_positioning/encoder_model/motion_model_encoders.cpp
  control/local_positioning/imu_model/delta_estimation_imu.cpp
  control/local_positioning/imu_model/motion_model_imu.cpp
  control/imu_calibration/imu_calibration_pipeline.cpp
  control/imu_calibration/imu_calibration_drive_and_sample_alignment.cpp
  control/imu_calibration/imu_calibration_solve_and_set.cpp
  control/imu_calibration/imu_calibration_stop_and_sample_tare.cpp
  control/controller_robot_future.cpp
  control/robot_control.cpp
  impl/comm_uart_impl.cpp
  impl/motor_drv8871_impl.cpp
  impl/obstacle_hcsr04_impl.cpp
  impl/voltage_monitor_adc_impl.cpp
  ../libraries/lsm9ds1_driver/lsm9ds1_reg.c
  state/agv_state.cpp
)

target_include_directories(agv_core PUBLIC
  ${PROJECT_SOURCE_DIR}
)

set_target_properties(agv_core PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)
