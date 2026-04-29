cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32g474re_agv)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)
enable_language(CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../app/app_entry.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/motors_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/encoders_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/imu_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/voltage_monitor_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/obstacle_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/comm_uart_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/led_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32g474re_agv/subsystems/user_button_board_stm32g474re_agv.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/comm_uart_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/button_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/led_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/encoder_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/imu_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/motor_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/obstacle_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/voltage_monitor_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/middleware_runtime.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/incoming_middleware_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/outgoing_middleware_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/runtime/local_position_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/runtime/safety_status_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/runtime/power_status_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/debug/encoder_debug_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/debug/imu_debug_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/debug/obstacle_debug_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/outgoing_payloads/debug/voltage_debug_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/runtime/motion_command_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/service/start_imu_calibration_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/service/clear_imu_calibration_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/debug/debug_stream_control_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/service/trailer_status_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/service/unlock_safe_guard_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/service/lock_safe_guard_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/middleware/incoming_payloads/service/obstacle_safety_control_payload.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/collision_prediction/collision_prediction_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/collision_prediction/collision_prediction_logic.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/collision_prediction/internal/vehicle_motion_estimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/collision_prediction/internal/collision_input_builder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/collision_prediction/internal/obstacle_sensor_logic.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/drive_control/wheel_drive_controller.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/drive_control/drive_control_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/imu_calibration/imu_calibration_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/imu_calibration/imu_calibration_drive_and_sample_alignment.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/imu_calibration/imu_calibration_solve_and_set.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/imu_calibration/imu_calibration_stop_and_sample_tare.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/encoder_model/encoder_model_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/encoder_model/confidence_estimation_encoders.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/encoder_model/delta_estimation_encoders.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/encoder_model/input_storage_encoders.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/encoder_model/motion_model_encoders.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/imu_model/delta_estimation_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/imu_model/input_storage_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/imu_model/stationary_detection_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/imu_model/motion_model_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/imu_model/imu_model_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/sensorfusion_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/local_positioning.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/local_positioning_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/rotation/imu/sensorfusion_rotation_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/translation/sensorfusion_translation.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/translation/encoder/sensorfusion_translation_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/translation/imu/sensorfusion_translation_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/safe_guard/safe_guard_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/robot_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/comm_uart_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/button_gpio_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/led_gpio_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/encoder_as5600_pwm_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/imu_lsm9ds1_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/voltage_monitor_adc_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/motor_drv8871_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/obstacle_hcsr04_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../libraries/lsm9ds1_driver/lsm9ds1_reg.c
    ${CMAKE_CURRENT_LIST_DIR}/../../core/state/agv_state.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/system_select/system_select.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/main_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/motor_pwm_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/encoder_capture_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/imu_spi_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/voltage_adc_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/obstacle_hcsr04_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/button_gpio_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/led_gpio_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/comm_uart_platform_stm32.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../..
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
