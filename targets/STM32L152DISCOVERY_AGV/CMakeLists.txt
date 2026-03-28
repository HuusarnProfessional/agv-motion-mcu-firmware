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
set(CMAKE_PROJECT_NAME STM32L152DISCOVERY_AGV)

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
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/main_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/motors_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/encoders_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/imu_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/voltage_monitor_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/obstacle_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32l152discovery/comm_uart_board_stm32l152discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/comm_uart_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/encoder_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/imu_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/motor_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/obstacle_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/voltage_monitor_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/controller_esp32_wroom32_demo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/controller_robot_future.cpp
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
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/rotation/sensorfusion_rotation.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/rotation/encoder/sensorfusion_rotation_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/sensorfusion/rotation/imu/sensorfusion_rotation_imu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/robot_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/comm_uart_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/encoder_as5600_pwm_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/imu_lsm9ds1_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/voltage_monitor_adc_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/motor_drv8871_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/obstacle_hcsr04_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../libraries/lsm9ds1_driver/lsm9ds1_reg.c
    ${CMAKE_CURRENT_LIST_DIR}/../../core/state/agv_state.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/system_select/system_select.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/motor_pwm_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/encoder_capture_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/imu_spi_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/voltage_adc_platform_stm32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/stm32/obstacle_hcsr04_platform_stm32.cpp
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
