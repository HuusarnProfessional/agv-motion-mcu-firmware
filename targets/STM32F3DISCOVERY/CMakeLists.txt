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
set(CMAKE_PROJECT_NAME STM32F3DISCOVERY)

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
    ${CMAKE_CURRENT_LIST_DIR}/../../board/stm32f3discovery/board_stm32f3discovery.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/comm_uart_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/encoder_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/api/motor_api.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/collision_prediction/collision_prediction_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/drive_control/drive_control_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/local_positioning/local_positioning_pipeline.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/control/robot_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/comm_uart_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/encoder_as5600_pwm_impl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../core/impl/motor_drv8871_impl.cpp
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

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
