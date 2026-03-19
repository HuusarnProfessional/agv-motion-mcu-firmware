add_library(agv_core STATIC
  api/comm_api_ai.cpp
  api/middleware_api_ai.cpp
  api/motor_api.cpp
  api/obstacle_api.cpp
  api/voltage_monitor_api.cpp
  control/controller_esp32_wroom32_demo.cpp
  control/robot_control.cpp
  control/robot_control_ai.cpp
  impl/comm_uart_impl_ai.cpp
  impl/middleware_impl_ai.cpp
  impl/motor_drv8871_impl.cpp
  impl/obstacle_hcsr04_impl.cpp
  impl/voltage_monitor_adc_impl.cpp
  ../libraries/lsm9ds1_driver/lsm9ds1_reg.c
  state/agv_state.cpp
)

# Inkludera repo-root sa att #include "core/..." funkar
target_include_directories(agv_core PUBLIC
  ${PROJECT_SOURCE_DIR}
)

# Behall C++-kompilering for hela core
set_target_properties(agv_core PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)
