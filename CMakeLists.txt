cmake_minimum_required(VERSION 3.22)

project(agv_firmware LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Host build: endast for att kompilera och smoke-testa core-logik.
# For embedded targets genererar CubeMX egna CMake-projekt som sedan kan lanka mot agv_core.

add_subdirectory(core)

add_executable(host_smoke
  tests/host_smoke.cpp
)

target_link_libraries(host_smoke PRIVATE agv_core)
