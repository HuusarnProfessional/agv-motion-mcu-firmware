# Deferred Issues

Use this file for problems we discover now but intentionally postpone.

## How to write an entry
- `status`: `open` | `planned` | `done`
- `scope`: short area name (for example `local_positioning`, `state`, `board`)
- `problem`: what is wrong
- `risk`: why it matters
- `target`: what we want instead
- `refs`: file paths to current places

## Backlog

### ISSUE-001
- status: open
- scope: config/state
- problem: drivetrain mechanical parameters are defined in multiple places.
- risk: conflicting values can cause wrong motion estimates and inconsistent behavior.
- target: one single source of truth for mechanical parameters, and all modules read from that source.
- refs:
  - `core/state/agv_state.hpp`
  - `core/control/robot_control.hpp`
  - `core/mechanical_config/mechanical_config.hpp`
  - `app/app_entry.hpp`
  - `board/stm32f3discovery/board_stm32f3discovery.cpp`

### ISSUE-002
- status: open
- scope: imu/local_positioning
- problem: LSM9DS1 init sets full-scale values, but does not set an explicit output data rate for gyro or accelerometer.
- risk: local positioning tick rate may be chosen from assumptions instead of a known sensor rate, and sensor behavior may depend on reset defaults instead of firmware intent.
- target: set explicit LSM9DS1 output data rate in IMU init and align local positioning tick with that chosen rate.
- refs:
  - `core/impl/imu_lsm9ds1_impl.cpp`
  - `core/control/controller_robot_future.cpp`

