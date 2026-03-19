# AGV Firmware Structure

## Goal
Core logic (state + control rules) should be reusable across multiple targets.
Board-specific and platform-specific dependencies are kept in separate layers.

## Folders
- `core/`: shared C++ code
  - `api/`: stable interfaces for hardware-facing modules
  - `impl/`: implementations behind the APIs
  - `control/`: control and motion logic
  - `state/`: state handling and flow
  - `system_select/`: compile-time implementation selection
- `platform/`: wraps vendor/HAL access per MCU family
- `board/`: board mapping (pinout, timers, buses)
- `targets/`: generated board projects (CubeMX, Arduino, etc.)
- `app/`: common entry point called by target main

## Principles
- `core` must not include HAL headers.
- `impl` should only use generic wrappers exposed from `platform`.
- `board` defines resources and mapping, not control logic.
- `system_select` decides which implementation is used (for example DRV8871 vs custom H-bridge).
