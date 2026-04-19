# collision prediction and safe guard

## goal

The system is split into two parts:

- `collision_prediction`
- `safe_guard`

`collision_prediction` detects hazard.

`safe_guard` latches the fault and executes the safety action.

This keeps the hazard math separate from the system-level stop behavior.

---

## responsibility split

### collision_prediction

This module is responsible for:

- reading the data needed for collision evaluation
- estimating forward / backward motion from existing internal data
- checking which sensor is relevant for the current motion direction
- filtering obstacle distance
- deciding if collision risk exists

For now its public output is only:

```text
bool collision_blocked
```

That is enough for first implementation.

Later we may add debug data, but not now.

### safe_guard

This module is responsible for:

- latching a safety fault
- stopping the AGV
- blocking further drive commands while latched
- reading an already decoded unlock / reset flag
- setting LED or other simple safety indication
- exposing the current latched safety state

`safe_guard` does not decide whether hazard exists.

That decision happens before `safe_guard` is called.

In practice:

- if `collision_prediction` concludes hazard -> call `safe_guard`
- `safe_guard` then handles stop + latch + unlock flow

---

## data flow

The intended flow is:

1. middleware decodes latest motion command
2. middleware decodes unlock / reset flag
3. `collision_prediction::tick(now_ms)` runs
4. if `collision_prediction` says blocked -> `safe_guard::trip()`
5. `safe_guard::tick(now_ms)` runs
6. `drive_control` must respect `safe_guard` latched state
7. outgoing safety status should read from `safe_guard`

So:

- `collision_prediction` answers: "is there hazard now?"
- `safe_guard` answers: "is the system latched in safety stop?"

---

## collision prediction inputs

`collision_prediction` needs:

- obstacle samples
- latest decoded motion command
- existing local positioning data for motion estimation
- current time `now_ms`

It must stay inside `collision_prediction`.

`local_positioning` public output must stay unchanged.

So if extra motion values are needed, they must be derived locally inside `collision_prediction`.

---

## motion estimation

We should not rely only on measured motion.

We should not rely only on commanded motion.

We need both.

Use:

- measured forward motion
- commanded forward motion

Then derive a conservative effective motion.

Example:

```text
front_approach_mm_s = max(command_forward_mm_s, measured_forward_mm_s, 0)
rear_approach_mm_s  = max(-command_forward_mm_s, -measured_forward_mm_s, 0)
```

This gives two important properties:

- we can stop before the AGV has fully started moving
- we still stay protected if the AGV is still rolling when command is already zero

The measured speed should be derived from fused translation delta and real `dt_ms`.

No fixed loop time assumptions.

---

## sensor roles

For the first version we use explicit sensor roles by `sensor_id`.

For this target:

- `sensor_id 0 = front`
- `sensor_id 1 = rear`

This mapping should be treated as configuration, not scattered logic.

That means:

- `collision_prediction` may assume a fixed role mapping
- but the mapping must live in one place, for example tuning / config

Later this can be extended if more sensors are added.

---

## direction relevance

A sensor is only relevant if the AGV is moving toward that sensor.

For first version:

- front sensor matters only for forward approach
- rear sensor matters only for backward approach

So:

- front sensor relevant if `front_approach_mm_s > v_arm_mm_s`
- rear sensor relevant if `rear_approach_mm_s > v_arm_mm_s`

If a sensor is not relevant:

- it must not trigger hazard

---

## trailer / attachment rule

If an attachment covers a sensor, for example a trailer covering the rear sensor, that sensor must be disabled by configuration.

Do not try to infer this from obstacle distance.

The correct behavior is:

- masked sensor does not participate in collision prediction
- if motion is requested in a direction with no valid sensor coverage, that should be treated as blocked

So the trailer case must be solved by configuration, not by trying to classify odd measurements.

---

## obstacle sensor logic

The per-sensor logic should live in a separate internal file.

Its job is:

- read one sensor sample
- check freshness and validity
- apply asymmetric low-pass filter
- compare filtered distance to required stop distance
- return whether this sensor currently says hazard

The suggested filter is still:

```text
if z_raw_mm < d_hat_prev:
    d_hat = 0.6 * z_raw_mm + 0.4 * d_hat_prev
else:
    d_hat = 0.2 * z_raw_mm + 0.8 * d_hat_prev
```

This gives:

- fast reaction toward danger
- slower release away from danger

Only valid fresh samples may update the filter.

---

## collision decision

`collision_prediction` should decide hazard from:

- direction relevance
- sensor validity / freshness
- filtered obstacle distance
- required stopping distance

The required stopping distance should not be a fixed threshold only.

Use the usual form:

```text
d_required_mm =
    d_static_margin_mm +
    v_approach_mm_s * t_total_s +
    (v_approach_mm_s * v_approach_mm_s) / (2 * a_brake_min_mm_s2)
```

If a relevant sensor says the path is blocked, then:

```text
collision_blocked = true
```

If motion is requested in a direction with no valid active sensor coverage, then:

```text
collision_blocked = true
```

For first version the output remains only this bool.

---

## safe guard behavior

`safe_guard` should be simple.

It is not another decision module.

Its job is only to enforce safety once something else has already decided that safety stop is needed.

It should support:

- `trip()`
- `tick(now_ms)`
- reading unlock / reset request
- exposing whether the fault is latched

When latched it must:

1. stop motors
2. keep the stop latched
3. block further drive
4. hold indication state
5. unlock only from explicit decoded request

No automatic recovery.

---

## file structure

The planned structure is:

```text
core/control/collision_prediction/
    collision_prediction_pipeline.hpp
    collision_prediction_pipeline.cpp
    collision_prediction_logic.hpp
    collision_prediction_logic.cpp
    collision_tuning.hpp
    logic.md
    internal/
        vehicle_motion_estimator.hpp
        vehicle_motion_estimator.cpp
        collision_input_builder.hpp
        collision_input_builder.cpp
        obstacle_sensor_logic.hpp
        obstacle_sensor_logic.cpp

core/control/safe_guard/
    safe_guard_pipeline.hpp
    safe_guard_pipeline.cpp
    safe_guard_tuning.hpp
```

This keeps:

- hazard math in `collision_prediction`
- stop / latch behavior in `safe_guard`

---

## first implementation target

First implementation should aim for:

- `collision_prediction` outputs only `bool collision_blocked`
- `safe_guard` owns the latch
- `drive_control` respects `safe_guard`
- unlock comes from already decoded middleware state

No fault reason yet.
No debug snapshot yet.
