# controller contract

detta dokument beskriver vad controller-utvecklare far och inte far gora.
syftet ar att controller-teamet bara jobbar mot api-lagret och inte bryter arkitekturgransen.

## tillatna include-omraden

controller-kod far inkludera:

- `core/api/*`
- `core/state/agv_state.hpp`
- egna controller-filer i `core/control/*`

controller-kod far inte inkludera:

- `core/impl/*`
- `board/*`
- `platform/*`
- `targets/*`
- vendor/hal headers direkt (`main.h`, `stm32*.h`, osv)

## tillatet ansvar i controller

- lasa sensorprover via api
- satta motor-kommandon via api
- bygga state machine/regler/beteende
- hantera fallback och safety-regler i logiklagret

## ej tillatet ansvar i controller

- pin-mappning
- hal-anrop
- register-lase/skriv
- timing-implementation pa us-niva for drivrutiner
- backend/impl selection

## data- och statusregler

- kontrollera alltid return-varde fran `read_sample(...)`
- kontrollera status-falt innan data anvands
- anvand `time_ms` for stale-check i logik
- hardkoda inte antaganden om status-konvertering mellan api och impl

## id-regler

- anvand konsekventa id:n i controller (`0U`, `1U`, ...)
- inga magic numbers spridda i flera filer
- om flera controllers anvander samma id-mappning: samla i en gemensam controller-header

## tick-regler

- controller ska vara icke-blockerande i `tick(...)`
- inga `HAL_Delay`, busy-waits eller langa loopar i controller
- all periodisk logik ska vara tidsstyrd med `now_ms`

## kodstil for controller-team

- hall funktioner sma och tydliga
- fulla namn fore forkortningar dar det ar rimligt
- kommentera bara dar kodens intention annars ar oklar
- anvand samma status-/felhanteringsmonster i hela control-paketet

## review-checklista (controller-pr)

- bara tillatna includes?
- inga anrop till impl/board/platform?
- alla sensorlasningar valideras?
- motorutgangar satts till sakert lage vid fel?
- tick ar icke-blockerande?

## api-exempel (laggst ner enligt overenskommelse)

```cpp
// motor: set command
motor_api::set_u(0U, 300);
motor_api::set_u(1U, 300);
```

```cpp
// encoder: read sample
encoder_api::encoder_sample encoder = {};
const bool got_encoder = encoder_api::read_sample(0U, encoder);
if (got_encoder && encoder.status == encoder_api::encoder_status::ok)
{
  const std::uint16_t angle = encoder.angle_raw_12bit;
  (void)angle;
}
```

```cpp
// imu: read sample
imu_api::imu_sample imu = {};
const bool got_imu = imu_api::read_sample(0U, imu);
if (got_imu && imu.gyroscope_state == imu_api::gyroscope_status::ok)
{
  const std::int32_t yaw_rate_mdps = imu.gyroscope_z_mdps;
  (void)yaw_rate_mdps;
}
```

```cpp
// obstacle: stop on near object
obstacle_api::obstacle_sample front = {};
const bool got_front = obstacle_api::read_sample(0U, front);
if (got_front && front.status == obstacle_api::obstacle_status::ok)
{
  if (front.distance_mm < 250U)
  {
    for (std::uint8_t motor_id = 0U; motor_id < 4U; ++motor_id)
    {
      motor_api::set_u(motor_id, 0);
    }
  }
}
```

```cpp
// voltage monitor: read battery rail
voltage_monitor_api::voltage_sample battery = {};
const bool got_battery = voltage_monitor_api::read_sample(0U, battery);
if (got_battery && battery.status == voltage_monitor_api::voltage_status::ok)
{
  const std::uint32_t battery_mv = battery.voltage_mv;
  (void)battery_mv;
}
```
