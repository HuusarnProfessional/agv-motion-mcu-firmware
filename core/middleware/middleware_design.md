# Middleware Design Notes

## Purpose
- Middleware sits above `comm_uart_api`.
- Middleware is controller-side logic, not board/platform logic.
- Middleware owns packet framing, stream scheduling and incoming command parsing.

## Top-Level Files
- `core/middleware/incoming_middleware_pipeline.*`
- `core/middleware/outgoing_middleware_pipeline.*`
- `core/middleware/middleware_streams.hpp`

## Structure Rules
- `comm_uart_api` is the only UART abstraction layer.
- Middleware does not talk to board, platform or HAL.
- Top-level files stay clean:
  - pipeline = flow
  - streams = what gets scheduled
- Each payload has its own file under:
  - `core/middleware/outgoing_payloads/`
  - `core/middleware/incoming_payloads/`

## Stream Rules
- Stream order in the array is priority order.
- Stream names do not contain `ms`.
- Period is configured only through `period_ms`.
- `phase_offset_ms` is used to spread streams in time.
- One stream sends one payload.
- If two payloads should run at the same rate, they still get separate stream rows.

## Packet Rules
- Binary framing.
- Current packet format:
  - `sync`
  - `payload_id`
  - `payload_length`
  - `payload_bytes`
- No CRC yet.

## Outgoing Model
- Outgoing payloads read source data directly from control pipelines or subsystem APIs.
- Streams decide when a payload is sent.

## Incoming Model
- Middleware parses incoming packets and stores latest values or pending actions.
- Motion command is latest-value wins.
- Start/clear IMU calibration are consumable action flags.
- Debug stream control updates middleware stream enable state.

