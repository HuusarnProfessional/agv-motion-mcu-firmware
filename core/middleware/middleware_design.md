# Middleware Design Notes (No Code Yet)

## Purpose
- Middleware is the protocol layer between `comm_uart` and controller logic.
- Middleware is platform-independent.
- Middleware does not contain hardware access code.

## Responsibilities
- Read incoming bytes from `comm_uart_api`.
- Build complete text lines (newline-terminated framing).
- Parse command name and argument payload.
- Dispatch parsed commands to registered handlers.
- Send outgoing status/messages through `comm_uart_api`.
- Track protocol/runtime errors (overflow, invalid format, unknown command).

## Non-Responsibilities
- No HAL access.
- No board/platform pin configuration.
- No direct motor/encoder/imu control logic.
- No blocking delays.

## Runtime Model
- Middleware runs from the superloop (`tick` style).
- Each tick:
- Pull available RX bytes (non-blocking).
- Assemble one or more complete lines.
- Parse and dispatch each line.
- Flush queued TX messages (non-blocking best effort).

## Command Model
- Incoming route table:
- Maps command name to handler callback.
- Outgoing route table:
- Maps symbolic event/status name to formatter/sender helper.
- No separate response-routing layer is required for now.

## Error Handling Plan
- Invalid line format -> reject + optional error message.
- Unknown command -> reject + optional error message.
- RX overflow -> drop current line and mark overflow event.
- TX failure -> return status so controller can retry/log.

## Data Ownership Plan
- Middleware owns RX assembly buffer.
- Middleware owns parse scratch buffer.
- Handler receives parsed command view/copies only.
- Middleware should avoid dynamic allocation.

## Suggested Next Implementation Steps
1. Define middleware public interface (header only).
2. Define route entry types and handler signature.
3. Implement RX line assembly.
4. Implement parser and dispatcher.
5. Implement TX helper path.
6. Add simple command tests using host build.

