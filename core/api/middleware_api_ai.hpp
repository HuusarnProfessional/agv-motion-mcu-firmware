#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/comm_api_ai.hpp"

namespace middleware_api_ai
{
  enum class handle_status : std::uint8_t
  {
    ok = 0,
    err_bad_format,
    err_unknown,
    err_handler,
    not_initialized,
    invalid_arg
  };

  using tx_line_fn = bool (*)(void *tx_ctx, const char *line);

  struct tx_ops
  {
    tx_line_fn send_line;
    void *tx_ctx;
  };

  struct dispatch_ops
  {
    bool (*on_ping)(void *app_ctx);

    bool (*on_get_speed)(void *app_ctx);
    bool (*on_set_speed)(void *app_ctx, std::int32_t speed);

    bool (*on_get_position)(void *app_ctx);
    bool (*on_set_position)(void *app_ctx, std::int32_t x, std::int32_t y, std::int32_t heading_deg);

    bool (*on_get_angle)(void *app_ctx);
    bool (*on_get_angle_deg)(void *app_ctx);
    bool (*on_set_angle_reset)(void *app_ctx);
    bool (*on_set_drive_angle)(void *app_ctx, std::int32_t heading_deg);

    bool (*on_get_time)(void *app_ctx);
    bool (*on_get_status)(void *app_ctx);
    bool (*on_get_stop)(void *app_ctx);
    bool (*on_get_uart_diag)(void *app_ctx);
    bool (*on_get_status_uwb)(void *app_ctx);
    bool (*on_get_uwb_raw)(void *app_ctx);

    bool (*on_set_stop)(void *app_ctx, bool stop);
    bool (*on_set_shutdown)(void *app_ctx, bool shutdown);

    // Legacy commands (kept for compatibility).
    bool (*on_set_drive_forward)(void *app_ctx, std::int32_t mm);
    bool (*on_set_drive_backward)(void *app_ctx, std::int32_t mm);

    // New drive commands for testing path and semantics.
    bool (*on_set_drive_forward_mm)(void *app_ctx, std::int32_t mm);
    bool (*on_set_drive_backward_mm)(void *app_ctx, std::int32_t mm);
    bool (*on_set_drive_forward_ms)(void *app_ctx, std::int32_t ms);
    bool (*on_set_drive_backward_ms)(void *app_ctx, std::int32_t ms);
    bool (*on_set_drive_arc_mm_deg)(void *app_ctx, std::int32_t radius_mm, std::int32_t angle_deg);
    bool (*on_set_drive_arc_v2_mm_deg)(void *app_ctx, std::int32_t radius_mm, std::int32_t angle_deg);

    // Encoder-based distance/speed test controls.
    bool (*on_get_distance)(void *app_ctx);
    bool (*on_get_pose_x)(void *app_ctx);
    bool (*on_get_pose_y)(void *app_ctx);
    bool (*on_set_distance_reset)(void *app_ctx);
    bool (*on_set_pose_reset)(void *app_ctx);
    bool (*on_set_wheel_diameter_mm)(void *app_ctx, std::int32_t mm);
    bool (*on_get_wheel_diameter_mm)(void *app_ctx);
    bool (*on_set_wheel_separation_mm)(void *app_ctx, std::int32_t mm);
    bool (*on_get_wheel_separation_mm)(void *app_ctx);
    bool (*on_set_yaw_effective_track_mm)(void *app_ctx, std::int32_t mm);
    bool (*on_get_yaw_effective_track_mm)(void *app_ctx);
    bool (*on_set_arc_effective_track_mm)(void *app_ctx, std::int32_t mm);
    bool (*on_get_arc_effective_track_mm)(void *app_ctx);

    // Motor id map and per-motor manual test control.
    bool (*on_set_motor_id_map)(void *app_ctx,
                                std::int32_t front_left_id,
                                std::int32_t front_right_id,
                                std::int32_t rear_left_id,
                                std::int32_t rear_right_id);
    bool (*on_get_motor_id_map)(void *app_ctx);
    bool (*on_set_motor_u)(void *app_ctx, std::int32_t motor_id, std::int32_t pct);

    bool (*on_set_encoder_id_map)(void *app_ctx, std::int32_t left_id, std::int32_t right_id);
    bool (*on_get_encoder_id_map)(void *app_ctx);
    bool (*on_set_encoder_dir_map)(void *app_ctx, std::int32_t left_dir, std::int32_t right_dir);
    bool (*on_get_encoder_dir_map)(void *app_ctx);

    // Store default PWM level in percent (0..100 expected).
    bool (*on_set_pwm_pct)(void *app_ctx, std::int32_t pct);
    bool (*on_get_pwm_pct)(void *app_ctx);

    // Encoder command placeholders for upcoming encoder_api integration.
    bool (*on_get_encoder_raw)(void *app_ctx, std::int32_t encoder_id);
    bool (*on_get_encoder_deg)(void *app_ctx, std::int32_t encoder_id);
    bool (*on_get_encoder_rad)(void *app_ctx, std::int32_t encoder_id);
    bool (*on_get_encoder_time)(void *app_ctx, std::int32_t encoder_id);
    bool (*on_get_encoder_status)(void *app_ctx, std::int32_t encoder_id);

    bool (*on_set_slot)(void *app_ctx, const char *slot_id, bool occupied);

    // Temporary catch-all for mission commands during bring-up.
    bool (*on_mission_stub)(void *app_ctx, const char *cmd_name, const char *arg_blob);
  };

  using handle_line_impl_fn = handle_status (*)(const char *raw_line,
                                                const dispatch_ops &dispatch,
                                                void *app_ctx,
                                                const tx_ops &tx);

  struct impl_ops
  {
    handle_line_impl_fn handle_line;
  };

  struct comm_link
  {
    comm_api_ai::impl_kind impl;
    const comm_api_ai::transport_ops *transport;
    void *transport_ctx;
  };

  struct io_buffers
  {
    // comm_api internal RX accumulation buffer.
    char *rx_accum_buf;
    std::size_t rx_accum_cap;

    // middleware dispatch input (one parsed line).
    char *parsed_line_buf;
    std::size_t parsed_line_cap;
  };

  void init(const impl_ops *impl,
            const dispatch_ops *dispatch,
            void *app_ctx,
            const comm_link *link,
            const io_buffers *buffers);

  bool is_ready(void);

  // Direct call path if caller already has one full line.
  handle_status handle_line(const char *raw_line);

  // Poll comm_api once and dispatch at most one full line.
  handle_status poll_once(bool &out_had_line);

  // Optional raw transmit path (for status/telemetry lines).
  handle_status send_line(const char *line);
}
