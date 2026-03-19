#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "core/api/encoder_api.hpp"
#include "core/api/middleware_api_ai.hpp"
#include "core/api/motor_api.hpp"
#include "core/control/RC_arc_controller_ai.hpp"
#include "core/control/RC_encoder_sample_guard_ai.hpp"
#include "core/impl/middleware_impl_ai.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

namespace rc_test_comm_middleware_ai
{
  struct state
  {
    std::uint32_t now_ms = 0U;
    std::uint32_t last_comm_rx_diag_ms = 0U;
    std::uint32_t last_uart_link_diag_ms = 0U;

    std::int32_t pos_x = 0;
    std::int32_t pos_y = 0;
    std::int32_t heading_deg = 0;
    std::int64_t heading_total_urad = 0;
    std::int64_t heading_zero_urad = 0;
    std::int32_t speed_cmd = 0;

    bool stop = false;
    std::int16_t pwm_hold_u = 0;

    bool motion_active = false;
    std::uint32_t motion_end_ms = 0U;
    std::int16_t motion_u = 0;

    // Encoder-based odometry using 4 wheel encoders.
    // Front ids are user-configurable through set_encoder_id_map.
    // Rear ids are paired automatically (id ^ 2) so all 4 channels are used.
    std::int32_t encoder_left_id = 0;
    std::int32_t encoder_right_id = 1;
    std::int32_t encoder_left_rear_id = 2;
    std::int32_t encoder_right_rear_id = 3;
    std::int32_t encoder_left_dir_sign = 1;
    std::int32_t encoder_right_dir_sign = 1;
    std::int32_t encoder_left_rear_dir_sign = 1;
    std::int32_t encoder_right_rear_dir_sign = 1;
    std::int32_t wheel_diameter_mm = 63;
    std::int32_t wheel_separation_mm = 164;
    std::int32_t yaw_effective_track_mm = 164;
    std::int32_t arc_effective_track_mm = 164;

    bool odom_have_prev = false;
    std::uint16_t prev_left_raw = 0U;
    std::uint16_t prev_right_raw = 0U;
    std::uint16_t prev_left_rear_raw = 0U;
    std::uint16_t prev_right_rear_raw = 0U;
    std::uint32_t odom_last_time_ms = 0U;
    rc_encoder_sample_guard_ai::config encoder_guard_cfg{};
    rc_encoder_sample_guard_ai::channel_state encoder_guard[4] = {};

    std::int64_t distance_left_mm_x1000 = 0;
    std::int64_t distance_right_mm_x1000 = 0;
    std::int64_t distance_center_mm_x1000 = 0;
    std::int64_t pose_x_mm_x1000 = 0;
    std::int64_t pose_y_mm_x1000 = 0;
    std::int32_t speed_mm_s = 0;

    bool distance_target_active = false;
    std::int64_t distance_target_center_mm_x1000 = 0;

    bool arc_active = false;
    std::int16_t arc_left_u = 0;
    std::int16_t arc_right_u = 0;
    std::int64_t arc_start_left_mm_x1000 = 0;
    std::int64_t arc_start_right_mm_x1000 = 0;
    std::int64_t arc_target_left_mm_x1000 = 0;
    std::int64_t arc_target_right_mm_x1000 = 0;
    std::int64_t arc_target_center_mm_x1000 = 0;
    std::int32_t arc_theta_target_urad = 0;
    std::int32_t arc_theta_ref_urad = 0;
    std::int32_t arc_theta_curr_urad = 0;
    std::int16_t arc_gain_permille = 700;
    std::uint32_t arc_start_ms = 0U;
    std::uint32_t arc_last_ctrl_ms = 0U;
    std::uint32_t arc_last_odom_ms = 0U;
    std::int32_t arc_theta_i_urad = 0;
    std::int16_t arc_left_cmd_u = 0;
    std::int16_t arc_right_cmd_u = 0;
    std::int16_t arc_left_out_u = 0;
    std::int16_t arc_right_out_u = 0;
    std::uint32_t odom_last_any_ok_ms = 0U;

    rc_arc_controller_ai::state arc_v2{};
    rc_arc_controller_ai::tuning arc_v2_tuning{};

    // Default side grouping for 4 motors.
    std::uint8_t motor_left_front_id = 0U;
    std::uint8_t motor_left_rear_id = 2U;
    std::uint8_t motor_right_front_id = 1U;
    std::uint8_t motor_right_rear_id = 3U;

    bool manual_motor_active = false;
    std::uint8_t manual_motor_id = 0U;
    std::int16_t manual_motor_u = 0;

    char rx_accum[192] = {};
    char parsed_line[192] = {};
    middleware_api_ai::io_buffers io{};
  };

  struct drive_defaults
  {
    std::int32_t wheel_diameter_mm = 63;
    std::int32_t wheel_separation_mm = 164;
    std::int32_t yaw_effective_track_mm = 164;
    std::int32_t arc_effective_track_mm = 164;
  };

  static constexpr bool k_uart_link_diag_periodic = false;
  static constexpr std::uint32_t k_uart_link_diag_period_ms = 2000U;
  static constexpr std::int64_t k_pi_urad = 3141593LL;
  static constexpr std::uint32_t k_arc_sensor_timeout_ms = 500U;
  static constexpr std::uint32_t k_arc_ctrl_period_ms = 10U;
  static constexpr std::int32_t k_arc_theta_done_tol_urad = 104720; // ~6 deg
  static constexpr std::int64_t k_arc_min_progress_mm_x1000 = 20000; // 20 mm
  static constexpr std::int64_t k_arc_distance_guard_mm_x1000 = 400000; // 400 mm
  static constexpr std::int32_t k_arc_theta_ki_permille = 120;
  static constexpr std::int32_t k_arc_theta_i_limit_urad = 400000; // ~23 deg equivalent integral cap
  static constexpr std::int16_t k_arc_cmd_slew_u_per_tick = 40;
  static constexpr std::int16_t k_arc_out_slew_u_per_tick = 25;
  static constexpr std::int16_t k_arc_min_active_u = 80;
  static constexpr std::int16_t k_arc_motor_deadzone_u = 140;

  inline bool read_encoder_start_samples_all_ok(state &s,
                                                encoder_api::encoder_sample &left_front,
                                                encoder_api::encoder_sample &right_front,
                                                encoder_api::encoder_sample &left_rear,
                                                encoder_api::encoder_sample &right_rear);

  inline std::int32_t clamp_i32(std::int32_t v, std::int32_t lo, std::int32_t hi)
  {
    if (v < lo)
    {
      return lo;
    }
    if (v > hi)
    {
      return hi;
    }
    return v;
  }

  inline std::int32_t normalize_dir_sign(std::int32_t v)
  {
    return (v < 0) ? -1 : 1;
  }

  inline std::int16_t pct_to_u(std::int32_t pct)
  {
    // PWM command is defined as 0..100 (%).
    const std::int32_t clamped = clamp_i32(pct, 0, 100);
    return static_cast<std::int16_t>(clamped * 10);
  }

  inline std::int16_t abs_i16(std::int16_t v)
  {
    return static_cast<std::int16_t>((v < 0) ? -v : v);
  }

  inline std::int32_t clamp_i64_to_i32(std::int64_t v)
  {
    if (v < static_cast<std::int64_t>(-2147483647L - 1L))
    {
      return static_cast<std::int32_t>(-2147483647L - 1L);
    }
    if (v > static_cast<std::int64_t>(2147483647L))
    {
      return static_cast<std::int32_t>(2147483647L);
    }
    return static_cast<std::int32_t>(v);
  }

  inline std::int16_t clamp_i16(std::int32_t v, std::int16_t lo, std::int16_t hi)
  {
    if (v < static_cast<std::int32_t>(lo))
    {
      return lo;
    }
    if (v > static_cast<std::int32_t>(hi))
    {
      return hi;
    }
    return static_cast<std::int16_t>(v);
  }

  inline std::int16_t sign_i16(std::int16_t v)
  {
    return static_cast<std::int16_t>((v < 0) ? -1 : 1);
  }

  inline std::int16_t slew_i16(std::int16_t current, std::int16_t target, std::int16_t step)
  {
    const std::int32_t c = static_cast<std::int32_t>(current);
    const std::int32_t t = static_cast<std::int32_t>(target);
    const std::int32_t s = static_cast<std::int32_t>((step > 0) ? step : 1);
    if (t > c + s)
    {
      return static_cast<std::int16_t>(c + s);
    }
    if (t < c - s)
    {
      return static_cast<std::int16_t>(c - s);
    }
    return target;
  }

  inline std::int64_t abs_i64(std::int64_t v)
  {
    return (v < 0) ? -v : v;
  }

  inline std::int64_t div_round_nearest(std::int64_t num, std::int64_t den)
  {
    if (den == 0)
    {
      return 0;
    }
    if (num >= 0)
    {
      return (num + (den / 2)) / den;
    }
    return (num - (den / 2)) / den;
  }

  inline std::int16_t apply_deadzone_i16(std::int16_t u, std::int16_t deadzone)
  {
    const std::int16_t a = abs_i16(u);
    if (a == 0)
    {
      return 0;
    }
    const std::int16_t dz = clamp_i16(deadzone, 0, 900);
    // Keep linear command shape. Only lift values that are below deadzone.
    const std::int16_t out_abs = (a < dz) ? dz : a;
    return static_cast<std::int16_t>((u < 0) ? -out_abs : out_abs);
  }

  inline void apply_deadzone_pair_i16(std::int16_t left_in,
                                      std::int16_t right_in,
                                      std::int16_t deadzone,
                                      std::int16_t &left_out,
                                      std::int16_t &right_out)
  {
    const std::int16_t a_left = abs_i16(left_in);
    const std::int16_t a_right = abs_i16(right_in);
    const std::int16_t dz = clamp_i16(deadzone, 0, 900);

    if (a_left == 0 && a_right == 0)
    {
      left_out = 0;
      right_out = 0;
      return;
    }

    if (a_left == 0 || a_right == 0)
    {
      left_out = (a_left == 0) ? 0 : apply_deadzone_i16(left_in, dz);
      right_out = (a_right == 0) ? 0 : apply_deadzone_i16(right_in, dz);
      return;
    }

    const std::int16_t min_a = (a_left < a_right) ? a_left : a_right;
    if (min_a >= dz || dz == 0)
    {
      left_out = left_in;
      right_out = right_in;
      return;
    }

    // Scale both sides by a common factor so the smaller side reaches deadzone,
    // while preserving left/right ratio (arc curvature).
    const std::int32_t num = static_cast<std::int32_t>(dz);
    const std::int32_t den = static_cast<std::int32_t>(min_a);
    const std::int32_t left_scaled_abs =
      (static_cast<std::int32_t>(a_left) * num + (den / 2)) / den;
    const std::int32_t right_scaled_abs =
      (static_cast<std::int32_t>(a_right) * num + (den / 2)) / den;

    const std::int16_t left_abs = clamp_i16(left_scaled_abs, 0, 1000);
    const std::int16_t right_abs = clamp_i16(right_scaled_abs, 0, 1000);
    left_out = static_cast<std::int16_t>((left_in < 0) ? -left_abs : left_abs);
    right_out = static_cast<std::int16_t>((right_in < 0) ? -right_abs : right_abs);
  }

  inline std::int32_t delta_raw12_wrapped(std::uint16_t prev_raw, std::uint16_t curr_raw)
  {
    std::int32_t d = static_cast<std::int32_t>(curr_raw) - static_cast<std::int32_t>(prev_raw);
    if (d > 2048)
    {
      d -= 4096;
    }
    else if (d < -2048)
    {
      d += 4096;
    }
    return d;
  }

  inline std::int32_t paired_encoder_id(std::int32_t front_id)
  {
    // 0<->2 and 1<->3 pairing keeps 4-channel wheel layout.
    return (front_id >= 0 && front_id <= 3) ? (front_id ^ 2) : front_id;
  }

  inline std::uint32_t max4_u32(std::uint32_t a,
                                std::uint32_t b,
                                std::uint32_t c,
                                std::uint32_t d)
  {
    std::uint32_t m = a;
    if (b > m)
    {
      m = b;
    }
    if (c > m)
    {
      m = c;
    }
    if (d > m)
    {
      m = d;
    }
    return m;
  }

  inline rc_encoder_sample_guard_ai::channel_state *guard_for_encoder_id(state &s, std::int32_t encoder_id)
  {
    if (encoder_id < 0 || encoder_id >= 4)
    {
      return nullptr;
    }
    return &s.encoder_guard[encoder_id];
  }

  inline void reset_all_encoder_guards(state &s)
  {
    for (std::uint8_t i = 0U; i < 4U; ++i)
    {
      rc_encoder_sample_guard_ai::reset(s.encoder_guard[i]);
    }
  }

  inline std::int64_t raw_delta_to_mm_x1000(std::int32_t raw_delta, std::int32_t wheel_diameter_mm)
  {
    // pi*1000 ~= 3142 gives mm*1000 output (sub-mm precision without float).
    static constexpr std::int32_t k_pi_x1000 = 3142;
    return (static_cast<std::int64_t>(raw_delta) *
            static_cast<std::int64_t>(wheel_diameter_mm) *
            static_cast<std::int64_t>(k_pi_x1000)) / 4096;
  }

  inline bool is_odom_usable_status(encoder_api::encoder_status status)
  {
    return status == encoder_api::encoder_status::ok ||
           status == encoder_api::encoder_status::stale;
  }

  inline void set_all_u(std::int16_t u)
  {
    for (std::uint8_t id = 0U; id < 4U; ++id)
    {
      motor_api::set_u(id, u);
    }
  }

  inline void set_side_u(state &s, std::int16_t left_u, std::int16_t right_u)
  {
    motor_api::set_u(s.motor_left_front_id, left_u);
    motor_api::set_u(s.motor_left_rear_id, left_u);
    motor_api::set_u(s.motor_right_front_id, right_u);
    motor_api::set_u(s.motor_right_rear_id, right_u);
  }

  inline bool is_valid_motor_id(std::int32_t motor_id)
  {
    return (motor_id >= 0) && (motor_id < 4);
  }

  inline void set_only_motor_u(std::uint8_t active_motor_id, std::int16_t u)
  {
    for (std::uint8_t id = 0U; id < 4U; ++id)
    {
      motor_api::set_u(id, (id == active_motor_id) ? u : 0);
    }
  }

  inline bool reached_target(std::int64_t curr, std::int64_t target, std::int16_t cmd_u)
  {
    if (cmd_u == 0)
    {
      return true;
    }
    if (cmd_u >= 0)
    {
      return curr >= target;
    }
    return curr <= target;
  }

  inline bool send_line_fmt(const char *fmt, std::int32_t a)
  {
    char line[96];
    std::snprintf(line, sizeof(line), fmt, static_cast<long>(a));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline bool send_line_fmt2(const char *fmt, std::int32_t a, std::int32_t b)
  {
    char line[96];
    std::snprintf(line, sizeof(line), fmt, static_cast<long>(a), static_cast<long>(b));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline bool send_encoder_u32(const char *label, std::int32_t encoder_id, std::uint32_t value)
  {
    char line[96];
    std::snprintf(line,
                  sizeof(line),
                  "%s %ld %lu",
                  (label != nullptr) ? label : "encoder",
                  static_cast<long>(encoder_id),
                  static_cast<unsigned long>(value));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline const char *encoder_status_text(encoder_api::encoder_status status)
  {
    switch (status)
    {
      case encoder_api::encoder_status::ok:
        return "ok";
      case encoder_api::encoder_status::no_signal:
        return "no_signal";
      case encoder_api::encoder_status::stale:
        return "stale";
      case encoder_api::encoder_status::invalid_duty:
        return "invalid_duty";
      case encoder_api::encoder_status::invalid_id:
        return "invalid_id";
      default:
        return "unknown";
    }
  }

  inline bool send_encoder_u32_with_status(const char *label,
                                           std::int32_t encoder_id,
                                           std::uint32_t value,
                                           encoder_api::encoder_status status)
  {
    char line[128];
    std::snprintf(line,
                  sizeof(line),
                  "%s %ld %lu %s",
                  (label != nullptr) ? label : "encoder",
                  static_cast<long>(encoder_id),
                  static_cast<unsigned long>(value),
                  encoder_status_text(status));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline bool send_line_fmt3(const char *fmt, std::int32_t a, std::int32_t b, std::int32_t c)
  {
    char line[128];
    std::snprintf(line, sizeof(line), fmt,
                  static_cast<long>(a),
                  static_cast<long>(b),
                  static_cast<long>(c));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline bool send_line_fmt4(const char *fmt, std::int32_t a, std::int32_t b, std::int32_t c, std::int32_t d)
  {
    char line[160];
    std::snprintf(line, sizeof(line), fmt,
                  static_cast<long>(a),
                  static_cast<long>(b),
                  static_cast<long>(c),
                  static_cast<long>(d));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline bool send_uart_link_diag_line(void)
  {
    platform_stm32_hal::uart_diag_snapshot d{};
    platform_stm32_hal::get_uart_diag_snapshot(d);

    char line[192];
    std::snprintf(line,
                  sizeof(line),
                  "status stm_uart_diag rx_bytes=%lu ring_ovf=%lu ore=%lu ne=%lu fe=%lu pe=%lu sent=%lu",
                  static_cast<unsigned long>(d.bytes_rx),
                  static_cast<unsigned long>(d.ring_overflow),
                  static_cast<unsigned long>(d.err_overrun),
                  static_cast<unsigned long>(d.err_noise),
                  static_cast<unsigned long>(d.err_framing),
                  static_cast<unsigned long>(d.err_parity),
                  static_cast<unsigned long>(d.sentinels));
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline void send_drive_done(void)
  {
    (void)middleware_api_ai::send_line("status drive_done");
  }

  inline void send_drive_abort(const char *reason)
  {
    char line[96];
    std::snprintf(line,
                  sizeof(line),
                  "status drive_abort %s",
                  (reason != nullptr) ? reason : "unknown");
    (void)middleware_api_ai::send_line(line);
  }

  inline void reset_arc_tracking(state &s)
  {
    s.arc_active = false;
    s.arc_left_u = 0;
    s.arc_right_u = 0;
    s.arc_start_left_mm_x1000 = 0;
    s.arc_start_right_mm_x1000 = 0;
    s.arc_target_left_mm_x1000 = 0;
    s.arc_target_right_mm_x1000 = 0;
    s.arc_target_center_mm_x1000 = 0;
    s.arc_theta_target_urad = 0;
    s.arc_theta_ref_urad = 0;
    s.arc_theta_curr_urad = 0;
    s.arc_start_ms = 0U;
    s.arc_last_ctrl_ms = 0U;
    s.arc_last_odom_ms = 0U;
    s.arc_theta_i_urad = 0;
    s.arc_left_cmd_u = 0;
    s.arc_right_cmd_u = 0;
    s.arc_left_out_u = 0;
    s.arc_right_out_u = 0;
  }

  inline void start_motion(state &s, std::int16_t signed_u, std::uint32_t duration_ms)
  {
    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    s.distance_target_active = false;
    s.manual_motor_active = false;
    s.manual_motor_u = 0;
    s.motion_u = signed_u;
    s.motion_end_ms = s.now_ms + duration_ms;
    s.motion_active = (duration_ms > 0U);
    if (!s.motion_active)
    {
      s.motion_u = 0;
    }
  }

  inline void start_distance_motion(state &s, std::int16_t signed_u, std::int64_t delta_center_mm_x1000)
  {
    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    s.manual_motor_active = false;
    s.manual_motor_u = 0;
    s.motion_u = signed_u;
    s.motion_active = (signed_u != 0 && delta_center_mm_x1000 != 0);
    s.distance_target_active = s.motion_active;
    s.distance_target_center_mm_x1000 = s.distance_center_mm_x1000 + delta_center_mm_x1000;
    s.motion_end_ms = s.now_ms;
    if (!s.motion_active)
    {
      s.motion_u = 0;
    }
  }

  inline bool cb_ping(void *ctx)
  {
    return (ctx != nullptr);
  }

  inline bool cb_get_speed(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("speed %ld", s.speed_mm_s);
  }

  inline bool cb_set_speed(void *ctx, std::int32_t speed)
  {
    state &s = *static_cast<state *>(ctx);
    s.speed_cmd = speed;
    return true;
  }

  inline bool cb_get_position(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt3("position %ld %ld %ld", s.pos_x, s.pos_y, s.heading_deg);
  }

  inline bool cb_set_position(void *ctx, std::int32_t x, std::int32_t y, std::int32_t heading_deg)
  {
    state &s = *static_cast<state *>(ctx);
    s.pos_x = x;
    s.pos_y = y;
    s.heading_deg = heading_deg;
    return true;
  }

  inline bool cb_get_angle(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("angle %ld", s.heading_deg);
  }

  inline bool cb_get_angle_deg(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("angle_deg %ld", s.heading_deg);
  }

  inline bool cb_set_angle_reset(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    s.heading_zero_urad = s.heading_total_urad;
    s.heading_deg = 0;
    return true;
  }

  inline bool cb_set_drive_angle(void *ctx, std::int32_t heading_deg)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t clamped_deg = clamp_i32(heading_deg, -3600, 3600);
    const std::int64_t desired_urad = div_round_nearest(static_cast<std::int64_t>(clamped_deg) * k_pi_urad, 180LL);
    s.heading_zero_urad = s.heading_total_urad - desired_urad;
    s.heading_deg = clamped_deg;
    return true;
  }

  inline bool cb_get_time(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("time %ld", static_cast<std::int32_t>(s.now_ms));
  }

  inline bool cb_get_status(void *ctx)
  {
    (void)ctx;
    return middleware_api_ai::send_line("status ok") == middleware_api_ai::handle_status::ok;
  }

  inline bool cb_get_stop(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("stop %ld", s.stop ? 1 : 0);
  }

  inline bool cb_get_uart_diag(void *ctx)
  {
    (void)ctx;
    return send_uart_link_diag_line();
  }

  inline bool cb_get_status_uwb(void *ctx)
  {
    (void)ctx;
    return middleware_api_ai::send_line("status_uwb 0") == middleware_api_ai::handle_status::ok;
  }

  inline bool cb_get_uwb_raw(void *ctx)
  {
    (void)ctx;
    return middleware_api_ai::send_line("uwb_raw 0") == middleware_api_ai::handle_status::ok;
  }

  inline bool cb_set_stop(void *ctx, bool stop)
  {
    state &s = *static_cast<state *>(ctx);
    s.stop = stop;
    if (s.stop)
    {
      reset_arc_tracking(s);
      rc_arc_controller_ai::reset(s.arc_v2);
      s.motion_active = false;
      s.distance_target_active = false;
      s.motion_u = 0;
      s.manual_motor_active = false;
      s.manual_motor_u = 0;
      set_all_u(0);
    }
    return true;
  }

  inline bool cb_set_shutdown(void *ctx, bool shutdown)
  {
    state &s = *static_cast<state *>(ctx);
    s.stop = shutdown;
    if (shutdown)
    {
      reset_arc_tracking(s);
      rc_arc_controller_ai::reset(s.arc_v2);
      s.motion_active = false;
      s.distance_target_active = false;
      s.motion_u = 0;
      s.manual_motor_active = false;
      s.manual_motor_u = 0;
      set_all_u(0);
    }
    return true;
  }

  inline bool cb_set_drive_forward(void *ctx, std::int32_t mm)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t mm_abs = (mm < 0) ? -mm : mm;
    const std::int16_t base_u = abs_i16(s.pwm_hold_u);
    if (mm_abs <= 0 || base_u <= 0)
    {
      return false;
    }
    const std::int64_t delta_center_mm_x1000 = static_cast<std::int64_t>(mm_abs) * 1000;
    start_distance_motion(s, base_u, delta_center_mm_x1000);
    return s.motion_active;
  }

  inline bool cb_set_drive_backward(void *ctx, std::int32_t mm)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t mm_abs = (mm < 0) ? -mm : mm;
    const std::int16_t base_u = abs_i16(s.pwm_hold_u);
    if (mm_abs <= 0 || base_u <= 0)
    {
      return false;
    }
    const std::int64_t delta_center_mm_x1000 = static_cast<std::int64_t>(mm_abs) * 1000;
    start_distance_motion(s, static_cast<std::int16_t>(-base_u), -delta_center_mm_x1000);
    return s.motion_active;
  }

  inline bool cb_set_drive_forward_mm(void *ctx, std::int32_t mm)
  {
    return cb_set_drive_forward(ctx, mm);
  }

  inline bool cb_set_drive_backward_mm(void *ctx, std::int32_t mm)
  {
    return cb_set_drive_backward(ctx, mm);
  }

  inline bool cb_set_drive_forward_ms(void *ctx, std::int32_t ms)
  {
    state &s = *static_cast<state *>(ctx);
    const std::uint32_t duration_ms = static_cast<std::uint32_t>(clamp_i32(ms, 0, 30000));
    const std::int16_t base_u = abs_i16(s.pwm_hold_u);
    if (duration_ms == 0U || base_u <= 0)
    {
      return false;
    }
    start_motion(s, base_u, duration_ms);
    return s.motion_active;
  }

  inline bool cb_set_drive_backward_ms(void *ctx, std::int32_t ms)
  {
    state &s = *static_cast<state *>(ctx);
    const std::uint32_t duration_ms = static_cast<std::uint32_t>(clamp_i32(ms, 0, 30000));
    const std::int16_t base_u = abs_i16(s.pwm_hold_u);
    if (duration_ms == 0U || base_u <= 0)
    {
      return false;
    }
    start_motion(s, static_cast<std::int16_t>(-base_u), duration_ms);
    return s.motion_active;
  }

  inline bool cb_set_drive_arc_mm_deg(void *ctx, std::int32_t radius_mm, std::int32_t angle_deg)
  {
    state &s = *static_cast<state *>(ctx);

    if (angle_deg == 0)
    {
      return false;
    }

    // Use arc-effective track for arc command split so command model and
    // feedback model are consistent during radius tracking.
    const std::int32_t wheel_sep_mm = clamp_i32(s.arc_effective_track_mm, 20, 2000);
    const std::int32_t clamped_radius_mm = clamp_i32(radius_mm, -10000, 10000);
    const std::int32_t clamped_angle_deg = clamp_i32(angle_deg, -1080, 1080);

    // dL = (R - B/2)*theta, dR = (R + B/2)*theta, theta in rad.
    // Scaled to mm*1000 using pi*1000 ~= 3142.
    const std::int64_t left_num = static_cast<std::int64_t>(2 * clamped_radius_mm - wheel_sep_mm) *
                                  static_cast<std::int64_t>(clamped_angle_deg) *
                                  static_cast<std::int64_t>(3142);
    const std::int64_t right_num = static_cast<std::int64_t>(2 * clamped_radius_mm + wheel_sep_mm) *
                                   static_cast<std::int64_t>(clamped_angle_deg) *
                                   static_cast<std::int64_t>(3142);
    const std::int64_t d_left_mm_x1000 = left_num / 360;
    const std::int64_t d_right_mm_x1000 = right_num / 360;

    if (d_left_mm_x1000 == 0 && d_right_mm_x1000 == 0)
    {
      return false;
    }
    const std::int64_t d_center_mm_x1000 = (d_left_mm_x1000 + d_right_mm_x1000) / 2;

    const std::int16_t base_u = abs_i16(s.pwm_hold_u);
    if (base_u <= 0)
    {
      return false;
    }

    const std::int64_t abs_left = abs_i64(d_left_mm_x1000);
    const std::int64_t abs_right = abs_i64(d_right_mm_x1000);
    const std::int64_t abs_max = (abs_left >= abs_right) ? abs_left : abs_right;
    if (abs_max == 0)
    {
      return false;
    }

    std::int16_t left_u = 0;
    if (abs_left > 0)
    {
      const std::int64_t scaled = (static_cast<std::int64_t>(base_u) * abs_left + (abs_max / 2)) / abs_max;
      left_u = static_cast<std::int16_t>((scaled > 0) ? scaled : 1);
      if (d_left_mm_x1000 < 0)
      {
        left_u = static_cast<std::int16_t>(-left_u);
      }
    }

    std::int16_t right_u = 0;
    if (abs_right > 0)
    {
      const std::int64_t scaled = (static_cast<std::int64_t>(base_u) * abs_right + (abs_max / 2)) / abs_max;
      right_u = static_cast<std::int16_t>((scaled > 0) ? scaled : 1);
      if (d_right_mm_x1000 < 0)
      {
        right_u = static_cast<std::int16_t>(-right_u);
      }
    }

    if (left_u == 0 && right_u == 0)
    {
      return false;
    }

    s.motion_active = false;
    s.distance_target_active = false;
    s.motion_u = 0;
    s.manual_motor_active = false;
    s.manual_motor_u = 0;
    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    reset_all_encoder_guards(s);

    // Re-sync odometry baseline on every arc start so previous run history
    // (stale/no_signal bursts and accumulated deltas) does not bias the next arc.
    encoder_api::encoder_sample left_front_start{};
    encoder_api::encoder_sample right_front_start{};
    encoder_api::encoder_sample left_rear_start{};
    encoder_api::encoder_sample right_rear_start{};
    if (!read_encoder_start_samples_all_ok(s,
                                           left_front_start,
                                           right_front_start,
                                           left_rear_start,
                                           right_rear_start))
    {
      return false;
    }

    // Auto-zero arc-relevant odometry for each new arc command.
    s.distance_left_mm_x1000 = 0;
    s.distance_right_mm_x1000 = 0;
    s.distance_center_mm_x1000 = 0;
    s.speed_mm_s = 0;
    s.heading_zero_urad = s.heading_total_urad;
    s.heading_deg = 0;

    s.prev_left_raw = left_front_start.angle_raw_12bit;
    s.prev_right_raw = right_front_start.angle_raw_12bit;
    s.prev_left_rear_raw = left_rear_start.angle_raw_12bit;
    s.prev_right_rear_raw = right_rear_start.angle_raw_12bit;
    s.odom_last_time_ms = max4_u32(left_front_start.time_ms,
                                   right_front_start.time_ms,
                                   left_rear_start.time_ms,
                                   right_rear_start.time_ms);
    s.odom_have_prev = true;

    s.arc_active = true;
    s.arc_left_u = left_u;
    s.arc_right_u = right_u;
    s.arc_start_left_mm_x1000 = 0;
    s.arc_start_right_mm_x1000 = 0;
    s.arc_target_left_mm_x1000 = d_left_mm_x1000;
    s.arc_target_right_mm_x1000 = d_right_mm_x1000;
    s.arc_target_center_mm_x1000 = d_center_mm_x1000;
    s.arc_theta_target_urad = clamp_i64_to_i32(
      (static_cast<std::int64_t>(clamped_angle_deg) * k_pi_urad) / 180LL);
    s.arc_theta_ref_urad = 0;
    s.arc_theta_curr_urad = 0;
    s.arc_start_ms = s.now_ms;
    s.arc_last_ctrl_ms = s.now_ms;
    s.arc_last_odom_ms = s.odom_last_time_ms;
    s.arc_theta_i_urad = 0;
    s.arc_left_cmd_u = left_u;
    s.arc_right_cmd_u = right_u;
    // Start output at the feedforward split so turn-in is immediate.
    s.arc_left_out_u = left_u;
    s.arc_right_out_u = right_u;
    s.odom_last_any_ok_ms = s.now_ms;
    return true;
  }

  inline bool cb_set_drive_arc_v2_mm_deg(void *ctx, std::int32_t radius_mm, std::int32_t angle_deg)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int16_t base_u = abs_i16(s.pwm_hold_u);
    if (base_u <= 0 || angle_deg == 0)
    {
      return false;
    }

    encoder_api::encoder_sample left_front_start{};
    encoder_api::encoder_sample right_front_start{};
    encoder_api::encoder_sample left_rear_start{};
    encoder_api::encoder_sample right_rear_start{};
    if (!read_encoder_start_samples_all_ok(s,
                                           left_front_start,
                                           right_front_start,
                                           left_rear_start,
                                           right_rear_start))
    {
      return false;
    }

    // Cancel other drive modes before starting v2 arc.
    s.motion_active = false;
    s.distance_target_active = false;
    s.motion_u = 0;
    s.manual_motor_active = false;
    s.manual_motor_u = 0;
    reset_arc_tracking(s);

    rc_arc_controller_ai::start_request req{};
    req.radius_mm = radius_mm;
    req.angle_deg = angle_deg;
    req.base_u = base_u;
    req.arc_track_mm = clamp_i32(s.arc_effective_track_mm, 20, 2000);
    req.yaw_track_mm = clamp_i32(s.yaw_effective_track_mm, 20, 2000);
    req.now_ms = s.now_ms;
    req.left_mm_x1000 = s.distance_left_mm_x1000;
    req.right_mm_x1000 = s.distance_right_mm_x1000;

    if (!rc_arc_controller_ai::start(s.arc_v2, req))
    {
      return false;
    }

    // Rebase heading at arc-v2 start for local heading feedback observability.
    s.heading_zero_urad = s.heading_total_urad;
    s.heading_deg = 0;
    s.odom_last_any_ok_ms = s.now_ms;
    set_side_u(s, s.arc_v2.left_cmd_u, s.arc_v2.right_cmd_u);
    return true;
  }

  inline bool cb_set_pwm_pct(void *ctx, std::int32_t pct)
  {
    state &s = *static_cast<state *>(ctx);
    s.pwm_hold_u = pct_to_u(pct);
    // set_pwm stores default throttle only; it should not start motion by itself.
    return true;
  }

  inline bool cb_get_pwm_pct(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t pct = static_cast<std::int32_t>(s.pwm_hold_u) / 10;
    return send_line_fmt("pwm %ld", pct);
  }

  inline bool read_encoder_sample_from_api(state &s,
                                           std::int32_t encoder_id,
                                           encoder_api::encoder_sample &sample_out)
  {
    (void)s;
    if (encoder_id < 0 || encoder_id > 255)
    {
      return false;
    }

    if (!encoder_api::read_sample(static_cast<std::uint8_t>(encoder_id), sample_out))
    {
      return false;
    }

    return true;
  }

  inline bool read_encoder_sample_for_odom(state &s,
                                           std::int32_t encoder_id,
                                           encoder_api::encoder_sample &sample_out)
  {
    if (!read_encoder_sample_from_api(s, encoder_id, sample_out))
    {
      return false;
    }

    if (rc_encoder_sample_guard_ai::channel_state *guard = guard_for_encoder_id(s, encoder_id))
    {
      (void)rc_encoder_sample_guard_ai::sanitize(*guard, s.encoder_guard_cfg, sample_out);
    }

    return true;
  }

  inline bool read_encoder_ok_sample_from_api(state &s,
                                              std::int32_t encoder_id,
                                              encoder_api::encoder_sample &sample_out)
  {
    if (!read_encoder_sample_from_api(s, encoder_id, sample_out))
    {
      return false;
    }

    return sample_out.status == encoder_api::encoder_status::ok;
  }

  inline bool read_encoder_start_samples_all_ok(state &s,
                                                encoder_api::encoder_sample &left_front,
                                                encoder_api::encoder_sample &right_front,
                                                encoder_api::encoder_sample &left_rear,
                                                encoder_api::encoder_sample &right_rear)
  {
    return read_encoder_ok_sample_from_api(s, s.encoder_left_id, left_front) &&
           read_encoder_ok_sample_from_api(s, s.encoder_right_id, right_front) &&
           read_encoder_ok_sample_from_api(s, s.encoder_left_rear_id, left_rear) &&
           read_encoder_ok_sample_from_api(s, s.encoder_right_rear_id, right_rear);
  }

  inline void update_odometry_from_encoders(state &s)
  {
    encoder_api::encoder_sample left_front{};
    encoder_api::encoder_sample right_front{};
    encoder_api::encoder_sample left_rear{};
    encoder_api::encoder_sample right_rear{};

    const bool left_front_have = read_encoder_sample_for_odom(s, s.encoder_left_id, left_front);
    const bool right_front_have = read_encoder_sample_for_odom(s, s.encoder_right_id, right_front);
    const bool left_rear_have = read_encoder_sample_for_odom(s, s.encoder_left_rear_id, left_rear);
    const bool right_rear_have = read_encoder_sample_for_odom(s, s.encoder_right_rear_id, right_rear);

    const bool left_front_usable = left_front_have && is_odom_usable_status(left_front.status);
    const bool right_front_usable = right_front_have && is_odom_usable_status(right_front.status);
    const bool left_rear_usable = left_rear_have && is_odom_usable_status(left_rear.status);
    const bool right_rear_usable = right_rear_have && is_odom_usable_status(right_rear.status);

    const bool left_side_usable = left_front_usable || left_rear_usable;
    const bool right_side_usable = right_front_usable || right_rear_usable;
    if (!left_side_usable || !right_side_usable)
    {
      return;
    }

    const std::uint32_t sample_time_ms = max4_u32(left_front.time_ms,
                                                  right_front.time_ms,
                                                  left_rear.time_ms,
                                                  right_rear.time_ms);

    if (!s.odom_have_prev)
    {
      // First baseline uses all four so later side averaging starts aligned.
      if (!(left_front_usable && right_front_usable && left_rear_usable && right_rear_usable))
      {
        return;
      }

      s.prev_left_raw = left_front.angle_raw_12bit;
      s.prev_right_raw = right_front.angle_raw_12bit;
      s.prev_left_rear_raw = left_rear.angle_raw_12bit;
      s.prev_right_rear_raw = right_rear.angle_raw_12bit;
      s.odom_last_time_ms = sample_time_ms;
      s.odom_last_any_ok_ms = s.now_ms;
      s.odom_have_prev = true;
      return;
    }

    if (sample_time_ms <= s.odom_last_time_ms)
    {
      return;
    }

    const std::uint32_t dt_ms = sample_time_ms - s.odom_last_time_ms;

    std::int64_t d_left_raw_sum = 0;
    std::uint32_t d_left_raw_count = 0U;
    if (left_front_usable)
    {
      const std::int32_t d = delta_raw12_wrapped(s.prev_left_raw, left_front.angle_raw_12bit) *
                             normalize_dir_sign(s.encoder_left_dir_sign);
      s.prev_left_raw = left_front.angle_raw_12bit;
      d_left_raw_sum += static_cast<std::int64_t>(d);
      ++d_left_raw_count;
    }
    if (left_rear_usable)
    {
      const std::int32_t d = delta_raw12_wrapped(s.prev_left_rear_raw, left_rear.angle_raw_12bit) *
                             normalize_dir_sign(s.encoder_left_rear_dir_sign);
      s.prev_left_rear_raw = left_rear.angle_raw_12bit;
      d_left_raw_sum += static_cast<std::int64_t>(d);
      ++d_left_raw_count;
    }

    std::int64_t d_right_raw_sum = 0;
    std::uint32_t d_right_raw_count = 0U;
    if (right_front_usable)
    {
      const std::int32_t d = delta_raw12_wrapped(s.prev_right_raw, right_front.angle_raw_12bit) *
                             normalize_dir_sign(s.encoder_right_dir_sign);
      s.prev_right_raw = right_front.angle_raw_12bit;
      d_right_raw_sum += static_cast<std::int64_t>(d);
      ++d_right_raw_count;
    }
    if (right_rear_usable)
    {
      const std::int32_t d = delta_raw12_wrapped(s.prev_right_rear_raw, right_rear.angle_raw_12bit) *
                             normalize_dir_sign(s.encoder_right_rear_dir_sign);
      s.prev_right_rear_raw = right_rear.angle_raw_12bit;
      d_right_raw_sum += static_cast<std::int64_t>(d);
      ++d_right_raw_count;
    }

    if (d_left_raw_count == 0U || d_right_raw_count == 0U)
    {
      return;
    }

    const std::int32_t d_left_raw = clamp_i64_to_i32(
      div_round_nearest(d_left_raw_sum, static_cast<std::int64_t>(d_left_raw_count)));
    const std::int32_t d_right_raw = clamp_i64_to_i32(
      div_round_nearest(d_right_raw_sum, static_cast<std::int64_t>(d_right_raw_count)));

    const std::int64_t d_left_mm_x1000 = raw_delta_to_mm_x1000(d_left_raw, s.wheel_diameter_mm);
    const std::int64_t d_right_mm_x1000 = raw_delta_to_mm_x1000(d_right_raw, s.wheel_diameter_mm);
    const std::int64_t d_center_mm_x1000 = (d_left_mm_x1000 + d_right_mm_x1000) / 2;
    const std::int32_t yaw_track_mm = clamp_i32(s.yaw_effective_track_mm, 20, 2000);
    const std::int64_t d_theta_urad = ((d_right_mm_x1000 - d_left_mm_x1000) * 1000LL) /
                                      static_cast<std::int64_t>(yaw_track_mm);

    s.distance_left_mm_x1000 += d_left_mm_x1000;
    s.distance_right_mm_x1000 += d_right_mm_x1000;
    s.distance_center_mm_x1000 += d_center_mm_x1000;
    s.heading_total_urad += d_theta_urad;

    // Integrate planar pose using midpoint heading during this increment.
    const std::int64_t heading_mid_urad = (s.heading_total_urad - s.heading_zero_urad) - (d_theta_urad / 2LL);
    const double heading_mid_rad = static_cast<double>(heading_mid_urad) / 1000000.0;
    const std::int64_t d_x_mm_x1000 = static_cast<std::int64_t>(
      std::llround(static_cast<double>(d_center_mm_x1000) * std::cos(heading_mid_rad)));
    const std::int64_t d_y_mm_x1000 = static_cast<std::int64_t>(
      std::llround(static_cast<double>(d_center_mm_x1000) * std::sin(heading_mid_rad)));
    s.pose_x_mm_x1000 += d_x_mm_x1000;
    s.pose_y_mm_x1000 += d_y_mm_x1000;

    const std::int64_t heading_rel_urad = s.heading_total_urad - s.heading_zero_urad;
    s.heading_deg = clamp_i64_to_i32(div_round_nearest(heading_rel_urad * 180LL, k_pi_urad));

    s.odom_last_time_ms = sample_time_ms;
    s.odom_last_any_ok_ms = s.now_ms;
    if (dt_ms > 0U)
    {
      s.speed_mm_s = clamp_i64_to_i32(d_center_mm_x1000 / static_cast<std::int64_t>(dt_ms));
    }
  }

  inline bool cb_get_distance(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t distance_mm = clamp_i64_to_i32(s.distance_center_mm_x1000 / 1000);
    return send_line_fmt("distance %ld", distance_mm);
  }

  inline bool cb_get_pose_x(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t x_mm = clamp_i64_to_i32(s.pose_x_mm_x1000 / 1000);
    return send_line_fmt("pose_x %ld", x_mm);
  }

  inline bool cb_get_pose_y(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    const std::int32_t y_mm = clamp_i64_to_i32(s.pose_y_mm_x1000 / 1000);
    return send_line_fmt("pose_y %ld", y_mm);
  }

  inline bool cb_set_distance_reset(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    s.distance_left_mm_x1000 = 0;
    s.distance_right_mm_x1000 = 0;
    s.distance_center_mm_x1000 = 0;
    s.pose_x_mm_x1000 = 0;
    s.pose_y_mm_x1000 = 0;
    s.speed_mm_s = 0;
    s.distance_target_active = false;
    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    s.motion_active = false;
    s.motion_u = 0;
    s.manual_motor_active = false;
    s.manual_motor_u = 0;
    reset_all_encoder_guards(s);
    set_all_u(0);

    encoder_api::encoder_sample left_front{};
    encoder_api::encoder_sample right_front{};
    encoder_api::encoder_sample left_rear{};
    encoder_api::encoder_sample right_rear{};
    if (read_encoder_start_samples_all_ok(s,
                                          left_front,
                                          right_front,
                                          left_rear,
                                          right_rear))
    {
      s.prev_left_raw = left_front.angle_raw_12bit;
      s.prev_right_raw = right_front.angle_raw_12bit;
      s.prev_left_rear_raw = left_rear.angle_raw_12bit;
      s.prev_right_rear_raw = right_rear.angle_raw_12bit;
      s.odom_last_time_ms = max4_u32(left_front.time_ms,
                                     right_front.time_ms,
                                     left_rear.time_ms,
                                     right_rear.time_ms);
      s.odom_last_any_ok_ms = s.now_ms;
      s.odom_have_prev = true;
    }
    else
    {
      s.odom_have_prev = false;
      s.prev_left_raw = 0U;
      s.prev_right_raw = 0U;
      s.prev_left_rear_raw = 0U;
      s.prev_right_rear_raw = 0U;
      s.odom_last_time_ms = 0U;
      s.odom_last_any_ok_ms = 0U;
    }

    return true;
  }

  inline bool cb_set_pose_reset(void *ctx)
  {
    // Pose reset is intentionally an alias of distance reset:
    // both commands reset the full odometry baseline.
    return cb_set_distance_reset(ctx);
  }

  inline bool cb_set_wheel_diameter_mm(void *ctx, std::int32_t mm)
  {
    state &s = *static_cast<state *>(ctx);
    s.wheel_diameter_mm = clamp_i32(mm, 10, 300);
    return true;
  }

  inline bool cb_get_wheel_diameter_mm(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("wheel_diameter_mm %ld", s.wheel_diameter_mm);
  }

  inline bool cb_set_wheel_separation_mm(void *ctx, std::int32_t mm)
  {
    state &s = *static_cast<state *>(ctx);
    s.wheel_separation_mm = clamp_i32(mm, 20, 2000);
    return true;
  }

  inline bool cb_get_wheel_separation_mm(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("wheel_separation_mm %ld", s.wheel_separation_mm);
  }

  inline bool cb_set_yaw_effective_track_mm(void *ctx, std::int32_t mm)
  {
    state &s = *static_cast<state *>(ctx);
    s.yaw_effective_track_mm = clamp_i32(mm, 20, 2000);
    return true;
  }

  inline bool cb_get_yaw_effective_track_mm(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("yaw_effective_track_mm %ld", s.yaw_effective_track_mm);
  }

  inline bool cb_set_arc_effective_track_mm(void *ctx, std::int32_t mm)
  {
    state &s = *static_cast<state *>(ctx);
    s.arc_effective_track_mm = clamp_i32(mm, 20, 2000);
    return true;
  }

  inline bool cb_get_arc_effective_track_mm(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt("arc_effective_track_mm %ld", s.arc_effective_track_mm);
  }

  inline bool cb_set_motor_id_map(void *ctx,
                                  std::int32_t front_left_id,
                                  std::int32_t front_right_id,
                                  std::int32_t rear_left_id,
                                  std::int32_t rear_right_id)
  {
    state &s = *static_cast<state *>(ctx);
    if (!is_valid_motor_id(front_left_id) ||
        !is_valid_motor_id(front_right_id) ||
        !is_valid_motor_id(rear_left_id) ||
        !is_valid_motor_id(rear_right_id))
    {
      return false;
    }

    s.motor_left_front_id = static_cast<std::uint8_t>(front_left_id);
    s.motor_right_front_id = static_cast<std::uint8_t>(front_right_id);
    s.motor_left_rear_id = static_cast<std::uint8_t>(rear_left_id);
    s.motor_right_rear_id = static_cast<std::uint8_t>(rear_right_id);
    return true;
  }

  inline bool cb_get_motor_id_map(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt4("motor_id_map %ld %ld %ld %ld",
                          s.motor_left_front_id,
                          s.motor_right_front_id,
                          s.motor_left_rear_id,
                          s.motor_right_rear_id);
  }

  inline bool cb_set_motor_u(void *ctx, std::int32_t motor_id, std::int32_t pct)
  {
    state &s = *static_cast<state *>(ctx);
    if (!is_valid_motor_id(motor_id) || s.stop)
    {
      return false;
    }

    const std::int16_t u = static_cast<std::int16_t>(clamp_i32(pct, -100, 100) * 10);

    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    s.motion_active = false;
    s.distance_target_active = false;
    s.motion_u = 0;

    s.manual_motor_id = static_cast<std::uint8_t>(motor_id);
    s.manual_motor_u = u;
    s.manual_motor_active = (u != 0);
    return true;
  }

  inline bool cb_set_encoder_id_map(void *ctx, std::int32_t left_id, std::int32_t right_id)
  {
    state &s = *static_cast<state *>(ctx);
    // Controller uses 4 physical encoder channels (0..3).
    // Rear ids are auto-paired from the chosen front ids.
    if (left_id < 0 || left_id > 3 ||
        right_id < 0 || right_id > 3 ||
        left_id == right_id ||
        paired_encoder_id(left_id) == right_id ||
        paired_encoder_id(right_id) == left_id)
    {
      return false;
    }

    s.encoder_left_id = left_id;
    s.encoder_right_id = right_id;
    s.encoder_left_rear_id = paired_encoder_id(left_id);
    s.encoder_right_rear_id = paired_encoder_id(right_id);

    // Re-sync odometry baselines so mapping change does not create a jump.
    s.odom_have_prev = false;
    s.prev_left_raw = 0U;
    s.prev_right_raw = 0U;
    s.prev_left_rear_raw = 0U;
    s.prev_right_rear_raw = 0U;
    s.odom_last_time_ms = 0U;
    s.speed_mm_s = 0;
    reset_all_encoder_guards(s);
    return true;
  }

  inline bool cb_get_encoder_id_map(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt2("encoder_id_map %ld %ld", s.encoder_left_id, s.encoder_right_id);
  }

  inline bool cb_set_encoder_dir_map(void *ctx, std::int32_t left_dir, std::int32_t right_dir)
  {
    state &s = *static_cast<state *>(ctx);
    if ((left_dir != -1 && left_dir != 1) ||
        (right_dir != -1 && right_dir != 1))
    {
      return false;
    }

    s.encoder_left_dir_sign = normalize_dir_sign(left_dir);
    s.encoder_right_dir_sign = normalize_dir_sign(right_dir);
    s.encoder_left_rear_dir_sign = s.encoder_left_dir_sign;
    s.encoder_right_rear_dir_sign = s.encoder_right_dir_sign;

    // Re-sync odometry baselines so direction-map change does not create a jump.
    s.odom_have_prev = false;
    s.prev_left_raw = 0U;
    s.prev_right_raw = 0U;
    s.prev_left_rear_raw = 0U;
    s.prev_right_rear_raw = 0U;
    s.odom_last_time_ms = 0U;
    s.speed_mm_s = 0;
    reset_all_encoder_guards(s);
    return true;
  }

  inline bool cb_get_encoder_dir_map(void *ctx)
  {
    state &s = *static_cast<state *>(ctx);
    return send_line_fmt2("encoder_dir_map %ld %ld", s.encoder_left_dir_sign, s.encoder_right_dir_sign);
  }

  inline bool cb_get_encoder_raw(void *ctx, std::int32_t encoder_id)
  {
    state &s = *static_cast<state *>(ctx);
    encoder_api::encoder_sample sample{};
    if (!read_encoder_sample_from_api(s, encoder_id, sample))
    {
      return false;
    }
    return send_encoder_u32_with_status("encoder_raw",
                                        encoder_id,
                                        sample.angle_raw_12bit,
                                        sample.status);
  }

  inline bool cb_get_encoder_deg(void *ctx, std::int32_t encoder_id)
  {
    state &s = *static_cast<state *>(ctx);
    encoder_api::encoder_sample sample{};
    if (!read_encoder_sample_from_api(s, encoder_id, sample))
    {
      return false;
    }
    return send_encoder_u32_with_status("encoder_deg",
                                        encoder_id,
                                        sample.angle_mdeg,
                                        sample.status);
  }

  inline bool cb_get_encoder_rad(void *ctx, std::int32_t encoder_id)
  {
    state &s = *static_cast<state *>(ctx);
    encoder_api::encoder_sample sample{};
    if (!read_encoder_sample_from_api(s, encoder_id, sample))
    {
      return false;
    }
    return send_encoder_u32_with_status("encoder_rad",
                                        encoder_id,
                                        sample.angle_mrad,
                                        sample.status);
  }

  inline bool cb_get_encoder_time(void *ctx, std::int32_t encoder_id)
  {
    state &s = *static_cast<state *>(ctx);
    encoder_api::encoder_sample sample{};
    if (!read_encoder_sample_from_api(s, encoder_id, sample))
    {
      return false;
    }
    return send_encoder_u32_with_status("encoder_time",
                                        encoder_id,
                                        sample.time_ms,
                                        sample.status);
  }

  inline bool cb_get_encoder_status(void *ctx, std::int32_t encoder_id)
  {
    state &s = *static_cast<state *>(ctx);
    encoder_api::encoder_sample sample{};
    if (!read_encoder_sample_from_api(s, encoder_id, sample))
    {
      return false;
    }

    return send_encoder_u32_with_status("encoder_status",
                                        encoder_id,
                                        static_cast<std::uint32_t>(sample.status),
                                        sample.status);
  }

  inline bool cb_set_slot(void *ctx, const char *slot_id, bool occupied)
  {
    (void)ctx;
    char line[96];
    std::snprintf(line,
                  sizeof(line),
                  "slot %s %d",
                  (slot_id != nullptr) ? slot_id : "",
                  occupied ? 1 : 0);
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline bool cb_mission_stub(void *ctx, const char *cmd_name, const char *arg_blob)
  {
    (void)ctx;
    (void)arg_blob;
    char line[96];
    std::snprintf(line,
                  sizeof(line),
                  "mission_stub %s",
                  (cmd_name != nullptr) ? cmd_name : "");
    return middleware_api_ai::send_line(line) == middleware_api_ai::handle_status::ok;
  }

  inline const middleware_api_ai::dispatch_ops &dispatch_table(void)
  {
    static const middleware_api_ai::dispatch_ops k_dispatch =
    {
      cb_ping,
      cb_get_speed,
      cb_set_speed,
      cb_get_position,
      cb_set_position,
      cb_get_angle,
      cb_get_angle_deg,
      cb_set_angle_reset,
      cb_set_drive_angle,
      cb_get_time,
      cb_get_status,
      cb_get_stop,
      cb_get_uart_diag,
      cb_get_status_uwb,
      cb_get_uwb_raw,
      cb_set_stop,
      cb_set_shutdown,
      cb_set_drive_forward,
      cb_set_drive_backward,
      cb_set_drive_forward_mm,
      cb_set_drive_backward_mm,
      cb_set_drive_forward_ms,
      cb_set_drive_backward_ms,
      cb_set_drive_arc_mm_deg,
      cb_set_drive_arc_v2_mm_deg,
      cb_get_distance,
      cb_get_pose_x,
      cb_get_pose_y,
      cb_set_distance_reset,
      cb_set_pose_reset,
      cb_set_wheel_diameter_mm,
      cb_get_wheel_diameter_mm,
      cb_set_wheel_separation_mm,
      cb_get_wheel_separation_mm,
      cb_set_yaw_effective_track_mm,
      cb_get_yaw_effective_track_mm,
      cb_set_arc_effective_track_mm,
      cb_get_arc_effective_track_mm,
      cb_set_motor_id_map,
      cb_get_motor_id_map,
      cb_set_motor_u,
      cb_set_encoder_id_map,
      cb_get_encoder_id_map,
      cb_set_encoder_dir_map,
      cb_get_encoder_dir_map,
      cb_set_pwm_pct,
      cb_get_pwm_pct,
      cb_get_encoder_raw,
      cb_get_encoder_deg,
      cb_get_encoder_rad,
      cb_get_encoder_time,
      cb_get_encoder_status,
      cb_set_slot,
      cb_mission_stub
    };
    return k_dispatch;
  }

  inline void init(state &s)
  {
    s.now_ms = 0U;
    s.last_comm_rx_diag_ms = 0U;
    s.last_uart_link_diag_ms = 0U;
    s.pos_x = 0;
    s.pos_y = 0;
    s.heading_deg = 0;
    s.heading_total_urad = 0;
    s.heading_zero_urad = 0;
    s.speed_cmd = 0;
    s.stop = false;
    s.pwm_hold_u = 0;
    s.motion_active = false;
    s.motion_end_ms = 0U;
    s.motion_u = 0;
    s.encoder_left_id = 0;
    s.encoder_right_id = 1;
    s.encoder_left_rear_id = 2;
    s.encoder_right_rear_id = 3;
    s.encoder_left_dir_sign = 1;
    s.encoder_right_dir_sign = 1;
    s.encoder_left_rear_dir_sign = 1;
    s.encoder_right_rear_dir_sign = 1;
    s.wheel_diameter_mm = 63;
    s.wheel_separation_mm = 164;
    s.yaw_effective_track_mm = 164;
    s.arc_effective_track_mm = 164;
    s.odom_have_prev = false;
    s.prev_left_raw = 0U;
    s.prev_right_raw = 0U;
    s.prev_left_rear_raw = 0U;
    s.prev_right_rear_raw = 0U;
    s.odom_last_time_ms = 0U;
    s.odom_last_any_ok_ms = 0U;
    s.encoder_guard_cfg = rc_encoder_sample_guard_ai::config{};
    reset_all_encoder_guards(s);
    s.distance_left_mm_x1000 = 0;
    s.distance_right_mm_x1000 = 0;
    s.distance_center_mm_x1000 = 0;
    s.pose_x_mm_x1000 = 0;
    s.pose_y_mm_x1000 = 0;
    s.speed_mm_s = 0;
    s.distance_target_active = false;
    s.distance_target_center_mm_x1000 = 0;
    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    s.arc_gain_permille = 700;
    s.arc_v2_tuning = rc_arc_controller_ai::tuning{};
    s.motor_left_front_id = 0U;
    s.motor_left_rear_id = 2U;
    s.motor_right_front_id = 1U;
    s.motor_right_rear_id = 3U;
    s.manual_motor_active = false;
    s.manual_motor_id = 0U;
    s.manual_motor_u = 0;

    set_all_u(0);

    s.io.rx_accum_buf = s.rx_accum;
    s.io.rx_accum_cap = sizeof(s.rx_accum);
    s.io.parsed_line_buf = s.parsed_line;
    s.io.parsed_line_cap = sizeof(s.parsed_line);

    middleware_api_ai::comm_link link{};
    link.impl = comm_api_ai::impl_kind::uart;
    link.transport = platform_stm32_hal::get_uart_transport_ops();
    link.transport_ctx = platform_stm32_hal::get_default_uart_transport_ctx();

    middleware_api_ai::init(middleware_impl_ai::get_impl_ops(),
                            &dispatch_table(),
                            &s,
                            &link,
                            &s.io);

    if (middleware_api_ai::is_ready())
    {
      (void)middleware_api_ai::send_line("status middleware_ready");
    }
    else
    {
      (void)middleware_api_ai::send_line("status middleware_not_ready");
    }
  }

  inline void apply_defaults(state &s, const drive_defaults &defaults)
  {
    s.wheel_diameter_mm = clamp_i32(defaults.wheel_diameter_mm, 10, 300);
    s.wheel_separation_mm = clamp_i32(defaults.wheel_separation_mm, 20, 2000);
    s.yaw_effective_track_mm = clamp_i32(defaults.yaw_effective_track_mm, 20, 2000);
    s.arc_effective_track_mm = clamp_i32(defaults.arc_effective_track_mm, 20, 2000);

    // Rebase after defaults update so the next odometry tick starts clean.
    s.odom_have_prev = false;
    s.prev_left_raw = 0U;
    s.prev_right_raw = 0U;
    s.prev_left_rear_raw = 0U;
    s.prev_right_rear_raw = 0U;
    s.odom_last_time_ms = 0U;
    s.odom_last_any_ok_ms = 0U;
    s.speed_mm_s = 0;
    reset_arc_tracking(s);
    rc_arc_controller_ai::reset(s.arc_v2);
    reset_all_encoder_guards(s);
  }

  inline void tick(state &s, std::uint32_t now_ms)
  {
    s.now_ms = now_ms;

    // Prioritize UART ingress first so command bytes are drained as early as possible.
    // This reduces risk of USART overrun when host sends bursts.
    const std::uint8_t comm_poll_budget = (s.arc_active || s.arc_v2.active) ? 2U : 12U;
    for (std::uint8_t i = 0U; i < comm_poll_budget; ++i)
    {
      bool had_line = false;
      const middleware_api_ai::handle_status st = middleware_api_ai::poll_once(had_line);

      // If comm layer dropped a corrupted/overflow line before parser,
      // emit one throttled status line so host side gets actionable feedback.
      if (!had_line && st == middleware_api_ai::handle_status::err_bad_format)
      {
        if (static_cast<std::int32_t>(s.now_ms - s.last_comm_rx_diag_ms) >= 250)
        {
          s.last_comm_rx_diag_ms = s.now_ms;
          (void)middleware_api_ai::send_line("status comm_rx_overflow");
        }
        continue;
      }

      if (!had_line)
      {
        break;
      }
    }

    if (k_uart_link_diag_periodic &&
        static_cast<std::int32_t>(s.now_ms - s.last_uart_link_diag_ms) >=
            static_cast<std::int32_t>(k_uart_link_diag_period_ms))
    {
      s.last_uart_link_diag_ms = s.now_ms;
      (void)send_uart_link_diag_line();
    }

    update_odometry_from_encoders(s);

    if (s.stop)
    {
      reset_arc_tracking(s);
      rc_arc_controller_ai::reset(s.arc_v2);
      s.motion_active = false;
      s.distance_target_active = false;
      s.motion_u = 0;
      s.manual_motor_active = false;
      s.manual_motor_u = 0;
      set_all_u(0);
      return;
    }

    if (s.arc_v2.active)
    {
      rc_arc_controller_ai::measurement meas{};
      meas.now_ms = s.now_ms;
      meas.odom_last_ok_ms = s.odom_last_any_ok_ms;
      meas.left_mm_x1000 = s.distance_left_mm_x1000;
      meas.right_mm_x1000 = s.distance_right_mm_x1000;

      const rc_arc_controller_ai::output out =
        rc_arc_controller_ai::step(s.arc_v2, meas, s.arc_v2_tuning);

      if (out.status == rc_arc_controller_ai::step_status::running)
      {
        if (out.diag == rc_arc_controller_ai::diag_event::startup_boost_on)
        {
          (void)middleware_api_ai::send_line("status arc_v2_startup_boost_on");
        }
        else if (out.diag == rc_arc_controller_ai::diag_event::startup_boost_off)
        {
          (void)middleware_api_ai::send_line("status arc_v2_startup_boost_off");
        }
        else if (out.diag == rc_arc_controller_ai::diag_event::stiction_kick)
        {
          (void)middleware_api_ai::send_line("status arc_v2_stiction_kick");
        }
        set_side_u(s, out.left_u, out.right_u);
        return;
      }

      set_all_u(0);
      if (out.status == rc_arc_controller_ai::step_status::abort_sensor_timeout)
      {
        send_drive_abort("arc_v2_sensor_timeout");
      }
      else if (out.status == rc_arc_controller_ai::step_status::abort_distance_guard)
      {
        send_drive_abort("arc_v2_distance_guard");
      }
      send_drive_done();
      return;
    }

    if (s.arc_active)
    {
      const std::int32_t yaw_track_mm = clamp_i32(s.arc_effective_track_mm, 20, 2000);
      const std::int64_t d_left_mm_x1000 = s.distance_left_mm_x1000 - s.arc_start_left_mm_x1000;
      const std::int64_t d_right_mm_x1000 = s.distance_right_mm_x1000 - s.arc_start_right_mm_x1000;
      const std::int64_t d_center_mm_x1000 = (d_left_mm_x1000 + d_right_mm_x1000) / 2;
      const std::int64_t d_diff_mm_x1000 = d_right_mm_x1000 - d_left_mm_x1000;

      // theta(rad) = (dR - dL)/B.
      // Distances are in mm*1000 so theta(urad) = (d_diff_mm_x1000 * 1000) / B(mm).
      s.arc_theta_curr_urad = clamp_i64_to_i32((d_diff_mm_x1000 * 1000LL) /
                                                static_cast<std::int64_t>(yaw_track_mm));

      const bool odom_fresh = (s.odom_last_time_ms != s.arc_last_odom_ms);
      if (odom_fresh)
      {
        s.arc_last_odom_ms = s.odom_last_time_ms;
      }

      const std::uint32_t ctrl_dt_ms_u = s.now_ms - s.arc_last_ctrl_ms;
      const std::uint32_t ctrl_dt_ms = (ctrl_dt_ms_u == 0U) ? 1U : ctrl_dt_ms_u;
      bool ran_control = false;

      if (ctrl_dt_ms_u >= k_arc_ctrl_period_ms)
      {
        s.arc_last_ctrl_ms = s.now_ms;
        ran_control = true;

        std::int32_t theta_ref_urad = s.arc_theta_target_urad;
        const std::int64_t target_basis = s.arc_target_center_mm_x1000;
        const std::int64_t curr_basis = d_center_mm_x1000;
        if (target_basis != 0)
        {
          // Radius-faithful reference: progress angle according to center
          // distance progression, not a single wheel progression.
          const std::int64_t ref_num = static_cast<std::int64_t>(s.arc_theta_target_urad) *
                                       curr_basis;
          std::int64_t ref = ref_num / target_basis;
          if (s.arc_theta_target_urad >= 0)
          {
            if (ref < 0)
            {
              ref = 0;
            }
            if (ref > s.arc_theta_target_urad)
            {
              ref = s.arc_theta_target_urad;
            }
          }
          else
          {
            if (ref > 0)
            {
              ref = 0;
            }
            if (ref < s.arc_theta_target_urad)
            {
              ref = s.arc_theta_target_urad;
            }
          }
          theta_ref_urad = clamp_i64_to_i32(ref);
        }

        s.arc_theta_ref_urad = theta_ref_urad;

        const std::int32_t theta_err_urad = s.arc_theta_ref_urad - s.arc_theta_curr_urad;
        if (odom_fresh)
        {
          const std::int64_t di = (static_cast<std::int64_t>(theta_err_urad) *
                                   static_cast<std::int64_t>(ctrl_dt_ms)) / 1000LL;
          s.arc_theta_i_urad = clamp_i64_to_i32(
            static_cast<std::int64_t>(s.arc_theta_i_urad) + di);
          s.arc_theta_i_urad = clamp_i32(
            s.arc_theta_i_urad,
            -k_arc_theta_i_limit_urad,
            k_arc_theta_i_limit_urad);
        }

        const std::int32_t du_p = clamp_i64_to_i32(
          (static_cast<std::int64_t>(theta_err_urad) *
           static_cast<std::int64_t>(s.arc_gain_permille)) / 1000000LL);
        const std::int32_t du_i = clamp_i64_to_i32(
          (static_cast<std::int64_t>(s.arc_theta_i_urad) *
           static_cast<std::int64_t>(k_arc_theta_ki_permille)) / 1000000LL);
        const std::int32_t du_total = du_p + du_i;

        std::int16_t du_limit = 320;
        const std::int16_t ff_left_abs = abs_i16(s.arc_left_u);
        const std::int16_t ff_right_abs = abs_i16(s.arc_right_u);
        if (ff_left_abs > 0 && ff_right_abs > 0)
        {
          const std::int16_t ff_max = (ff_left_abs > ff_right_abs) ? ff_left_abs : ff_right_abs;
          du_limit = clamp_i16(static_cast<std::int32_t>((ff_max * 9) / 10), 120, 700);
        }
        const std::int16_t du = clamp_i16(du_total, static_cast<std::int16_t>(-du_limit), du_limit);

        std::int16_t left_target = clamp_i16(static_cast<std::int32_t>(s.arc_left_u) - du, -1000, 1000);
        std::int16_t right_target = clamp_i16(static_cast<std::int32_t>(s.arc_right_u) + du, -1000, 1000);

        // Keep each side direction monotonic during the arc (no sign flip from correction).
        if (s.arc_left_u > 0 && left_target < 0)
        {
          left_target = 0;
        }
        else if (s.arc_left_u < 0 && left_target > 0)
        {
          left_target = 0;
        }
        else if (s.arc_left_u == 0)
        {
          left_target = 0;
        }

        if (s.arc_right_u > 0 && right_target < 0)
        {
          right_target = 0;
        }
        else if (s.arc_right_u < 0 && right_target > 0)
        {
          right_target = 0;
        }
        else if (s.arc_right_u == 0)
        {
          right_target = 0;
        }

        // Keep both sides moving during the arc (except very near completion),
        // to avoid forward-then-pivot behavior.
        const std::int32_t theta_to_target_urad_now = s.arc_theta_target_urad - s.arc_theta_curr_urad;
        const bool near_end = abs_i64(static_cast<std::int64_t>(theta_to_target_urad_now)) <=
                              static_cast<std::int64_t>(2 * k_arc_theta_done_tol_urad);
        if (!near_end)
        {
          if (s.arc_left_u != 0 && abs_i16(left_target) < k_arc_min_active_u)
          {
            left_target = static_cast<std::int16_t>(sign_i16(s.arc_left_u) * k_arc_min_active_u);
          }
          if (s.arc_right_u != 0 && abs_i16(right_target) < k_arc_min_active_u)
          {
            right_target = static_cast<std::int16_t>(sign_i16(s.arc_right_u) * k_arc_min_active_u);
          }
        }

        s.arc_left_cmd_u = slew_i16(s.arc_left_cmd_u, left_target, k_arc_cmd_slew_u_per_tick);
        s.arc_right_cmd_u = slew_i16(s.arc_right_cmd_u, right_target, k_arc_cmd_slew_u_per_tick);
      }

      if (!ran_control && s.arc_last_ctrl_ms == 0U)
      {
        s.arc_left_cmd_u = s.arc_left_u;
        s.arc_right_cmd_u = s.arc_right_u;
        s.arc_left_out_u = s.arc_left_u;
        s.arc_right_out_u = s.arc_right_u;
      }

      const std::int32_t theta_to_target_now_urad = s.arc_theta_target_urad - s.arc_theta_curr_urad;
      const bool near_end_now = abs_i64(static_cast<std::int64_t>(theta_to_target_now_urad)) <=
                                static_cast<std::int64_t>(2 * k_arc_theta_done_tol_urad);
      std::int16_t left_out = s.arc_left_cmd_u;
      std::int16_t right_out = s.arc_right_cmd_u;
      if (!near_end_now)
      {
        apply_deadzone_pair_i16(s.arc_left_cmd_u,
                                s.arc_right_cmd_u,
                                k_arc_motor_deadzone_u,
                                left_out,
                                right_out);
      }
      s.arc_left_out_u = slew_i16(s.arc_left_out_u, left_out, k_arc_out_slew_u_per_tick);
      s.arc_right_out_u = slew_i16(s.arc_right_out_u, right_out, k_arc_out_slew_u_per_tick);
      set_side_u(s, s.arc_left_out_u, s.arc_right_out_u);

      if (static_cast<std::int32_t>(s.now_ms - s.odom_last_any_ok_ms) >
          static_cast<std::int32_t>(k_arc_sensor_timeout_ms))
      {
        reset_arc_tracking(s);
        set_all_u(0);
        send_drive_abort("arc_sensor_timeout");
        send_drive_done();
        return;
      }

      const std::int32_t theta_to_target_urad = s.arc_theta_target_urad - s.arc_theta_curr_urad;
      bool angle_done = abs_i64(static_cast<std::int64_t>(theta_to_target_urad)) <=
                        static_cast<std::int64_t>(k_arc_theta_done_tol_urad);
      if (!angle_done)
      {
        if (s.arc_theta_target_urad >= 0)
        {
          angle_done = (s.arc_theta_curr_urad >= s.arc_theta_target_urad);
        }
        else
        {
          angle_done = (s.arc_theta_curr_urad <= s.arc_theta_target_urad);
        }
      }

      const std::int64_t s_target_abs = abs_i64(s.arc_target_center_mm_x1000);
      const std::int64_t s_curr_abs = abs_i64(d_center_mm_x1000);
      const std::int64_t s_guard_limit = s_target_abs +
                                         (s_target_abs / 2) +
                                         k_arc_distance_guard_mm_x1000;
      const bool guard_hit = (s_target_abs >= k_arc_min_progress_mm_x1000) &&
                             (s_curr_abs > s_guard_limit);

      if (angle_done || guard_hit)
      {
        reset_arc_tracking(s);
        set_all_u(0);
        if (guard_hit && !angle_done)
        {
          send_drive_abort("arc_distance_guard");
        }
        send_drive_done();
      }
      return;
    }

    if (s.motion_active)
    {
      if (s.distance_target_active)
      {
        const bool reached = (s.motion_u >= 0)
                           ? (s.distance_center_mm_x1000 >= s.distance_target_center_mm_x1000)
                           : (s.distance_center_mm_x1000 <= s.distance_target_center_mm_x1000);
        if (!reached)
        {
          set_all_u(s.motion_u);
          return;
        }
      }
      else if (static_cast<std::int32_t>(s.now_ms - s.motion_end_ms) < 0)
      {
        set_all_u(s.motion_u);
        return;
      }

      s.motion_active = false;
      s.distance_target_active = false;
      s.motion_u = 0;
      send_drive_done();
    }

    if (s.manual_motor_active)
    {
      set_only_motor_u(s.manual_motor_id, s.manual_motor_u);
      return;
    }

    set_all_u(0);
  }
}
