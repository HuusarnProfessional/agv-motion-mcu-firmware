#pragma once

#include <cstdint>

namespace rc_arc_controller_ai
{
  enum class step_status : std::uint8_t
  {
    idle = 0,
    running,
    done,
    abort_sensor_timeout,
    abort_distance_guard
  };

  enum class diag_event : std::uint8_t
  {
    none = 0,
    startup_boost_on,
    startup_boost_off,
    stiction_kick
  };

  struct tuning
  {
    std::uint32_t ctrl_period_ms = 10U;
    std::uint32_t sensor_timeout_ms = 500U;
    std::int32_t theta_done_tol_urad = 104720;        // ~6 deg
    std::int64_t distance_guard_mm_x1000 = 400000;    // 400 mm
    std::int64_t min_progress_mm_x1000 = 20000;       // 20 mm
    std::int32_t kp_permille = 950;                   // u per rad-ish
    std::int32_t ki_permille = 160;                   // u per rad*s-ish
    std::int32_t i_limit_urad = 500000;               // integral clamp
    std::int16_t du_limit = 420;                      // correction clamp
    std::int16_t min_active_u = 140;                  // overcome static friction
    std::int32_t cmd_rate_u_per_s = 3000;             // max command slew-rate
    std::int32_t cmd_accel_u_per_s2 = 14000;          // max command acceleration
    std::int32_t cmd_jerk_u_per_s3 = 70000;           // max command jerk
    std::int32_t min_ref_progress_permille = 250;     // angle-priority launch
    std::uint32_t startup_boost_ms = 150U;            // extra kick from standstill
    std::int16_t startup_boost_u = 260;               // minimum |u| during startup boost
    std::uint32_t stall_check_ms = 120U;              // periodic stall detector
    std::int64_t stall_min_progress_mm_x1000 = 1200;  // 1.2 mm/interval threshold
    std::int16_t stall_cmd_min_u = 220;               // ignore tiny command regions
    std::uint32_t stall_kick_ms = 120U;               // re-kick duration
    std::int16_t stall_kick_u = 340;                  // re-kick amplitude
  };

  struct state
  {
    bool active = false;

    std::uint32_t start_ms = 0U;
    std::uint32_t last_ctrl_ms = 0U;

    std::int64_t start_left_mm_x1000 = 0;
    std::int64_t start_right_mm_x1000 = 0;

    std::int64_t target_left_mm_x1000 = 0;
    std::int64_t target_right_mm_x1000 = 0;
    std::int64_t target_center_mm_x1000 = 0;
    std::int32_t theta_target_urad = 0;
    std::int32_t yaw_track_mm = 164;

    std::int16_t left_ff_u = 0;
    std::int16_t right_ff_u = 0;
    std::int16_t left_cmd_u = 0;
    std::int16_t right_cmd_u = 0;
    std::int32_t left_rate_u_per_s = 0;
    std::int32_t right_rate_u_per_s = 0;
    std::int32_t left_accel_u_per_s2 = 0;
    std::int32_t right_accel_u_per_s2 = 0;

    std::int32_t theta_curr_urad = 0;
    std::int32_t theta_ref_urad = 0;
    std::int32_t theta_i_urad = 0;

    bool startup_boost_active = false;
    std::uint32_t stall_last_check_ms = 0U;
    std::int64_t stall_last_center_mm_x1000 = 0;
    std::uint32_t stall_kick_until_ms = 0U;
  };

  struct start_request
  {
    std::int32_t radius_mm = 0;
    std::int32_t angle_deg = 0;
    std::int16_t base_u = 0;
    std::int32_t arc_track_mm = 164;
    std::int32_t yaw_track_mm = 164;
    std::uint32_t now_ms = 0U;
    std::int64_t left_mm_x1000 = 0;
    std::int64_t right_mm_x1000 = 0;
  };

  struct measurement
  {
    std::uint32_t now_ms = 0U;
    std::uint32_t odom_last_ok_ms = 0U;
    std::int64_t left_mm_x1000 = 0;
    std::int64_t right_mm_x1000 = 0;
  };

  struct output
  {
    step_status status = step_status::idle;
    std::int16_t left_u = 0;
    std::int16_t right_u = 0;
    bool startup_boost_active = false;
    diag_event diag = diag_event::none;
  };

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

  inline std::int64_t abs_i64(std::int64_t v)
  {
    return (v < 0) ? -v : v;
  }

  inline std::int16_t abs_i16(std::int16_t v)
  {
    return static_cast<std::int16_t>((v < 0) ? -v : v);
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

  inline std::int16_t rate_step_from_dt(std::int16_t per_s, std::uint32_t dt_ms)
  {
    const std::int32_t step = static_cast<std::int32_t>(per_s) *
                              static_cast<std::int32_t>((dt_ms > 0U) ? dt_ms : 1U) / 1000;
    return static_cast<std::int16_t>((step > 0) ? step : 1);
  }

  inline std::int32_t slew_i32(std::int32_t current, std::int32_t target, std::int32_t step)
  {
    const std::int32_t s = (step > 0) ? step : 1;
    if (target > current + s)
    {
      return current + s;
    }
    if (target < current - s)
    {
      return current - s;
    }
    return target;
  }

  inline void reset(state &s)
  {
    s = state{};
  }

  inline void apply_pair_min_active(std::int16_t left_in,
                                    std::int16_t right_in,
                                    std::int16_t min_abs,
                                    std::int16_t &left_out,
                                    std::int16_t &right_out)
  {
    const std::int16_t a_left = abs_i16(left_in);
    const std::int16_t a_right = abs_i16(right_in);
    const std::int16_t min_a = (a_left < a_right) ? a_left : a_right;

    if (a_left == 0 && a_right == 0)
    {
      left_out = 0;
      right_out = 0;
      return;
    }

    if (a_left == 0 || a_right == 0)
    {
      left_out = (a_left == 0)
               ? 0
               : static_cast<std::int16_t>((left_in < 0) ? -min_abs : min_abs);
      right_out = (a_right == 0)
                ? 0
                : static_cast<std::int16_t>((right_in < 0) ? -min_abs : min_abs);
      return;
    }

    if (min_a >= min_abs || min_abs <= 0)
    {
      left_out = left_in;
      right_out = right_in;
      return;
    }

    const std::int32_t num = static_cast<std::int32_t>(min_abs);
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

  inline bool start(state &s, const start_request &req)
  {
    if (req.angle_deg == 0 || req.base_u <= 0)
    {
      return false;
    }

    static constexpr std::int64_t k_pi_urad = 3141593LL;
    static constexpr std::int64_t k_pi_x1000 = 3142LL;

    const std::int32_t arc_track_mm = clamp_i32(req.arc_track_mm, 20, 2000);
    const std::int32_t yaw_track_mm = clamp_i32(req.yaw_track_mm, 20, 2000);
    const std::int32_t radius_mm = clamp_i32(req.radius_mm, -10000, 10000);
    const std::int32_t angle_deg = clamp_i32(req.angle_deg, -1080, 1080);

    const std::int64_t left_num = static_cast<std::int64_t>(2 * radius_mm - arc_track_mm) *
                                  static_cast<std::int64_t>(angle_deg) *
                                  k_pi_x1000;
    const std::int64_t right_num = static_cast<std::int64_t>(2 * radius_mm + arc_track_mm) *
                                   static_cast<std::int64_t>(angle_deg) *
                                   k_pi_x1000;
    const std::int64_t target_left_mm_x1000 = left_num / 360LL;
    const std::int64_t target_right_mm_x1000 = right_num / 360LL;
    const std::int64_t target_center_mm_x1000 = (target_left_mm_x1000 + target_right_mm_x1000) / 2LL;

    const std::int64_t abs_left = abs_i64(target_left_mm_x1000);
    const std::int64_t abs_right = abs_i64(target_right_mm_x1000);
    const std::int64_t abs_max = (abs_left >= abs_right) ? abs_left : abs_right;
    if (abs_max == 0)
    {
      return false;
    }

    std::int16_t left_ff_u = 0;
    if (abs_left > 0)
    {
      const std::int64_t scaled =
        (static_cast<std::int64_t>(req.base_u) * abs_left + (abs_max / 2LL)) / abs_max;
      left_ff_u = static_cast<std::int16_t>((scaled > 0) ? scaled : 1);
      if (target_left_mm_x1000 < 0)
      {
        left_ff_u = static_cast<std::int16_t>(-left_ff_u);
      }
    }

    std::int16_t right_ff_u = 0;
    if (abs_right > 0)
    {
      const std::int64_t scaled =
        (static_cast<std::int64_t>(req.base_u) * abs_right + (abs_max / 2LL)) / abs_max;
      right_ff_u = static_cast<std::int16_t>((scaled > 0) ? scaled : 1);
      if (target_right_mm_x1000 < 0)
      {
        right_ff_u = static_cast<std::int16_t>(-right_ff_u);
      }
    }

    if (left_ff_u == 0 && right_ff_u == 0)
    {
      return false;
    }

    s = state{};
    s.active = true;
    s.start_ms = req.now_ms;
    s.last_ctrl_ms = req.now_ms;
    s.start_left_mm_x1000 = req.left_mm_x1000;
    s.start_right_mm_x1000 = req.right_mm_x1000;
    s.target_left_mm_x1000 = target_left_mm_x1000;
    s.target_right_mm_x1000 = target_right_mm_x1000;
    s.target_center_mm_x1000 = target_center_mm_x1000;
    s.theta_target_urad = static_cast<std::int32_t>(
      (static_cast<std::int64_t>(angle_deg) * k_pi_urad) / 180LL);
    s.yaw_track_mm = yaw_track_mm;
    s.left_ff_u = left_ff_u;
    s.right_ff_u = right_ff_u;
    s.left_cmd_u = left_ff_u;
    s.right_cmd_u = right_ff_u;
    s.stall_last_check_ms = req.now_ms;
    s.stall_last_center_mm_x1000 = 0;
    return true;
  }

  inline output step(state &s, const measurement &meas, const tuning &cfg)
  {
    output out{};
    if (!s.active)
    {
      out.status = step_status::idle;
      return out;
    }

    const std::uint32_t now_ms = meas.now_ms;
    if (static_cast<std::int32_t>(now_ms - meas.odom_last_ok_ms) >
        static_cast<std::int32_t>(cfg.sensor_timeout_ms))
    {
      s.active = false;
      out.status = step_status::abort_sensor_timeout;
      return out;
    }

    const std::int64_t d_left_mm_x1000 = meas.left_mm_x1000 - s.start_left_mm_x1000;
    const std::int64_t d_right_mm_x1000 = meas.right_mm_x1000 - s.start_right_mm_x1000;
    const std::int64_t d_center_mm_x1000 = (d_left_mm_x1000 + d_right_mm_x1000) / 2LL;
    const std::int64_t d_diff_mm_x1000 = d_right_mm_x1000 - d_left_mm_x1000;

    s.theta_curr_urad = static_cast<std::int32_t>((d_diff_mm_x1000 * 1000LL) /
                                                   static_cast<std::int64_t>(s.yaw_track_mm));

    std::int32_t progress_permille = 1000;
    if (s.target_center_mm_x1000 > 0)
    {
      const std::int64_t p = (d_center_mm_x1000 <= 0) ? 0 : (d_center_mm_x1000 * 1000LL) / s.target_center_mm_x1000;
      progress_permille = clamp_i32(static_cast<std::int32_t>(p), 0, 1000);
    }
    else if (s.target_center_mm_x1000 < 0)
    {
      const std::int64_t p = (d_center_mm_x1000 >= 0) ? 0 : ((-d_center_mm_x1000) * 1000LL) / (-s.target_center_mm_x1000);
      progress_permille = clamp_i32(static_cast<std::int32_t>(p), 0, 1000);
    }

    const std::int32_t ref_progress_permille =
      (progress_permille < cfg.min_ref_progress_permille)
      ? cfg.min_ref_progress_permille
      : progress_permille;
    s.theta_ref_urad = static_cast<std::int32_t>(
      (static_cast<std::int64_t>(s.theta_target_urad) *
       static_cast<std::int64_t>(ref_progress_permille)) / 1000LL);

    const std::uint32_t ctrl_dt_ms_u = now_ms - s.last_ctrl_ms;
    if (ctrl_dt_ms_u >= cfg.ctrl_period_ms)
    {
      const std::uint32_t dt_ms = (ctrl_dt_ms_u > 50U) ? 50U : ctrl_dt_ms_u;
      s.last_ctrl_ms = now_ms;

      const std::int32_t theta_err_urad = s.theta_ref_urad - s.theta_curr_urad;
      const std::int64_t di = (static_cast<std::int64_t>(theta_err_urad) *
                               static_cast<std::int64_t>(dt_ms)) / 1000LL;
      std::int64_t i_next = static_cast<std::int64_t>(s.theta_i_urad) + di;
      if (i_next < -static_cast<std::int64_t>(cfg.i_limit_urad))
      {
        i_next = -static_cast<std::int64_t>(cfg.i_limit_urad);
      }
      if (i_next > static_cast<std::int64_t>(cfg.i_limit_urad))
      {
        i_next = static_cast<std::int64_t>(cfg.i_limit_urad);
      }
      s.theta_i_urad = static_cast<std::int32_t>(i_next);

      const std::int32_t du_p = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(theta_err_urad) *
         static_cast<std::int64_t>(cfg.kp_permille)) / 1000000LL);
      const std::int32_t du_i = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(s.theta_i_urad) *
         static_cast<std::int64_t>(cfg.ki_permille)) / 1000000LL);
      const std::int16_t du = clamp_i16(du_p + du_i,
                                        static_cast<std::int16_t>(-cfg.du_limit),
                                        cfg.du_limit);

      std::int16_t left_target = clamp_i16(static_cast<std::int32_t>(s.left_ff_u) - du, -1000, 1000);
      std::int16_t right_target = clamp_i16(static_cast<std::int32_t>(s.right_ff_u) + du, -1000, 1000);

      if (s.left_ff_u > 0 && left_target < 0)
      {
        left_target = 0;
      }
      else if (s.left_ff_u < 0 && left_target > 0)
      {
        left_target = 0;
      }
      else if (s.left_ff_u == 0)
      {
        left_target = 0;
      }

      if (s.right_ff_u > 0 && right_target < 0)
      {
        right_target = 0;
      }
      else if (s.right_ff_u < 0 && right_target > 0)
      {
        right_target = 0;
      }
      else if (s.right_ff_u == 0)
      {
        right_target = 0;
      }

      const std::int32_t theta_to_target_now = s.theta_target_urad - s.theta_curr_urad;
      const bool near_end = abs_i64(static_cast<std::int64_t>(theta_to_target_now)) <=
                            static_cast<std::int64_t>(2 * cfg.theta_done_tol_urad);
      if (!near_end)
      {
        apply_pair_min_active(left_target, right_target, cfg.min_active_u, left_target, right_target);
      }

      const auto update_axis = [dt_ms, &cfg](std::int16_t &cmd_u,
                                             std::int32_t &rate_u_per_s,
                                             std::int32_t &accel_u_per_s2,
                                             std::int16_t target_u)
      {
        const std::int32_t dt = static_cast<std::int32_t>((dt_ms > 0U) ? dt_ms : 1U);
        const std::int32_t cmd = static_cast<std::int32_t>(cmd_u);
        const std::int32_t tgt = static_cast<std::int32_t>(target_u);
        const std::int32_t err = tgt - cmd;
        const std::int32_t desired_rate = clamp_i32((err * 1000) / dt,
                                                    -cfg.cmd_rate_u_per_s,
                                                    cfg.cmd_rate_u_per_s);
        const std::int32_t desired_accel = clamp_i32(((desired_rate - rate_u_per_s) * 1000) / dt,
                                                     -cfg.cmd_accel_u_per_s2,
                                                     cfg.cmd_accel_u_per_s2);
        const std::int32_t jerk_step = clamp_i32((cfg.cmd_jerk_u_per_s3 * dt) / 1000, 1, 1000000);
        accel_u_per_s2 = slew_i32(accel_u_per_s2, desired_accel, jerk_step);
        accel_u_per_s2 = clamp_i32(accel_u_per_s2,
                                   -cfg.cmd_accel_u_per_s2,
                                   cfg.cmd_accel_u_per_s2);

        rate_u_per_s += (accel_u_per_s2 * dt) / 1000;
        rate_u_per_s = clamp_i32(rate_u_per_s,
                                 -cfg.cmd_rate_u_per_s,
                                 cfg.cmd_rate_u_per_s);

        std::int32_t delta = (rate_u_per_s * dt) / 1000;
        if (delta == 0 && err != 0)
        {
          delta = (err > 0) ? 1 : -1;
        }
        std::int32_t next = cmd + delta;
        if ((err > 0 && next > tgt) || (err < 0 && next < tgt))
        {
          next = tgt;
          rate_u_per_s = 0;
          accel_u_per_s2 = 0;
        }
        cmd_u = clamp_i16(next, -1000, 1000);
      };

      update_axis(s.left_cmd_u, s.left_rate_u_per_s, s.left_accel_u_per_s2, left_target);
      update_axis(s.right_cmd_u, s.right_rate_u_per_s, s.right_accel_u_per_s2, right_target);
    }

    const std::int32_t theta_to_target_urad = s.theta_target_urad - s.theta_curr_urad;
    bool angle_done = abs_i64(static_cast<std::int64_t>(theta_to_target_urad)) <=
                      static_cast<std::int64_t>(cfg.theta_done_tol_urad);
    if (!angle_done)
    {
      if (s.theta_target_urad >= 0)
      {
        angle_done = (s.theta_curr_urad >= s.theta_target_urad);
      }
      else
      {
        angle_done = (s.theta_curr_urad <= s.theta_target_urad);
      }
    }

    const std::int64_t s_target_abs = abs_i64(s.target_center_mm_x1000);
    const std::int64_t s_curr_abs = abs_i64(d_center_mm_x1000);
    const std::int64_t s_guard_limit = s_target_abs + (s_target_abs / 2LL) + cfg.distance_guard_mm_x1000;
    const bool guard_hit = (s_target_abs >= cfg.min_progress_mm_x1000) &&
                           (s_curr_abs > s_guard_limit);

    if (guard_hit)
    {
      s.active = false;
      out.status = step_status::abort_distance_guard;
      return out;
    }

    if (angle_done)
    {
      s.active = false;
      out.status = step_status::done;
      return out;
    }

    std::int16_t left_out = s.left_cmd_u;
    std::int16_t right_out = s.right_cmd_u;

    // Stall detection and re-kick if command is present but progress is too low.
    if ((now_ms - s.stall_last_check_ms) >= cfg.stall_check_ms)
    {
      const std::int64_t delta_progress = abs_i64(d_center_mm_x1000 - s.stall_last_center_mm_x1000);
      const std::int16_t cmd_mag = (abs_i16(s.left_cmd_u) > abs_i16(s.right_cmd_u))
                                 ? abs_i16(s.left_cmd_u)
                                 : abs_i16(s.right_cmd_u);
      if (cmd_mag >= cfg.stall_cmd_min_u &&
          delta_progress < cfg.stall_min_progress_mm_x1000)
      {
        s.stall_kick_until_ms = now_ms + cfg.stall_kick_ms;
        out.diag = diag_event::stiction_kick;
      }
      s.stall_last_check_ms = now_ms;
      s.stall_last_center_mm_x1000 = d_center_mm_x1000;
    }

    // Startup boost: static friction is often much higher than rolling friction.
    // Keep the first short interval above breakaway torque so robot starts moving
    // without requiring an external push.
    const bool startup_active = (now_ms - s.start_ms) < cfg.startup_boost_ms;
    if (startup_active != s.startup_boost_active)
    {
      if (out.diag == diag_event::none)
      {
        out.diag = startup_active ? diag_event::startup_boost_on
                                  : diag_event::startup_boost_off;
      }
      s.startup_boost_active = startup_active;
    }

    if (startup_active)
    {
      const std::int16_t boost_abs = clamp_i16(cfg.startup_boost_u, 0, 1000);
      if (s.left_ff_u != 0 && abs_i16(left_out) < boost_abs)
      {
        const std::int16_t dir = (left_out != 0) ? sign_i16(left_out) : sign_i16(s.left_ff_u);
        left_out = static_cast<std::int16_t>(dir * boost_abs);
      }
      if (s.right_ff_u != 0 && abs_i16(right_out) < boost_abs)
      {
        const std::int16_t dir = (right_out != 0) ? sign_i16(right_out) : sign_i16(s.right_ff_u);
        right_out = static_cast<std::int16_t>(dir * boost_abs);
      }
    }

    // Re-kick window if we detect stall while command should be moving.
    if (now_ms < s.stall_kick_until_ms)
    {
      const std::int16_t kick_abs = clamp_i16(cfg.stall_kick_u, 0, 1000);
      if (s.left_ff_u != 0 && abs_i16(left_out) < kick_abs)
      {
        const std::int16_t dir = (left_out != 0) ? sign_i16(left_out) : sign_i16(s.left_ff_u);
        left_out = static_cast<std::int16_t>(dir * kick_abs);
      }
      if (s.right_ff_u != 0 && abs_i16(right_out) < kick_abs)
      {
        const std::int16_t dir = (right_out != 0) ? sign_i16(right_out) : sign_i16(s.right_ff_u);
        right_out = static_cast<std::int16_t>(dir * kick_abs);
      }
    }

    out.status = step_status::running;
    out.left_u = left_out;
    out.right_u = right_out;
    out.startup_boost_active = startup_active;
    return out;
  }
}
