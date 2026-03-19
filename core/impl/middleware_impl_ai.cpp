#include "core/impl/middleware_impl_ai.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace middleware_impl_ai
{
  namespace
  {
    using middleware_api_ai::dispatch_ops;
    using middleware_api_ai::handle_status;
    using middleware_api_ai::tx_ops;

    static bool tx_line(const tx_ops &tx, const char *line)
    {
      if (!tx.send_line)
      {
        return false;
      }
      return tx.send_line(tx.tx_ctx, line);
    }

    static handle_status send_ok(const tx_ops &tx)
    {
      return tx_line(tx, "ok") ? handle_status::ok : handle_status::err_handler;
    }

    static handle_status send_err(const tx_ops &tx, const char *reason, handle_status status_code)
    {
      char msg[32];
      std::snprintf(msg, sizeof(msg), "err %s", reason);
      (void)tx_line(tx, msg);
      return status_code;
    }

    static void to_printable_ascii(const char *in, char *out, std::size_t out_cap)
    {
      if (!out || out_cap == 0U)
      {
        return;
      }

      std::size_t w = 0U;
      if (in != nullptr)
      {
        for (std::size_t i = 0U; in[i] != '\0' && (w + 1U) < out_cap; ++i)
        {
          const unsigned char c = static_cast<unsigned char>(in[i]);
          if (c >= 32U && c <= 126U)
          {
            out[w++] = static_cast<char>(c);
          }
          else
          {
            out[w++] = '.';
          }
        }
      }
      out[w] = '\0';
    }

    static void send_dbg_bad_format(const tx_ops &tx, const char *raw_line)
    {
      char clean[56];
      to_printable_ascii(raw_line, clean, sizeof(clean));

      char msg[96];
      std::snprintf(msg, sizeof(msg), "status dbg_bad_format %s", clean);
      (void)tx_line(tx, msg);
    }

    static char *trim_ascii_inplace(char *s)
    {
      while (*s == ' ' || *s == '\t')
      {
        ++s;
      }

      std::size_t len = std::strlen(s);
      while (len > 0U)
      {
        const char c = s[len - 1U];
        if (c != ' ' && c != '\t')
        {
          break;
        }
        s[len - 1U] = '\0';
        --len;
      }

      return s;
    }

    static void to_lower_ascii(char *s)
    {
      while (*s)
      {
        if (*s >= 'A' && *s <= 'Z')
        {
          *s = static_cast<char>(*s - 'A' + 'a');
        }
        ++s;
      }
    }

    static bool parse_long_arg(const char *s, std::int32_t &out)
    {
      if (!s || !*s)
      {
        return false;
      }

      char *endp = nullptr;
      const long v = std::strtol(s, &endp, 10);
      if (endp == s || *endp != '\0')
      {
        return false;
      }

      out = static_cast<std::int32_t>(v);
      return true;
    }

    static bool parse_bool_arg(const char *s, bool &out)
    {
      if (!s)
      {
        return false;
      }

      if (std::strcmp(s, "1") == 0 || std::strcmp(s, "true") == 0 || std::strcmp(s, "on") == 0)
      {
        out = true;
        return true;
      }

      if (std::strcmp(s, "0") == 0 || std::strcmp(s, "false") == 0 || std::strcmp(s, "off") == 0)
      {
        out = false;
        return true;
      }

      return false;
    }

    struct parsed_call
    {
      char *cmd;
      char *arg_blob;
      char *tok[10];
      std::uint8_t tok_n;
    };

    static bool parse_function_call(char *line, parsed_call &out)
    {
      out.cmd = nullptr;
      out.arg_blob = nullptr;
      out.tok_n = 0U;

      char *open = std::strchr(line, '(');
      char *close = std::strrchr(line, ')');
      if (!open || !close || close < open)
      {
        return false;
      }

      char *tail = close + 1;
      while (*tail == ' ' || *tail == '\t')
      {
        ++tail;
      }
      if (*tail != '\0')
      {
        return false;
      }

      if (std::strchr(open + 1, '(') != nullptr)
      {
        return false;
      }

      char *first_close = std::strchr(open + 1, ')');
      if (!first_close || first_close != close)
      {
        return false;
      }

      *open = '\0';
      char *cmd = trim_ascii_inplace(line);
      if (*cmd == '\0')
      {
        return false;
      }
      if (std::strchr(cmd, ' ') || std::strchr(cmd, '\t'))
      {
        return false;
      }

      for (char *p = cmd; *p; ++p)
      {
        const bool ok = ((*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || (*p == '_'));
        if (!ok)
        {
          return false;
        }
      }

      *close = '\0';
      char *args = trim_ascii_inplace(open + 1);
      if (std::strchr(args, '(') || std::strchr(args, ')'))
      {
        return false;
      }

      out.cmd = cmd;
      out.arg_blob = args;
      out.tok[0] = cmd;
      out.tok_n = 1U;

      if (*args == '\0')
      {
        return true;
      }

      char *seg = args;
      for (char *p = args;; ++p)
      {
        if (*p == ',' || *p == '\0')
        {
          const char saved = *p;
          *p = '\0';

          char *part = trim_ascii_inplace(seg);
          if (*part == '\0')
          {
            return false;
          }
          if (std::strchr(part, ' ') || std::strchr(part, '\t'))
          {
            return false;
          }
          if (out.tok_n >= sizeof(out.tok) / sizeof(out.tok[0]))
          {
            return false;
          }

          out.tok[out.tok_n++] = part;

          if (saved == '\0')
          {
            break;
          }

          seg = p + 1;
        }
      }

      return true;
    }

    static bool call_getter(bool (*fn)(void *), void *ctx)
    {
      if (!fn)
      {
        return false;
      }
      return fn(ctx);
    }

    static bool call_setter_i32(bool (*fn)(void *, std::int32_t), void *ctx, std::int32_t v)
    {
      if (!fn)
      {
        return false;
      }
      return fn(ctx, v);
    }

    static bool call_setter_i32_i32(bool (*fn)(void *, std::int32_t, std::int32_t),
                                    void *ctx,
                                    std::int32_t a,
                                    std::int32_t b)
    {
      if (!fn)
      {
        return false;
      }
      return fn(ctx, a, b);
    }

    static bool call_setter_i32_i32_i32_i32(bool (*fn)(void *,
                                                       std::int32_t,
                                                       std::int32_t,
                                                       std::int32_t,
                                                       std::int32_t),
                                            void *ctx,
                                            std::int32_t a,
                                            std::int32_t b,
                                            std::int32_t c,
                                            std::int32_t d)
    {
      if (!fn)
      {
        return false;
      }
      return fn(ctx, a, b, c, d);
    }

    static handle_status handle_line_impl(const char *raw_line,
                                          const dispatch_ops &dispatch,
                                          void *app_ctx,
                                          const tx_ops &tx)
    {
      if (!raw_line)
      {
        return send_err(tx, "bad_format", handle_status::invalid_arg);
      }

      char buf[160];
      std::size_t n = 0U;

      while (raw_line[n] != '\0' && n < sizeof(buf) - 1U)
      {
        buf[n] = raw_line[n];
        ++n;
      }
      buf[n] = '\0';

      char *line = trim_ascii_inplace(buf);
      if (*line == '\0')
      {
        return send_err(tx, "empty", handle_status::err_bad_format);
      }

      to_lower_ascii(line);

      parsed_call call{};
      if (!parse_function_call(line, call))
      {
        send_dbg_bad_format(tx, raw_line);
        return send_err(tx, "bad_format", handle_status::err_bad_format);
      }

      const char *cmd = call.cmd;

      if (std::strcmp(cmd, "ping") == 0)
      {
        if (!dispatch.on_ping || !dispatch.on_ping(app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "ping_mission") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        if (!dispatch.on_ping || !dispatch.on_ping(app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }

        return tx_line(tx, "stm32_pong") ? handle_status::ok : handle_status::err_handler;
      }

      if (std::strcmp(cmd, "get_speed") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_speed, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_distance") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_distance, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_pose_x") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_pose_x, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_pose_y") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_pose_y, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_wheel_diameter_mm") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_wheel_diameter_mm, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_wheel_separation_mm") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_wheel_separation_mm, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_yaw_effective_track_mm") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_yaw_effective_track_mm, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_arc_effective_track_mm") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_arc_effective_track_mm, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_motor_id_map") == 0 ||
          std::strcmp(cmd, "get_motor_map") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_motor_id_map, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_id_map") == 0 ||
          std::strcmp(cmd, "get_encoder_ids") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_encoder_id_map, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_dir_map") == 0 ||
          std::strcmp(cmd, "get_encoder_dirs") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_encoder_dir_map, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "set_speed") == 0)
      {
        std::int32_t speed = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], speed))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!dispatch.on_set_speed || !dispatch.on_set_speed(app_ctx, speed))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_distance_reset") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_set_distance_reset, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_pose_reset") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_set_pose_reset, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_wheel_diameter_mm") == 0)
      {
        std::int32_t mm = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], mm))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_wheel_diameter_mm, app_ctx, mm))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_wheel_separation_mm") == 0 ||
          std::strcmp(cmd, "set_wheel_base_mm") == 0)
      {
        std::int32_t mm = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], mm))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_wheel_separation_mm, app_ctx, mm))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_yaw_effective_track_mm") == 0)
      {
        std::int32_t mm = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], mm))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_yaw_effective_track_mm, app_ctx, mm))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_arc_effective_track_mm") == 0)
      {
        std::int32_t mm = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], mm))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_arc_effective_track_mm, app_ctx, mm))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_motor_id_map") == 0 ||
          std::strcmp(cmd, "set_motor_map") == 0)
      {
        std::int32_t front_left_id = 0;
        std::int32_t front_right_id = 0;
        std::int32_t rear_left_id = 0;
        std::int32_t rear_right_id = 0;
        if (call.tok_n != 5U ||
            !parse_long_arg(call.tok[1], front_left_id) ||
            !parse_long_arg(call.tok[2], front_right_id) ||
            !parse_long_arg(call.tok[3], rear_left_id) ||
            !parse_long_arg(call.tok[4], rear_right_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32_i32_i32_i32(dispatch.on_set_motor_id_map,
                                         app_ctx,
                                         front_left_id,
                                         front_right_id,
                                         rear_left_id,
                                         rear_right_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_motor_u") == 0 ||
          std::strcmp(cmd, "set_motor_pct") == 0)
      {
        std::int32_t motor_id = 0;
        std::int32_t pct = 0;
        if (call.tok_n != 3U ||
            !parse_long_arg(call.tok[1], motor_id) ||
            !parse_long_arg(call.tok[2], pct))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32_i32(dispatch.on_set_motor_u, app_ctx, motor_id, pct))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_encoder_id_map") == 0 ||
          std::strcmp(cmd, "set_encoder_ids") == 0)
      {
        std::int32_t left_id = 0;
        std::int32_t right_id = 0;
        if (call.tok_n != 3U ||
            !parse_long_arg(call.tok[1], left_id) ||
            !parse_long_arg(call.tok[2], right_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        if (!dispatch.on_set_encoder_id_map ||
            !dispatch.on_set_encoder_id_map(app_ctx, left_id, right_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_encoder_dir_map") == 0 ||
          std::strcmp(cmd, "set_encoder_dirs") == 0)
      {
        std::int32_t left_dir = 0;
        std::int32_t right_dir = 0;
        if (call.tok_n != 3U ||
            !parse_long_arg(call.tok[1], left_dir) ||
            !parse_long_arg(call.tok[2], right_dir))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        if (!dispatch.on_set_encoder_dir_map ||
            !dispatch.on_set_encoder_dir_map(app_ctx, left_dir, right_dir))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "get_position") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_position, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "set_position") == 0)
      {
        std::int32_t x = 0;
        std::int32_t y = 0;
        std::int32_t heading = 0;
        if (call.tok_n != 4U ||
            !parse_long_arg(call.tok[1], x) ||
            !parse_long_arg(call.tok[2], y) ||
            !parse_long_arg(call.tok[3], heading))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        if (!dispatch.on_set_position || !dispatch.on_set_position(app_ctx, x, y, heading))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "get_angle") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_angle, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_angle_deg") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        bool ok = false;
        if (dispatch.on_get_angle_deg)
        {
          ok = dispatch.on_get_angle_deg(app_ctx);
        }
        else
        {
          ok = call_getter(dispatch.on_get_angle, app_ctx);
        }
        if (!ok)
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "set_angle_reset") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        bool ok = false;
        if (dispatch.on_set_angle_reset)
        {
          ok = dispatch.on_set_angle_reset(app_ctx);
        }
        else if (dispatch.on_set_drive_angle)
        {
          ok = dispatch.on_set_drive_angle(app_ctx, 0);
        }
        if (!ok)
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_angle") == 0)
      {
        std::int32_t heading = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], heading))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!dispatch.on_set_drive_angle || !dispatch.on_set_drive_angle(app_ctx, heading))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "get_time") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_time, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_status") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_status, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_stop") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_stop, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_uart_diag") == 0 ||
          std::strcmp(cmd, "diag_uart") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_uart_diag, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_status_uwb") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_status_uwb, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_uwb_raw") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_uwb_raw, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_raw") == 0)
      {
        std::int32_t encoder_id = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], encoder_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_get_encoder_raw, app_ctx, encoder_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_deg") == 0)
      {
        std::int32_t encoder_id = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], encoder_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_get_encoder_deg, app_ctx, encoder_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_rad") == 0)
      {
        std::int32_t encoder_id = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], encoder_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_get_encoder_rad, app_ctx, encoder_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_time") == 0 ||
          std::strcmp(cmd, "get_encoder_time_ms") == 0)
      {
        std::int32_t encoder_id = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], encoder_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_get_encoder_time, app_ctx, encoder_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_encoder_status") == 0)
      {
        std::int32_t encoder_id = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], encoder_id))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_get_encoder_status, app_ctx, encoder_id))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "get_pwm") == 0)
      {
        if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_getter(dispatch.on_get_pwm_pct, app_ctx))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return handle_status::ok;
      }

      if (std::strcmp(cmd, "set_stop") == 0)
      {
        bool stop = false;
        if (call.tok_n != 2U || !parse_bool_arg(call.tok[1], stop))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!dispatch.on_set_stop || !dispatch.on_set_stop(app_ctx, stop))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_shutdown") == 0)
      {
        bool shutdown = true;
        if (call.tok_n == 2U)
        {
          if (!parse_bool_arg(call.tok[1], shutdown))
          {
            return send_err(tx, "bad_format", handle_status::err_bad_format);
          }
        }
        else if (call.tok_n != 1U)
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        if (!dispatch.on_set_shutdown || !dispatch.on_set_shutdown(app_ctx, shutdown))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_pwm") == 0)
      {
        std::int32_t pct = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], pct))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_pwm_pct, app_ctx, pct))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_forward") == 0 ||
          std::strcmp(cmd, "set_drive_forwards") == 0 ||
          std::strcmp(cmd, "set_drive_forward_mm") == 0)
      {
        std::int32_t mm = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], mm))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        bool ok = false;
        if (dispatch.on_set_drive_forward_mm)
        {
          ok = dispatch.on_set_drive_forward_mm(app_ctx, mm);
        }
        else if (dispatch.on_set_drive_forward)
        {
          ok = dispatch.on_set_drive_forward(app_ctx, mm);
        }

        if (!ok)
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_backward") == 0 ||
          std::strcmp(cmd, "set_drive_backwards") == 0 ||
          std::strcmp(cmd, "set_drive_backwards_mm") == 0 ||
          std::strcmp(cmd, "set_drive_backward_mm") == 0)
      {
        std::int32_t mm = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], mm))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }

        bool ok = false;
        if (dispatch.on_set_drive_backward_mm)
        {
          ok = dispatch.on_set_drive_backward_mm(app_ctx, mm);
        }
        else if (dispatch.on_set_drive_backward)
        {
          ok = dispatch.on_set_drive_backward(app_ctx, mm);
        }

        if (!ok)
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_arc_mm_deg") == 0 ||
          std::strcmp(cmd, "set_drive_arc") == 0)
      {
        std::int32_t radius_mm = 0;
        std::int32_t angle_deg = 0;
        if (call.tok_n != 3U ||
            !parse_long_arg(call.tok[1], radius_mm) ||
            !parse_long_arg(call.tok[2], angle_deg))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!dispatch.on_set_drive_arc_mm_deg ||
            !dispatch.on_set_drive_arc_mm_deg(app_ctx, radius_mm, angle_deg))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_arc_v2_mm_deg") == 0 ||
          std::strcmp(cmd, "set_drive_arc_v2") == 0)
      {
        std::int32_t radius_mm = 0;
        std::int32_t angle_deg = 0;
        if (call.tok_n != 3U ||
            !parse_long_arg(call.tok[1], radius_mm) ||
            !parse_long_arg(call.tok[2], angle_deg))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!dispatch.on_set_drive_arc_v2_mm_deg ||
            !dispatch.on_set_drive_arc_v2_mm_deg(app_ctx, radius_mm, angle_deg))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_forward_ms") == 0)
      {
        std::int32_t ms = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], ms))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_drive_forward_ms, app_ctx, ms))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_drive_backward_ms") == 0 ||
          std::strcmp(cmd, "set_drive_backwards_ms") == 0)
      {
        std::int32_t ms = 0;
        if (call.tok_n != 2U || !parse_long_arg(call.tok[1], ms))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!call_setter_i32(dispatch.on_set_drive_backward_ms, app_ctx, ms))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_slot") == 0)
      {
        bool occupied = false;
        if (call.tok_n != 3U || !parse_bool_arg(call.tok[2], occupied))
        {
          return send_err(tx, "bad_format", handle_status::err_bad_format);
        }
        if (!dispatch.on_set_slot || !dispatch.on_set_slot(app_ctx, call.tok[1], occupied))
        {
          return send_err(tx, "handler", handle_status::err_handler);
        }
        return send_ok(tx);
      }

      if (std::strcmp(cmd, "set_mission_begin") == 0 ||
          std::strcmp(cmd, "set_mission_point") == 0 ||
          std::strcmp(cmd, "set_mission_end") == 0 ||
          std::strcmp(cmd, "start_mission") == 0 ||
          std::strcmp(cmd, "stop_mission") == 0)
      {
        if (dispatch.on_mission_stub)
        {
          if (!dispatch.on_mission_stub(app_ctx, cmd, call.arg_blob))
          {
            return send_err(tx, "handler", handle_status::err_handler);
          }
        }
        return send_ok(tx);
      }

      return send_err(tx, "unknown", handle_status::err_unknown);
    }

    const middleware_api_ai::impl_ops k_impl_ops =
    {
      handle_line_impl
    };
  }

  const middleware_api_ai::impl_ops *get_impl_ops(void)
  {
    return &k_impl_ops;
  }
}

