#include "core/control/controller_esp32_wroom32_demo.hpp"
#include "core/api/comm_uart_api.hpp"
#include "core/api/encoder_api.hpp"
#include "core/api/motor_api.hpp"
#include "main.h"
#include <cstdlib>


extern "C"
{
  uint8_t g_pick_controller = 0U;
}

namespace controller_esp32_wroom32_demo
{
  static uint8_t read_encoder_raw_all(uint16_t raw_out[4], bool valid_out[4]);
  static int32_t encoder_delta_and_rollover_check(uint16_t prev_raw, uint16_t curr_raw);
  static void handle_forward_command_line(const char *command_line);

  void init(void)
  {
    // keep motor output 0
    for (uint8_t id = 0U; id < 4U; ++id)
    {
      motor_api::set_u(id, 0);
    }
  }

  static void handle_forward_command_line(const char *command_line)
  {
    if (command_line[0] != 'F' || command_line[1] != ':')
    {
      return;
    }

    int32_t mm = static_cast<int32_t>(std::atoi(&command_line[2]));
    if (mm > 0)
    {
      set_drive_forward_mm(mm);
    }
  }

  void tick(uint32_t now_ms) // drain all available UART bytes each tick
  {
    (void)now_ms;

    static char command_line[20] = {0};
    static uint8_t command_line_length = 0U;

    uint8_t received_byte = 0U;

    while (comm_uart_api::read_bytes(0U, &received_byte, 1U) == 1U)
    {
      if (received_byte == static_cast<uint8_t>('\r'))
      {
        continue;
      }

      if (received_byte == static_cast<uint8_t>('\n'))
      {
        command_line[command_line_length] = '\0';
        handle_forward_command_line(command_line);
        command_line_length = 0U;
        continue;
      }

      if (command_line_length < (sizeof(command_line) - 1U))
      {
        command_line[command_line_length] = static_cast<char>(received_byte);
        ++command_line_length;
      }
      else
      {
        command_line_length = 0U;
      }
    }
  }

  void set_drive_forward_mm(int32_t mm_target)
  {
    if (mm_target <= 0)
    {
      return;
    }

    uint16_t raw_prev[4] = {0, 0, 0, 0};
    bool valid_prev[4] = {false, false, false, false};
    if (read_encoder_raw_all(raw_prev, valid_prev) < 2U)
    {
      return;
    }

    int64_t distance_mm_x1000 = 0;
    const int64_t target_mm_x1000 = static_cast<int64_t>(mm_target) * 1000;

    for (uint8_t id = 0U; id < 4U; ++id)
    {
      motor_api::set_u(id, 300);
    }

    const uint32_t start_ms = HAL_GetTick();
    const uint32_t timeout_ms = 12000U;

    while (distance_mm_x1000 < target_mm_x1000)
    {
      if ((HAL_GetTick() - start_ms) > timeout_ms)
      {
        break;
      }

      uint16_t raw_now[4] = {0, 0, 0, 0};
      bool valid_now[4] = {false, false, false, false};
      const uint8_t valid_now_count = read_encoder_raw_all(raw_now, valid_now);
      if (valid_now_count < 2U)
      {
        for (uint8_t i = 0U; i < 4U; ++i)
        {
          if (valid_now[i])
          {
            raw_prev[i] = raw_now[i];
          }
          valid_prev[i] = valid_now[i];
        }
        continue;
      }

      int32_t d_sum_raw = 0;
      uint8_t d_count = 0U;
      for (uint8_t i = 0U; i < 4U; ++i)
      {
        if (valid_prev[i] && valid_now[i])
        {
          d_sum_raw += encoder_delta_and_rollover_check(raw_prev[i], raw_now[i]);
          ++d_count;
        }
      }

      if (d_count >= 2U)
      {
        const int32_t d_avg_raw = d_sum_raw / static_cast<int32_t>(d_count);
        distance_mm_x1000 += (static_cast<int64_t>(d_avg_raw) * 63 * 3142) / 4096;
      }

      for (uint8_t i = 0U; i < 4U; ++i)
      {
        if (valid_now[i])
        {
          raw_prev[i] = raw_now[i];
        }
        valid_prev[i] = valid_now[i];
      }
    }

    for (uint8_t id = 0U; id < 4U; ++id)
    {
      motor_api::set_u(id, 0);
    }
  }

  static uint8_t read_encoder_raw_all(uint16_t raw_out[4], bool valid_out[4])
  {
    uint8_t valid_count = 0U;

    for (uint8_t id = 0U; id < 4U; ++id)
    {
      valid_out[id] = false;
      raw_out[id] = 0U;

      encoder_api::encoder_sample sample{};
      if (!encoder_api::read_sample(id, sample))
      {
        continue;
      }

      if (sample.status != encoder_api::encoder_status::ok &&
          sample.status != encoder_api::encoder_status::stale)
      {
        continue;
      }

      raw_out[id] = sample.angle_raw_12bit;
      valid_out[id] = true;
      ++valid_count;
    }

    return valid_count;
  }

  static int32_t encoder_delta_and_rollover_check(uint16_t prev_raw, uint16_t curr_raw)
  {
    int32_t delta = static_cast<int32_t>(curr_raw) - static_cast<int32_t>(prev_raw);

    // Check for jumps between 0 and 4095 in both directions.
    if (delta > 2048) // 12-bit: 4096 / 2
    {
      delta -= 4096;
    }
    else if (delta < -2048)
    {
      delta += 4096;
    }

    return delta;
  }
}
