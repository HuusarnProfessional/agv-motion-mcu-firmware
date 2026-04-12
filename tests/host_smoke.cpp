#include "core/api/encoder_api.hpp"
#include "core/api/imu_api.hpp"
#include "core/control/controller_robot_future.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>

namespace
{
  std::uint32_t g_simulated_step = 0U;

  constexpr std::uint16_t k_encoder_modulus = 4096U;
  constexpr std::uint16_t k_encoder_step_left = 11U;
  constexpr std::uint16_t k_encoder_step_right = 12U;
  constexpr std::uint32_t k_simulated_tick_period_ms = 10U;

  std::uint16_t make_encoder_angle(std::uint8_t encoder_id)
  {
    const bool is_right_side = (encoder_id == 1U) || (encoder_id == 3U);
    const std::uint32_t step_per_tick = is_right_side ? k_encoder_step_right : k_encoder_step_left;
    const std::uint32_t phase_offset = static_cast<std::uint32_t>(encoder_id) * 73U;
    return static_cast<std::uint16_t>((g_simulated_step * step_per_tick + phase_offset) % k_encoder_modulus);
  }
}

namespace encoder_api
{
  void init(const encoder_input *encoders, std::size_t count)
  {
    (void)encoders;
    (void)count;
  }

  bool read_sample(std::uint8_t encoder_id, encoder_sample &out)
  {
    if (encoder_id >= 4U)
    {
      out = {};
      out.status = encoder_status::invalid_id;
      return false;
    }

    out = {};
    out.angle_raw_12bit = make_encoder_angle(encoder_id);
    out.time_ms = g_simulated_step * k_simulated_tick_period_ms;
    out.status = encoder_status::ok;
    return true;
  }
}

namespace imu_api
{
  void init(const imu_input *imus, std::size_t count)
  {
    (void)imus;
    (void)count;
  }

  bool read_sample(std::uint8_t imu_id, imu_sample &out)
  {
    if (imu_id != 0U)
    {
      out = {};
      out.gyroscope_state = gyroscope_status::invalid_sample;
      out.accelerometer_state = accelerometer_status::invalid_sample;
      out.magnetometer_state = magnetometer_status::invalid_sample;
      return false;
    }

    out = {};
    out.gyroscope_state = gyroscope_status::ok;
    out.accelerometer_state = accelerometer_status::ok;
    out.magnetometer_state = magnetometer_status::ok;
    out.gyroscope_z_mdps = 3500;
    out.gyroscope_z_calibrated_mdps = 3500;
    out.accelerometer_x_mg = 80;
    out.accelerometer_y_mg = 0;
    out.accelerometer_z_mg = 1000;
    out.accelerometer_x_calibrated_mg = 80;
    out.accelerometer_y_calibrated_mg = 0;
    out.accelerometer_z_calibrated_mg = 1000;
    out.time_ms = g_simulated_step * k_simulated_tick_period_ms;
    return true;
  }

  void set_calibration(std::uint8_t imu_id, const imu_calibration_profile &profile)
  {
    (void)imu_id;
    (void)profile;
  }

  bool get_calibration(std::uint8_t imu_id, imu_calibration_profile &out)
  {
    (void)imu_id;
    out = {};
    return false;
  }

  void clear_calibration(std::uint8_t imu_id)
  {
    (void)imu_id;
  }
}

int main(void)
{
  constexpr std::uint32_t k_warmup_ticks = 1000U;
  constexpr std::uint32_t k_measured_ticks = 20000U;

  controller_robot_future::init();

  std::uint32_t now_ms = 0U;

  for (std::uint32_t index = 0U; index < k_warmup_ticks; ++index)
  {
    g_simulated_step = index;
    now_ms = index * k_simulated_tick_period_ms;
    controller_robot_future::tick(now_ms);
  }

  const auto start_time = std::chrono::steady_clock::now();

  for (std::uint32_t index = 0U; index < k_measured_ticks; ++index)
  {
    const std::uint32_t simulated_index = k_warmup_ticks + index;
    g_simulated_step = simulated_index;
    now_ms = simulated_index * k_simulated_tick_period_ms;
    controller_robot_future::tick(now_ms);
  }

  const auto end_time = std::chrono::steady_clock::now();
  const auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  const double average_tick_duration_us = static_cast<double>(total_duration) / static_cast<double>(k_measured_ticks);

  local_positioning::snapshot snapshot = {};
  const bool has_pose = controller_robot_future::read_local_positioning_snapshot(snapshot);

  std::printf("future_tick_benchmark\n");
  std::printf("measured_ticks=%u\n", k_measured_ticks);
  std::printf("simulated_tick_period_ms=%u\n", k_simulated_tick_period_ms);
  std::printf("total_host_time_us=%lld\n", static_cast<long long>(total_duration));
  std::printf("average_host_time_per_tick_us=%.3f\n", average_tick_duration_us);
  std::printf("has_pose=%s\n", has_pose ? "true" : "false");

  if (has_pose)
  {
    std::printf("pose_x_um=%lld\n", static_cast<long long>(snapshot.x_um));
    std::printf("pose_y_um=%lld\n", static_cast<long long>(snapshot.y_um));
    std::printf("pose_heading_urad=%ld\n", static_cast<long>(snapshot.heading_urad));
    std::printf("confidence_position=%u\n", snapshot.confidence_position);
    std::printf("confidence_heading=%u\n", snapshot.confidence_heading);
  }

  return 0;
}
