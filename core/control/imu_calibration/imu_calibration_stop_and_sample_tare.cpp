#include "core/control/imu_calibration/imu_calibration_stop_and_sample_tare.hpp"
#include "core/control/imu_calibration/imu_calibration_tuning.hpp"

#include "core/api/imu_api.hpp"
#include "core/api/motor_api.hpp"
#include "core/control/local_positioning/encoder_model/encoder_model_pipeline.hpp"
#include <algorithm>
#include <cmath>

namespace
{
  constexpr std::uint8_t k_motor_count = 4u;
  constexpr std::int64_t k_tare_sample_count = 60000LL;
  constexpr std::int64_t k_tare_samples_per_tick = 100LL;

  double get_noise_quantile()
  {
    return static_cast<double>(imu_calibration_tuning::k_noise_p_proc_percent) / 100.0;
  }

  void initialize_p_proc_estimator(imu_noise_p_proc_estimator_state &state)
  {
    std::sort(state.initial_samples.begin(), state.initial_samples.end());

    for (std::size_t i = 0u; i < state.marker_heights.size(); ++i)
    {
      state.marker_heights[i] = state.initial_samples[i];
      state.marker_positions[i] = static_cast<std::int64_t>(i + 1u);
    }

    const double quantile = get_noise_quantile();
    state.desired_positions[0] = 1.0;
    state.desired_positions[1] = 1.0 + (2.0 * quantile);
    state.desired_positions[2] = 1.0 + (4.0 * quantile);
    state.desired_positions[3] = 3.0 + (2.0 * quantile);
    state.desired_positions[4] = 5.0;

    state.desired_increments[0] = 0.0;
    state.desired_increments[1] = quantile / 2.0;
    state.desired_increments[2] = quantile;
    state.desired_increments[3] = (1.0 + quantile) / 2.0;
    state.desired_increments[4] = 1.0;

    state.is_initialized = true;
  }

  void add_sample_to_p_proc_estimator(imu_noise_p_proc_estimator_state &state, std::int32_t sample)
  {
    const double sample_value = static_cast<double>(sample);

    if (!state.is_initialized)
    {
      state.initial_samples[state.initial_sample_count] = sample_value;
      ++state.initial_sample_count;

      if (state.initial_sample_count == 5u)
      {
        initialize_p_proc_estimator(state);
      }
      return;
    }

    std::size_t marker_bucket = 0u;

    if (sample_value < state.marker_heights[0])
    {
      state.marker_heights[0] = sample_value;
      marker_bucket = 0u;
    }
    else if (sample_value < state.marker_heights[1])
    {
      marker_bucket = 0u;
    }
    else if (sample_value < state.marker_heights[2])
    {
      marker_bucket = 1u;
    }
    else if (sample_value < state.marker_heights[3])
    {
      marker_bucket = 2u;
    }
    else if (sample_value <= state.marker_heights[4])
    {
      marker_bucket = 3u;
    }
    else
    {
      state.marker_heights[4] = sample_value;
      marker_bucket = 3u;
    }

    for (std::size_t i = marker_bucket + 1u; i < state.marker_positions.size(); ++i)
    {
      ++state.marker_positions[i];
    }

    for (std::size_t i = 0u; i < state.desired_positions.size(); ++i)
    {
      state.desired_positions[i] += state.desired_increments[i];
    }

    for (std::size_t i = 1u; i < 4u; ++i)
    {
      const double desired_delta = state.desired_positions[i] - static_cast<double>(state.marker_positions[i]);

      if (desired_delta >= 1.0 &&
          (state.marker_positions[i + 1u] - state.marker_positions[i]) > 1)
      {
        const double marker_delta = 1.0;
        const double left_position = static_cast<double>(state.marker_positions[i - 1u]);
        const double current_position = static_cast<double>(state.marker_positions[i]);
        const double right_position = static_cast<double>(state.marker_positions[i + 1u]);
        const double left_height = state.marker_heights[i - 1u];
        const double current_height = state.marker_heights[i];
        const double right_height = state.marker_heights[i + 1u];

        const double parabolic_height =
            current_height +
            (marker_delta / (right_position - left_position)) *
                (((current_position - left_position + marker_delta) * (right_height - current_height) / (right_position - current_position)) +
                 ((right_position - current_position - marker_delta) * (current_height - left_height) / (current_position - left_position)));

        if (left_height < parabolic_height && parabolic_height < right_height)
        {
          state.marker_heights[i] = parabolic_height;
        }
        else
        {
          state.marker_heights[i] =
              current_height +
              ((state.marker_heights[i + 1u] - current_height) / (right_position - current_position));
        }

        ++state.marker_positions[i];
      }
      else if (desired_delta <= -1.0 &&
               (state.marker_positions[i - 1u] - state.marker_positions[i]) < -1)
      {
        const double marker_delta = -1.0;
        const double left_position = static_cast<double>(state.marker_positions[i - 1u]);
        const double current_position = static_cast<double>(state.marker_positions[i]);
        const double right_position = static_cast<double>(state.marker_positions[i + 1u]);
        const double left_height = state.marker_heights[i - 1u];
        const double current_height = state.marker_heights[i];
        const double right_height = state.marker_heights[i + 1u];

        const double parabolic_height =
            current_height +
            (marker_delta / (right_position - left_position)) *
                (((current_position - left_position + marker_delta) * (right_height - current_height) / (right_position - current_position)) +
                 ((right_position - current_position - marker_delta) * (current_height - left_height) / (current_position - left_position)));

        if (left_height < parabolic_height && parabolic_height < right_height)
        {
          state.marker_heights[i] = parabolic_height;
        }
        else
        {
          state.marker_heights[i] =
              current_height -
              ((state.marker_heights[i - 1u] - current_height) / (left_position - current_position));
        }

        --state.marker_positions[i];
      }
    }
  }

  std::int32_t get_p_proc_value(const imu_noise_p_proc_estimator_state &state)
  {
    if (!state.is_initialized)
    {
      if (state.initial_sample_count == 0u)
      {
        return 0;
      }

      std::array<double, 5u> sorted_samples = state.initial_samples;
      std::sort(sorted_samples.begin(), sorted_samples.begin() + state.initial_sample_count);
      const double quantile = get_noise_quantile();
      const double position = quantile * static_cast<double>(state.initial_sample_count - 1u);
      const std::size_t index = static_cast<std::size_t>(position);
      return static_cast<std::int32_t>(sorted_samples[index]);
    }

    return static_cast<std::int32_t>(state.marker_heights[2u]);
  }

  void update_noise_profile_estimator(imu_tare_step_state &tare_step_state, const imu_api::imu_sample &sample)
  {
    const std::int32_t gyroscope_x_mean = static_cast<std::int32_t>(tare_step_state.gyroscope_x_sum / tare_step_state.valid_sample_count);
    const std::int32_t gyroscope_y_mean = static_cast<std::int32_t>(tare_step_state.gyroscope_y_sum / tare_step_state.valid_sample_count);
    const std::int32_t gyroscope_z_mean = static_cast<std::int32_t>(tare_step_state.gyroscope_z_sum / tare_step_state.valid_sample_count);
    const std::int32_t accelerometer_x_mean = static_cast<std::int32_t>(tare_step_state.accelerometer_x_sum / tare_step_state.valid_sample_count);
    const std::int32_t accelerometer_y_mean = static_cast<std::int32_t>(tare_step_state.accelerometer_y_sum / tare_step_state.valid_sample_count);
    const std::int32_t accelerometer_z_mean = static_cast<std::int32_t>(tare_step_state.accelerometer_z_sum / tare_step_state.valid_sample_count);

    add_sample_to_p_proc_estimator(tare_step_state.gyroscope_x_noise_state, std::abs(sample.gyroscope_x_mdps - gyroscope_x_mean));
    add_sample_to_p_proc_estimator(tare_step_state.gyroscope_y_noise_state, std::abs(sample.gyroscope_y_mdps - gyroscope_y_mean));
    add_sample_to_p_proc_estimator(tare_step_state.gyroscope_z_noise_state, std::abs(sample.gyroscope_z_mdps - gyroscope_z_mean));
    add_sample_to_p_proc_estimator(tare_step_state.accelerometer_x_noise_state, std::abs(sample.accelerometer_x_mg - accelerometer_x_mean));
    add_sample_to_p_proc_estimator(tare_step_state.accelerometer_y_noise_state, std::abs(sample.accelerometer_y_mg - accelerometer_y_mean));
    add_sample_to_p_proc_estimator(tare_step_state.accelerometer_z_noise_state, std::abs(sample.accelerometer_z_mg - accelerometer_z_mean));
  }
}

void stop_motor()
{
  for (std::uint8_t motor_id = 0u; motor_id < k_motor_count; ++motor_id)
  {
    motor_api::set_u(motor_id, 0);
  }
}

bool is_still_from_encoder_model(const local_positioning::state &encoder_model_state)
{
  const motion_model_encoders::motion_model_snapshot &motion = encoder_model_state.encoder_motion_snapshot;

  if (!motion.has_motion_model)
  {
    return false;
  }

  if (std::abs(motion.translation) > imu_calibration_tuning::k_still_translation_limit_um)
  {
    return false;
  }

  if (std::abs(motion.rotation) > imu_calibration_tuning::k_still_rotation_limit_urad)
  {
    return false;
  }

  return true;
}

bool tick_build_tare_values(imu_tare_step_state &tare_step_state, std::uint8_t imu_id, imu_api::imu_tare_values &out_tare, imu_api::imu_noise_profile &out_noise)
{
  if (tare_step_state.done)
  {
    return true;
  }

  for (std::int64_t i = 0; i < k_tare_samples_per_tick; ++i)
  {
    if (tare_step_state.valid_sample_count >= k_tare_sample_count)
    {
      break;
    }

    imu_api::imu_sample sample = {};
    if (!imu_api::read_sample(imu_id, sample))
    {
      continue;
    }

    tare_step_state.gyroscope_x_sum += sample.gyroscope_x_mdps;
    tare_step_state.gyroscope_y_sum += sample.gyroscope_y_mdps;
    tare_step_state.gyroscope_z_sum += sample.gyroscope_z_mdps;
    tare_step_state.accelerometer_x_sum += sample.accelerometer_x_mg;
    tare_step_state.accelerometer_y_sum += sample.accelerometer_y_mg;
    tare_step_state.accelerometer_z_sum += sample.accelerometer_z_mg;
    tare_step_state.magnetometer_x_sum += sample.magnetometer_x_mgauss;
    tare_step_state.magnetometer_y_sum += sample.magnetometer_y_mgauss;
    tare_step_state.magnetometer_z_sum += sample.magnetometer_z_mgauss;
    ++tare_step_state.valid_sample_count;
    update_noise_profile_estimator(tare_step_state, sample);
  }

  if (tare_step_state.valid_sample_count < k_tare_sample_count)
  {
    return false;
  }

  out_tare = {};
  out_tare.gyroscope_x_mdps = static_cast<std::int32_t>(tare_step_state.gyroscope_x_sum / tare_step_state.valid_sample_count);
  out_tare.gyroscope_y_mdps = static_cast<std::int32_t>(tare_step_state.gyroscope_y_sum / tare_step_state.valid_sample_count);
  out_tare.gyroscope_z_mdps = static_cast<std::int32_t>(tare_step_state.gyroscope_z_sum / tare_step_state.valid_sample_count);
  out_tare.accelerometer_x_mg = static_cast<std::int32_t>(tare_step_state.accelerometer_x_sum / tare_step_state.valid_sample_count);
  out_tare.accelerometer_y_mg = static_cast<std::int32_t>(tare_step_state.accelerometer_y_sum / tare_step_state.valid_sample_count);
  out_tare.accelerometer_z_mg = static_cast<std::int32_t>(tare_step_state.accelerometer_z_sum / tare_step_state.valid_sample_count);
  out_tare.magnetometer_x_mgauss = static_cast<std::int32_t>(tare_step_state.magnetometer_x_sum / tare_step_state.valid_sample_count);
  out_tare.magnetometer_y_mgauss = static_cast<std::int32_t>(tare_step_state.magnetometer_y_sum / tare_step_state.valid_sample_count);
  out_tare.magnetometer_z_mgauss = static_cast<std::int32_t>(tare_step_state.magnetometer_z_sum / tare_step_state.valid_sample_count);
  out_tare.has_tare = true;

  out_noise = {};
  out_noise.gyroscope_x_p_proc_mdps = get_p_proc_value(tare_step_state.gyroscope_x_noise_state);
  out_noise.gyroscope_y_p_proc_mdps = get_p_proc_value(tare_step_state.gyroscope_y_noise_state);
  out_noise.gyroscope_z_p_proc_mdps = get_p_proc_value(tare_step_state.gyroscope_z_noise_state);
  out_noise.accelerometer_x_p_proc_mg = get_p_proc_value(tare_step_state.accelerometer_x_noise_state);
  out_noise.accelerometer_y_p_proc_mg = get_p_proc_value(tare_step_state.accelerometer_y_noise_state);
  out_noise.accelerometer_z_p_proc_mg = get_p_proc_value(tare_step_state.accelerometer_z_noise_state);
  out_noise.has_noise_profile = true;

  tare_step_state.done = true;
  return true;
}
