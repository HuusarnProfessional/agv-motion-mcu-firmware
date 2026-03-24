#include "core/control/imu_calibration/imu_calibration_solve_and_set.hpp"

#include <cmath>

namespace
{
  constexpr double k_matrix_scale = 1000000.0;

  struct vector3
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  double vector_length(const vector3 &v)
  {
    return std::sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
  }

  bool normalize(vector3 &v)
  {
    const double length = vector_length(v);
    if (length <= 0.0)
    {
      return false;
    }

    v.x /= length;
    v.y /= length;
    v.z /= length;
    return true;
  }

  vector3 cross_product(const vector3 &a, const vector3 &b)
  {
    vector3 out = {};
    out.x = (a.y * b.z) - (a.z * b.y);
    out.y = (a.z * b.x) - (a.x * b.z);
    out.z = (a.x * b.y) - (a.y * b.x);
    return out;
  }
}

void build_mean_values_from_drive_samples(const imu_drive_sample_values &sample_values, imu_drive_sample_mean_values &out_mean_values)
{
  out_mean_values = {};

  if (sample_values.sample_count <= 0)
  {
    return;
  }

  out_mean_values.gyroscope_x_raw = static_cast<std::int32_t>(sample_values.gyroscope_x_raw_sum / sample_values.sample_count);
  out_mean_values.gyroscope_y_raw = static_cast<std::int32_t>(sample_values.gyroscope_y_raw_sum / sample_values.sample_count);
  out_mean_values.gyroscope_z_raw = static_cast<std::int32_t>(sample_values.gyroscope_z_raw_sum / sample_values.sample_count);
  out_mean_values.accelerometer_x_raw = static_cast<std::int32_t>(sample_values.accelerometer_x_raw_sum / sample_values.sample_count);
  out_mean_values.accelerometer_y_raw = static_cast<std::int32_t>(sample_values.accelerometer_y_raw_sum / sample_values.sample_count);
  out_mean_values.accelerometer_z_raw = static_cast<std::int32_t>(sample_values.accelerometer_z_raw_sum / sample_values.sample_count);
  out_mean_values.magnetometer_x_raw = static_cast<std::int32_t>(sample_values.magnetometer_x_raw_sum / sample_values.sample_count);
  out_mean_values.magnetometer_y_raw = static_cast<std::int32_t>(sample_values.magnetometer_y_raw_sum / sample_values.sample_count);
  out_mean_values.magnetometer_z_raw = static_cast<std::int32_t>(sample_values.magnetometer_z_raw_sum / sample_values.sample_count);
  out_mean_values.has_mean = true;
}

void solve_alignment_matrix(const imu_drive_sample_mean_values &forward_mean_values, const imu_drive_sample_mean_values &backward_mean_values, imu_api::imu_tare_values &io_tare_values)
{
  if (!io_tare_values.has_tare || !forward_mean_values.has_mean || !backward_mean_values.has_mean)
  {
    return;
  }

  vector3 z_axis_imu = {};
  z_axis_imu.x = static_cast<double>(io_tare_values.accelerometer_x_mg);
  z_axis_imu.y = static_cast<double>(io_tare_values.accelerometer_y_mg);
  z_axis_imu.z = static_cast<double>(io_tare_values.accelerometer_z_mg);

  vector3 x_axis_imu = {};
  x_axis_imu.x = static_cast<double>(forward_mean_values.accelerometer_x_raw - backward_mean_values.accelerometer_x_raw);
  x_axis_imu.y = static_cast<double>(forward_mean_values.accelerometer_y_raw - backward_mean_values.accelerometer_y_raw);
  x_axis_imu.z = static_cast<double>(forward_mean_values.accelerometer_z_raw - backward_mean_values.accelerometer_z_raw);

  if (!normalize(z_axis_imu) || !normalize(x_axis_imu))
  {
    return;
  }

  vector3 y_axis_imu = cross_product(z_axis_imu, x_axis_imu);
  if (!normalize(y_axis_imu))
  {
    return;
  }

  x_axis_imu = cross_product(y_axis_imu, z_axis_imu);
  if (!normalize(x_axis_imu))
  {
    return;
  }

  io_tare_values.r11 = static_cast<std::int32_t>(x_axis_imu.x * k_matrix_scale);
  io_tare_values.r12 = static_cast<std::int32_t>(x_axis_imu.y * k_matrix_scale);
  io_tare_values.r13 = static_cast<std::int32_t>(x_axis_imu.z * k_matrix_scale);
  io_tare_values.r21 = static_cast<std::int32_t>(y_axis_imu.x * k_matrix_scale);
  io_tare_values.r22 = static_cast<std::int32_t>(y_axis_imu.y * k_matrix_scale);
  io_tare_values.r23 = static_cast<std::int32_t>(y_axis_imu.z * k_matrix_scale);
  io_tare_values.r31 = static_cast<std::int32_t>(z_axis_imu.x * k_matrix_scale);
  io_tare_values.r32 = static_cast<std::int32_t>(z_axis_imu.y * k_matrix_scale);
  io_tare_values.r33 = static_cast<std::int32_t>(z_axis_imu.z * k_matrix_scale);
}

void build_calibration_profile_from_tare(const imu_api::imu_tare_values &tare_values, const imu_api::imu_noise_profile &noise_values, imu_api::imu_calibration_profile &out_profile)
{
  out_profile = {};

  if (!tare_values.has_tare)
  {
    return;
  }

  out_profile.tare = tare_values;
  out_profile.noise = noise_values;
  out_profile.remove_gravity = false;
  out_profile.has_calibration = true;
}

void set_calibration_profile_to_imu(std::uint8_t imu_id, const imu_api::imu_tare_values &tare_values, const imu_api::imu_noise_profile &noise_values)
{
  imu_api::imu_calibration_profile calibration_profile = {};
  build_calibration_profile_from_tare(tare_values, noise_values, calibration_profile);

  if (!calibration_profile.has_calibration)
  {
    return;
  }

  imu_api::set_calibration(imu_id, calibration_profile);
}
