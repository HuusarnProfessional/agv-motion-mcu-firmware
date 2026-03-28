#include "core/control/local_positioning/sensorfusion/rotation/imu/sensorfusion_rotation_imu.hpp"
#include "core/control/local_positioning/sensorfusion/rotation/sensorfusion_tuning.hpp"

#include <cmath>

namespace
{
  std::int32_t compute_gyro_rotation_math_model_confidence(const motion_model_imu::motion_model_snapshot &imu_motion)
  {
    const double omega_degps = std::abs(static_cast<double>(imu_motion.gyroscope_z_calibrated_mdps)) / 1000.0;
    const double exponent_argument = -omega_degps / sensorfusion_tuning::k_gyro_rotation_observability_degps;
    const double confidence_percent = 100.0 * (1.0 - std::exp(exponent_argument));
    const double confidence_scaled = confidence_percent * sensorfusion_tuning::k_confidence_max / 100.0;

    return static_cast<std::int32_t>(confidence_scaled);
  }
}

namespace sensorfusion_rotation_imu
{
  void update_rotation_confidence(const motion_model_imu::motion_model_snapshot &imu_motion, sensorfusion_rotation::rotation_snapshot &out)
  {
    out.gyro_confidence_rotation_raw = imu_motion.confidence_rotation;
    out.gyro_confidence_rotation_math_model = compute_gyro_rotation_math_model_confidence(imu_motion);

    if (out.gyro_confidence_rotation_raw < out.gyro_confidence_rotation_math_model)
    {
      out.gyro_confidence_rotation_final = out.gyro_confidence_rotation_raw;
    }
    else
    {
      out.gyro_confidence_rotation_final = out.gyro_confidence_rotation_math_model;
    }
  }
}
