#include "core/api/imu_api.hpp"
#include "core/system_select/system_select.hpp"
#include <array>

namespace
{
    constexpr std::size_t k_max_imu_count = 4u;
    constexpr std::int64_t k_matrix_scale = 1000000LL;
    imu_api::backend_operation g_backend = {};
    bool g_backend_ready = false;
    std::array<imu_api::imu_calibration_profile, k_max_imu_count> g_calibration_profiles = {};

    void apply_calibration(const imu_api::imu_calibration_profile &calibration_profile, imu_api::imu_sample &sample)
    {
        const imu_api::imu_tare_values &tare = calibration_profile.tare;
        const std::int32_t gyro_x_bias_removed = sample.gyroscope_x_mdps - tare.gyroscope_x_mdps;
        const std::int32_t gyro_y_bias_removed = sample.gyroscope_y_mdps - tare.gyroscope_y_mdps;
        const std::int32_t gyro_z_bias_removed = sample.gyroscope_z_mdps - tare.gyroscope_z_mdps;
        const std::int32_t acc_x_bias_removed = sample.accelerometer_x_mg - tare.accelerometer_x_mg;
        const std::int32_t acc_y_bias_removed = sample.accelerometer_y_mg - tare.accelerometer_y_mg;
        const std::int32_t acc_z_bias_removed = sample.accelerometer_z_mg - tare.accelerometer_z_mg;

        const std::int64_t gyro_x_mapped = static_cast<std::int64_t>(tare.r11) * gyro_x_bias_removed + static_cast<std::int64_t>(tare.r12) * gyro_y_bias_removed + static_cast<std::int64_t>(tare.r13) * gyro_z_bias_removed;
        const std::int64_t gyro_y_mapped = static_cast<std::int64_t>(tare.r21) * gyro_x_bias_removed + static_cast<std::int64_t>(tare.r22) * gyro_y_bias_removed + static_cast<std::int64_t>(tare.r23) * gyro_z_bias_removed;
        const std::int64_t gyro_z_mapped = static_cast<std::int64_t>(tare.r31) * gyro_x_bias_removed + static_cast<std::int64_t>(tare.r32) * gyro_y_bias_removed + static_cast<std::int64_t>(tare.r33) * gyro_z_bias_removed;
        const std::int64_t acc_x_mapped = static_cast<std::int64_t>(tare.r11) * acc_x_bias_removed + static_cast<std::int64_t>(tare.r12) * acc_y_bias_removed + static_cast<std::int64_t>(tare.r13) * acc_z_bias_removed;
        const std::int64_t acc_y_mapped = static_cast<std::int64_t>(tare.r21) * acc_x_bias_removed + static_cast<std::int64_t>(tare.r22) * acc_y_bias_removed + static_cast<std::int64_t>(tare.r23) * acc_z_bias_removed;
        const std::int64_t acc_z_mapped = static_cast<std::int64_t>(tare.r31) * acc_x_bias_removed + static_cast<std::int64_t>(tare.r32) * acc_y_bias_removed + static_cast<std::int64_t>(tare.r33) * acc_z_bias_removed;

        sample.gyroscope_x_calibrated_mdps = static_cast<std::int32_t>(gyro_x_mapped / k_matrix_scale);
        sample.gyroscope_y_calibrated_mdps = static_cast<std::int32_t>(gyro_y_mapped / k_matrix_scale);
        sample.gyroscope_z_calibrated_mdps = static_cast<std::int32_t>(gyro_z_mapped / k_matrix_scale);
        sample.accelerometer_x_calibrated_mg = static_cast<std::int32_t>(acc_x_mapped / k_matrix_scale);
        sample.accelerometer_y_calibrated_mg = static_cast<std::int32_t>(acc_y_mapped / k_matrix_scale);
        sample.accelerometer_z_calibrated_mg = static_cast<std::int32_t>(acc_z_mapped / k_matrix_scale);

        if (calibration_profile.remove_gravity)
        {
            sample.accelerometer_z_calibrated_mg = sample.accelerometer_z_calibrated_mg - 1000;
        }
    }
}

namespace imu_api
{
    void init(const imu_input *input, std::size_t count)
    {
        g_backend = {};
        g_backend_ready = false;

        if (input == nullptr || count == 0u)
        {
            return;
        }
        system_select::select_imu_backend(g_backend);

        if(g_backend.init_fn == nullptr || g_backend.read_sample_fn == nullptr)
        {
            return;
        }

        g_backend.init_fn(input, count);
        g_backend_ready = true;
    }

    bool read_sample(std::uint8_t imu_id, imu_sample &out)
    {
        if(!g_backend_ready || g_backend.read_sample_fn == nullptr)
        {
            return false;
        }

        const bool got_sample = g_backend.read_sample_fn(imu_id, out);
        out.has_calibration = false;

        if (imu_id < k_max_imu_count)
        {
            out.has_calibration = g_calibration_profiles[imu_id].has_calibration;
        }

        out.gyroscope_x_calibrated_mdps = out.gyroscope_x_mdps;
        out.gyroscope_y_calibrated_mdps = out.gyroscope_y_mdps;
        out.gyroscope_z_calibrated_mdps = out.gyroscope_z_mdps;
        out.accelerometer_x_calibrated_mg = out.accelerometer_x_mg;
        out.accelerometer_y_calibrated_mg = out.accelerometer_y_mg;
        out.accelerometer_z_calibrated_mg = out.accelerometer_z_mg;

        if (out.has_calibration)
        {
            apply_calibration(g_calibration_profiles[imu_id], out);
        }

        return got_sample;
    }

    void set_calibration(std::uint8_t imu_id, const imu_calibration_profile &profile)
    {
        if (imu_id >= k_max_imu_count)
        {
            return;
        }

        g_calibration_profiles[imu_id] = profile;
    }

    bool get_calibration(std::uint8_t imu_id, imu_calibration_profile &out)
    {
        if (imu_id >= k_max_imu_count)
        {
            return false;
        }

        out = g_calibration_profiles[imu_id];
        return out.has_calibration;
    }

    void clear_calibration(std::uint8_t imu_id)
    {
        if (imu_id >= k_max_imu_count)
        {
            return;
        }

        g_calibration_profiles[imu_id] = {};
    }
}
