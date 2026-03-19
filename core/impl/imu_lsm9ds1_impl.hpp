#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/imu_api.hpp"

namespace imu_lsm9ds1_impl
{
    enum class gyroscope_status : std::uint8_t
    {
        ok = 0,
        no_signal,
        stale,
        invalid_sample
    };

    enum class accelerometer_status : std::uint8_t
    {
        ok = 0,
        no_signal,
        stale,
        invalid_sample
    };

    enum class magnetometer_status : std::uint8_t
    {
        ok = 0,
        no_signal,
        stale,
        invalid_sample
    };

    struct sample
    {
        std::int32_t gyroscope_x_mdps;
        std::int32_t gyroscope_y_mdps;
        std::int32_t gyroscope_z_mdps;
        std::int16_t gyroscope_x_raw;
        std::int16_t gyroscope_y_raw;
        std::int16_t gyroscope_z_raw;
        gyroscope_status gyroscope_state;

        std::int32_t accelerometer_x_mg;
        std::int32_t accelerometer_y_mg;
        std::int32_t accelerometer_z_mg;
        std::int16_t accelerometer_x_raw;
        std::int16_t accelerometer_y_raw;
        std::int16_t accelerometer_z_raw;
        accelerometer_status accelerometer_state;

        std::int32_t magnetometer_x_mgauss;
        std::int32_t magnetometer_y_mgauss;
        std::int32_t magnetometer_z_mgauss;
        std::int16_t magnetometer_x_raw;
        std::int16_t magnetometer_y_raw;
        std::int16_t magnetometer_z_raw;
        magnetometer_status magnetometer_state;

        std::uint32_t time_ms;
    };

    void init(const imu_api::imu_input *inputs, std::size_t count);
    bool read_sample(std::uint8_t imu_id, sample &out);
}