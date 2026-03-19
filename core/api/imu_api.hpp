#pragma once
#include <cstdint>
#include <cstddef>


namespace imu_api
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

    enum class imu_target : std::uint8_t
    {
        accelerometer_gyroscope = 0,
        magnetometer
    };

    using read_register_fn = bool(*)(void *platform_handle, imu_target target, std::uint8_t register_address, std::uint8_t *data_out, std::size_t length);
    using write_register_fn = bool(*)(void *platform_handle, imu_target target, std::uint8_t register_address, const std::uint8_t *data_in, std::size_t length);

  
    
    struct imu_sample
    {
        //mdps = milli degrees per sec
        std::int32_t gyroscope_x_mdps;
        std::int32_t gyroscope_y_mdps;
        std::int32_t gyroscope_z_mdps;

        std::int16_t gyroscope_x_raw;
        std::int16_t gyroscope_y_raw;
        std::int16_t gyroscope_z_raw;

        gyroscope_status gyroscope_state;

        //mg = milli gravity
        std::int32_t accelerometer_x_mg;
        std::int32_t accelerometer_y_mg;
        std::int32_t accelerometer_z_mg;

        std::int16_t accelerometer_x_raw;
        std::int16_t accelerometer_y_raw;
        std::int16_t accelerometer_z_raw;

        accelerometer_status accelerometer_state;

        // mgauss = milli guass
        std::int32_t magnetometer_x_mgauss;
        std::int32_t magnetometer_y_mgauss;
        std::int32_t magnetometer_z_mgauss;

        std::int16_t magnetometer_x_raw;
        std::int16_t magnetometer_y_raw;
        std::int16_t magnetometer_z_raw;

        magnetometer_status magnetometer_state;

    
        std::uint32_t time_ms;
        
        
    };

    struct imu_input
    {
        void *platform_handle;

        const read_register_fn read_register;
        const write_register_fn write_register;


    };

    using backend_init_fn = void(*)(const imu_input *imus, std::size_t count);
    using backend_read_sample_fn = bool(*)(std::uint8_t imu_id, imu_sample &out);

    struct backend_operation
    {
        backend_init_fn init_fn;
        backend_read_sample_fn read_sample_fn;
    };


    void init(const imu_input *imus, std::size_t count);


    bool read_sample(std::uint8_t imu_id, imu_sample &out);

}