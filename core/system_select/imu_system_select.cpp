#include "core/system_select/system_select.hpp"

#include "core/api/imu_api.hpp"
#include "core/impl/imu_lsm9ds1_impl.hpp"


namespace 
{
    void init_imu_lsm9ds1(const imu_api::imu_input *inputs, std::size_t count)
  {
    imu_lsm9ds1_impl::init(inputs, count);
  }

  bool read_imu_lsm9ds1(std::uint8_t imu_id, imu_api::imu_sample &out)
  {
    imu_lsm9ds1_impl::sample impl_sample{};
    const bool ok = imu_lsm9ds1_impl::read_sample(imu_id, impl_sample);

    out.gyroscope_x_mdps = impl_sample.gyroscope_x_mdps;
    out.gyroscope_y_mdps = impl_sample.gyroscope_y_mdps;
    out.gyroscope_z_mdps = impl_sample.gyroscope_z_mdps;

    out.gyroscope_x_raw = impl_sample.gyroscope_x_raw;
    out.gyroscope_y_raw = impl_sample.gyroscope_y_raw;
    out.gyroscope_z_raw = impl_sample.gyroscope_z_raw;
    out.gyroscope_sample_id = impl_sample.gyroscope_sample_id;

    out.accelerometer_x_mg = impl_sample.accelerometer_x_mg;
    out.accelerometer_y_mg = impl_sample.accelerometer_y_mg;
    out.accelerometer_z_mg = impl_sample.accelerometer_z_mg;

    out.accelerometer_x_raw = impl_sample.accelerometer_x_raw;
    out.accelerometer_y_raw = impl_sample.accelerometer_y_raw;
    out.accelerometer_z_raw = impl_sample.accelerometer_z_raw;
    out.accelerometer_sample_id = impl_sample.accelerometer_sample_id;

    out.magnetometer_x_mgauss = impl_sample.magnetometer_x_mgauss;
    out.magnetometer_y_mgauss = impl_sample.magnetometer_y_mgauss;
    out.magnetometer_z_mgauss = impl_sample.magnetometer_z_mgauss;

    out.magnetometer_x_raw = impl_sample.magnetometer_x_raw;
    out.magnetometer_y_raw = impl_sample.magnetometer_y_raw;
    out.magnetometer_z_raw = impl_sample.magnetometer_z_raw;

    out.gyroscope_state = static_cast<imu_api::gyroscope_status>(impl_sample.gyroscope_state);
    out.accelerometer_state = static_cast<imu_api::accelerometer_status>(impl_sample.accelerometer_state);
    out.magnetometer_state = static_cast<imu_api::magnetometer_status>(impl_sample.magnetometer_state);

    out.time_ms = impl_sample.time_ms;
    return ok;
  }

}
