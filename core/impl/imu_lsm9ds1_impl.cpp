#include "core/impl/imu_lsm9ds1_impl.hpp"
#include "libraries/lsm9ds1_driver/lsm9ds1_reg.h"

namespace
{
  const imu_api::imu_input *g_input = nullptr;
  std::size_t g_input_count = 0u;

  constexpr std::int32_t k_library_status_ok = 0;
  constexpr std::int32_t k_library_status_error = -1;
  constexpr lsm9ds1_imu_odr_t k_imu_data_rate = LSM9DS1_IMU_238Hz;

  struct lsm9ds1_library_context
  {
    const imu_api::imu_input *selected_imu;
    imu_api::imu_target selected_target;
  };

  int32_t lsm9ds1_library_read(void *handle, uint8_t register_address, uint8_t *data_out, uint16_t length)
  {
    if (handle == nullptr || data_out == nullptr || length == 0u)
    {
      return k_library_status_error;
    }

    lsm9ds1_library_context *library_context = static_cast<lsm9ds1_library_context *>(handle);

    if (library_context->selected_imu == nullptr)
    {
      return k_library_status_error;
    }

    if (library_context->selected_imu->read_register == nullptr)
    {
      return k_library_status_error;
    }

    const bool ok = library_context->selected_imu->read_register(library_context->selected_imu->platform_handle, library_context->selected_target, register_address, data_out, static_cast<std::size_t>(length));

    if (ok)
    {
      return k_library_status_ok;
    }
    else
    {
      return k_library_status_error;
    }
  }

  int32_t lsm9ds1_library_write(void *handle, uint8_t register_address, const uint8_t *data_in, uint16_t length)
  {
    if (handle == nullptr || data_in == nullptr || length == 0u)
    {
      return k_library_status_error;
    }

    // cast generic library handle back to our typed imu context
    lsm9ds1_library_context *library_context = static_cast<lsm9ds1_library_context *>(handle);

    if (library_context->selected_imu == nullptr)
    {
      return k_library_status_error;
    }

    if (library_context->selected_imu->write_register == nullptr)
    {
      return k_library_status_error;
    }

    const bool ok = library_context->selected_imu->write_register(library_context->selected_imu->platform_handle, library_context->selected_target, register_address, data_in, static_cast<std::size_t>(length));

    if (ok)
    {
      return k_library_status_ok;
    }
    else
    {
      return k_library_status_error;
    }
  }

  stmdev_ctx_t make_lsm9ds1_context(const imu_api::imu_input &selected_imu, imu_api::imu_target selected_target, lsm9ds1_library_context &library_context)
  {
    library_context.selected_imu = &selected_imu;
    library_context.selected_target = selected_target;

    stmdev_ctx_t context = {};
    context.handle = &library_context;
    context.read_reg = lsm9ds1_library_read;
    context.write_reg = lsm9ds1_library_write;
    context.mdelay = nullptr;

    return context;
  }

  bool read_gyroscope_sample(const stmdev_ctx_t &ag_context, imu_lsm9ds1_impl::sample &out)
  {
    int16_t gyroscope_raw[3] = {0, 0, 0};
    const int32_t gyroscope_read_status = lsm9ds1_angular_rate_raw_get(&ag_context, gyroscope_raw);

    if (gyroscope_read_status != k_library_status_ok)
    {
      out.gyroscope_state = imu_lsm9ds1_impl::gyroscope_status::stale;
      return false;
    }

    out.gyroscope_x_raw = gyroscope_raw[0];
    out.gyroscope_y_raw = gyroscope_raw[1];
    out.gyroscope_z_raw = gyroscope_raw[2];

    out.gyroscope_x_mdps = static_cast<std::int32_t>(lsm9ds1_from_fs245dps_to_mdps(gyroscope_raw[0]));
    out.gyroscope_y_mdps = static_cast<std::int32_t>(lsm9ds1_from_fs245dps_to_mdps(gyroscope_raw[1]));
    out.gyroscope_z_mdps = static_cast<std::int32_t>(lsm9ds1_from_fs245dps_to_mdps(gyroscope_raw[2]));

    out.gyroscope_state = imu_lsm9ds1_impl::gyroscope_status::ok;
    return true;
  }

  bool read_accelerometer_sample(const stmdev_ctx_t &ag_context, imu_lsm9ds1_impl::sample &out)
  {
    int16_t accelerometer_raw[3] = {0, 0, 0};
    const int32_t accelerometer_read_status = lsm9ds1_acceleration_raw_get(&ag_context, accelerometer_raw);

    if (accelerometer_read_status != k_library_status_ok)
    {
      out.accelerometer_state = imu_lsm9ds1_impl::accelerometer_status::stale;
      return false;
    }

    out.accelerometer_x_raw = accelerometer_raw[0];
    out.accelerometer_y_raw = accelerometer_raw[1];
    out.accelerometer_z_raw = accelerometer_raw[2];

    out.accelerometer_x_mg = static_cast<std::int32_t>(lsm9ds1_from_fs2g_to_mg(accelerometer_raw[0]));
    out.accelerometer_y_mg = static_cast<std::int32_t>(lsm9ds1_from_fs2g_to_mg(accelerometer_raw[1]));
    out.accelerometer_z_mg = static_cast<std::int32_t>(lsm9ds1_from_fs2g_to_mg(accelerometer_raw[2]));

    out.accelerometer_state = imu_lsm9ds1_impl::accelerometer_status::ok;
    return true;
  }

  bool read_magnetometer_sample(const stmdev_ctx_t &magnetometer_context, imu_lsm9ds1_impl::sample &out)
  {
    int16_t magnetometer_raw[3] = {0, 0, 0};
    const int32_t magnetometer_read_status = lsm9ds1_magnetic_raw_get(&magnetometer_context, magnetometer_raw);

    if (magnetometer_read_status != k_library_status_ok)
    {
      out.magnetometer_state = imu_lsm9ds1_impl::magnetometer_status::stale;
      return false;
    }

    out.magnetometer_x_raw = magnetometer_raw[0];
    out.magnetometer_y_raw = magnetometer_raw[1];
    out.magnetometer_z_raw = magnetometer_raw[2];

    out.magnetometer_x_mgauss = static_cast<std::int32_t>(lsm9ds1_from_fs4gauss_to_mG(magnetometer_raw[0]));
    out.magnetometer_y_mgauss = static_cast<std::int32_t>(lsm9ds1_from_fs4gauss_to_mG(magnetometer_raw[1]));
    out.magnetometer_z_mgauss = static_cast<std::int32_t>(lsm9ds1_from_fs4gauss_to_mG(magnetometer_raw[2]));

    out.magnetometer_state = imu_lsm9ds1_impl::magnetometer_status::ok;
    return true;
  }
}

namespace imu_lsm9ds1_impl
{
  void init(const imu_api::imu_input *input, std::size_t count)
  {
    if (input == nullptr || count == 0u)
    {
      g_input = nullptr;
      g_input_count = 0u;
      return;
    }

    g_input = input;
    g_input_count = count;

    // build ag/mag library contexts and verify device ids for each imu
    for (std::size_t index = 0u; index < g_input_count; ++index)
    {
      const imu_api::imu_input &selected_imu = g_input[index];

      lsm9ds1_library_context ag_library_context = {};
      lsm9ds1_library_context magnetometer_library_context = {};

      stmdev_ctx_t ag_context = make_lsm9ds1_context(selected_imu, imu_api::imu_target::accelerometer_gyroscope, ag_library_context);
      stmdev_ctx_t magnetometer_context = make_lsm9ds1_context(selected_imu, imu_api::imu_target::magnetometer, magnetometer_library_context);

      lsm9ds1_id_t device_id = {0u, 0u};
      (void)lsm9ds1_dev_id_get(&magnetometer_context, &ag_context, &device_id);

      // dps = degrees per second, use +-245 because the agv does not rotate very fast
      (void)lsm9ds1_gy_full_scale_set(&ag_context, LSM9DS1_245dps);

      (void)lsm9ds1_xl_full_scale_set(&ag_context, LSM9DS1_2g);
      (void)lsm9ds1_imu_data_rate_set(&ag_context, k_imu_data_rate);

      (void)lsm9ds1_mag_full_scale_set(&magnetometer_context, LSM9DS1_4Ga);

      // struct to enable
      const lsm9ds1_gy_axis_t gyro_axes = {1u, 1u, 1u};
      const lsm9ds1_xl_axis_t accel_axes = {1u, 1u, 1u};

      // send enable
      (void)lsm9ds1_gy_axis_set(&ag_context, gyro_axes);
      (void)lsm9ds1_xl_axis_set(&ag_context, accel_axes);

    }
  }

  bool read_sample(std::uint8_t imu_id, sample &out)
  {
    out = {};

    out.gyroscope_state = gyroscope_status::no_signal;
    out.accelerometer_state = accelerometer_status::no_signal;
    out.magnetometer_state = magnetometer_status::no_signal;

    if (g_input == nullptr || g_input_count == 0u)
    {
      return false;
    }

    if (imu_id >= g_input_count)
    {
      return false;
    }

    const imu_api::imu_input &selected_imu = g_input[imu_id];

    if (selected_imu.read_register == nullptr)
    {
      return false;
    }

    lsm9ds1_library_context ag_library_context = {};
    stmdev_ctx_t ag_context = make_lsm9ds1_context(selected_imu, imu_api::imu_target::accelerometer_gyroscope, ag_library_context);

    lsm9ds1_library_context magnetometer_library_context = {};
    stmdev_ctx_t magnetometer_context = make_lsm9ds1_context(selected_imu, imu_api::imu_target::magnetometer, magnetometer_library_context);

    const bool gyroscope_ok = read_gyroscope_sample(ag_context, out);
    const bool accelerometer_ok = read_accelerometer_sample(ag_context, out);
    const bool magnetometer_ok = read_magnetometer_sample(magnetometer_context, out);
    return gyroscope_ok || accelerometer_ok || magnetometer_ok;
  }
}
