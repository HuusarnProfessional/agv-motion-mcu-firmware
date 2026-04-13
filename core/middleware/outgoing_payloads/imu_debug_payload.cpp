#include "core/middleware/outgoing_payloads/imu_debug_payload.hpp"

#include "core/api/imu_api.hpp"
#include "core/control/local_positioning/local_positioning_pipeline.hpp"
#include "core/middleware/binary_packing.hpp"
#include "core/middleware/middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const middleware::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    imu_api::imu_sample payload = {};
    payload_length_out = 0U;
    (void)state;

    if (payload_out == nullptr)
    {
      return false;
    }

    payload.gyroscope_state = imu_api::gyroscope_status::no_signal;
    payload.accelerometer_state = imu_api::accelerometer_status::no_signal;
    payload.magnetometer_state = imu_api::magnetometer_status::no_signal;
    imu_api::read_sample(local_positioning_pipeline::read_imu_id(), payload);
    middleware::binary_packing::writer writer(payload_out, capacity);

    if (!writer.write_i32(payload.gyroscope_x_mdps))
    {
      return false;
    }

    if (!writer.write_i32(payload.gyroscope_y_mdps))
    {
      return false;
    }

    if (!writer.write_i32(payload.gyroscope_z_mdps))
    {
      return false;
    }

    if (!writer.write_i32(payload.accelerometer_x_mg))
    {
      return false;
    }

    if (!writer.write_i32(payload.accelerometer_y_mg))
    {
      return false;
    }

    if (!writer.write_i32(payload.accelerometer_z_mg))
    {
      return false;
    }

    if (!writer.write_i32(payload.magnetometer_x_mgauss))
    {
      return false;
    }

    if (!writer.write_i32(payload.magnetometer_y_mgauss))
    {
      return false;
    }

    if (!writer.write_i32(payload.magnetometer_z_mgauss))
    {
      return false;
    }

    if (!writer.write_i16(payload.gyroscope_x_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.gyroscope_y_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.gyroscope_z_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.accelerometer_x_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.accelerometer_y_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.accelerometer_z_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.magnetometer_x_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.magnetometer_y_raw))
    {
      return false;
    }

    if (!writer.write_i16(payload.magnetometer_z_raw))
    {
      return false;
    }

    if (!writer.write_i32(payload.gyroscope_x_calibrated_mdps))
    {
      return false;
    }

    if (!writer.write_i32(payload.gyroscope_y_calibrated_mdps))
    {
      return false;
    }

    if (!writer.write_i32(payload.gyroscope_z_calibrated_mdps))
    {
      return false;
    }

    if (!writer.write_i32(payload.accelerometer_x_calibrated_mg))
    {
      return false;
    }

    if (!writer.write_i32(payload.accelerometer_y_calibrated_mg))
    {
      return false;
    }

    if (!writer.write_i32(payload.accelerometer_z_calibrated_mg))
    {
      return false;
    }

    if (!writer.write_bool(payload.has_calibration))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.gyroscope_state)))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.accelerometer_state)))
    {
      return false;
    }

    if (!writer.write_u8(static_cast<std::uint8_t>(payload.magnetometer_state)))
    {
      return false;
    }

    if (!writer.write_u32(payload.time_ms))
    {
      return false;
    }

    payload_length_out = writer.length();
    return true;
  }
}

namespace middleware_outgoing_payloads
{
  const outgoing_payload_definition imu_debug_payload_definition = {
    "imu_debug",
    0x12U,
    build_payload_bytes
  };
}
