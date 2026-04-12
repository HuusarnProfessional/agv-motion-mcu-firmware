#include "core/middleware/stm_esp/outgoing_payloads/imu_debug_payload.hpp"

#include "core/middleware/stm_esp/binary_packing.hpp"
#include "core/middleware/stm_esp/stm_esp_middleware_state.hpp"

namespace
{
  bool build_payload_bytes(const stm_esp::middleware_state &state, std::uint8_t *payload_out, std::size_t capacity, std::size_t &payload_length_out)
  {
    payload_length_out = 0U;

    if (!state.has_imu_debug || payload_out == nullptr)
    {
      return false;
    }

    const stm_esp_outgoing_payloads::imu_debug_payload_data &payload = state.imu_debug;
    stm_esp::binary_packing::writer writer(payload_out, capacity);

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

    if (!writer.write_u8(payload.gyroscope_status))
    {
      return false;
    }

    if (!writer.write_u8(payload.accelerometer_status))
    {
      return false;
    }

    if (!writer.write_u8(payload.magnetometer_status))
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

namespace stm_esp_outgoing_payloads
{
  const outgoing_payload_definition imu_debug_payload_definition = {
    "imu_debug",
    0x12U,
    build_payload_bytes
  };
}
