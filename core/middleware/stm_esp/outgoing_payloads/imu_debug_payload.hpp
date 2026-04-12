#pragma once

#include <cstdint>

#include "core/middleware/stm_esp/outgoing_payloads/outgoing_payload_definition.hpp"

namespace stm_esp_outgoing_payloads
{
  struct imu_debug_payload_data
  {
    std::int32_t gyroscope_x_mdps = 0;
    std::int32_t gyroscope_y_mdps = 0;
    std::int32_t gyroscope_z_mdps = 0;
    std::int32_t accelerometer_x_mg = 0;
    std::int32_t accelerometer_y_mg = 0;
    std::int32_t accelerometer_z_mg = 0;
    std::int32_t magnetometer_x_mgauss = 0;
    std::int32_t magnetometer_y_mgauss = 0;
    std::int32_t magnetometer_z_mgauss = 0;
    std::int16_t gyroscope_x_raw = 0;
    std::int16_t gyroscope_y_raw = 0;
    std::int16_t gyroscope_z_raw = 0;
    std::int16_t accelerometer_x_raw = 0;
    std::int16_t accelerometer_y_raw = 0;
    std::int16_t accelerometer_z_raw = 0;
    std::int16_t magnetometer_x_raw = 0;
    std::int16_t magnetometer_y_raw = 0;
    std::int16_t magnetometer_z_raw = 0;
    std::int32_t gyroscope_x_calibrated_mdps = 0;
    std::int32_t gyroscope_y_calibrated_mdps = 0;
    std::int32_t gyroscope_z_calibrated_mdps = 0;
    std::int32_t accelerometer_x_calibrated_mg = 0;
    std::int32_t accelerometer_y_calibrated_mg = 0;
    std::int32_t accelerometer_z_calibrated_mg = 0;
    bool has_calibration = false;
    std::uint8_t gyroscope_status = 0U;
    std::uint8_t accelerometer_status = 0U;
    std::uint8_t magnetometer_status = 0U;
    std::uint32_t time_ms = 0U;
  };

  extern const outgoing_payload_definition imu_debug_payload_definition;
}
