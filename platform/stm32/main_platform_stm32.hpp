#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/motor_api.hpp"
#include "core/api/encoder_api.hpp"
#include "core/api/imu_api.hpp"
#include "core/api/voltage_monitor_api.hpp"
#include "core/api/obstacle_api.hpp"

namespace platform_stm32_hal
{
  const motor_api::pwm_ops *get_pwm_ops(void);
  const encoder_api::capture_operations *get_encoder_capture_ops(void);

  struct gpio_pin_handle
  {
    void *port = nullptr;
    std::uint16_t pin = 0U;
  };

  struct imu_spi_bus_handle
  {
    void *spi_handle = nullptr;
    gpio_pin_handle chip_select_accelerometer_gyroscope = {};
    gpio_pin_handle chip_select_magnetometer = {};
    std::uint32_t transfer_timeout_ms = 10U;
  };

  bool imu_read_register_spi(void *platform_handle,
                             imu_api::imu_target target,
                             std::uint8_t register_address,
                             std::uint8_t *data_out,
                             std::size_t length);
  bool imu_write_register_spi(void *platform_handle,
                              imu_api::imu_target target,
                              std::uint8_t register_address,
                              const std::uint8_t *data_in,
                              std::size_t length);

  struct voltage_adc_handle
  {
    void *adc_handle = nullptr;
    std::uint32_t transfer_timeout_ms = 5U;
  };

  bool voltage_read_adc_raw(void *platform_handle,
                            std::uint8_t channel,
                            std::uint16_t &raw_adc_out,
                            std::uint32_t &time_ms_out);

  struct obstacle_pin_map
  {
    gpio_pin_handle trigger = {};
    gpio_pin_handle echo = {};
  };

  struct obstacle_hcsr04_handle
  {
    const obstacle_pin_map *sensors = nullptr;
    std::size_t sensor_count = 0U;
    std::uint32_t echo_timeout_us = 30000U;
  };

  bool obstacle_read_echo_pulse_us(void *platform_handle,
                                   std::uint8_t channel,
                                   std::uint32_t &pulse_width_us_out,
                                   std::uint32_t &time_ms_out);

  bool comm_uart_configure(void *platform_handle, std::uint16_t tx_pin_id, std::uint16_t rx_pin_id, std::uint32_t baud_rate);
  bool comm_uart_tx_bytes(void *platform_handle, const std::uint8_t *data, std::size_t length);
  std::size_t comm_uart_rx_bytes(void *platform_handle, std::uint8_t *data_out, std::size_t capacity);
}

