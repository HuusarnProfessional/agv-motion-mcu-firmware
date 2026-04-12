#pragma once

#include <cstddef>
#include <cstdint>

#include "core/api/motor_api.hpp"
#include "core/api/encoder_api.hpp"
#include "core/api/imu_api.hpp"
#include "core/api/voltage_monitor_api.hpp"
#include "core/api/comm_uart_api.hpp"
#include "core/api/led_api.hpp"
#include "core/api/obstacle_api.hpp"
#include "core/api/button_api.hpp"

namespace platform_stm32_hal
{
  const motor_api::pwm_ops *get_pwm_ops(void);
  const encoder_api::capture_operations *get_encoder_capture_ops(void);

  using gpio_exti_callback_fn = void (*)(void *callback_context, std::uint16_t gpio_pin);

  bool register_gpio_exti_callback(std::uint16_t gpio_pin, gpio_exti_callback_fn callback, void *callback_context);

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

  bool imu_read_register_spi(void *platform_handle, imu_api::imu_target target, std::uint8_t register_address, std::uint8_t *data_out, std::size_t length);
  bool imu_write_register_spi(void *platform_handle, imu_api::imu_target target, std::uint8_t register_address, const std::uint8_t *data_in, std::size_t length);

  struct voltage_adc_handle
  {
    void *adc_handle = nullptr;
    std::uint32_t transfer_timeout_ms = 5U;
  };

  bool voltage_read_adc_raw(void *platform_handle, std::uint8_t channel, std::uint16_t &raw_adc_out, std::uint32_t &time_ms_out);

  bool led_write_gpio_level(void *platform_handle, bool is_high);
  bool led_read_gpio_level(void *platform_handle, bool &is_high_out);

  struct obstacle_pin_map
  {
    gpio_pin_handle trigger = {};
    gpio_pin_handle echo = {};
  };

  struct obstacle_hcsr04_handle
  {
    const obstacle_pin_map *sensors = nullptr;
    std::size_t sensor_count = 0U;
    void *alarm_timer_handle = nullptr;
  };

  bool obstacle_register_event_callback(void *platform_handle, obstacle_api::platform_event_callback_fn event_callback, void *event_context);
  bool obstacle_set_trigger_level(void *platform_handle, std::uint8_t channel, bool is_high);
  bool obstacle_schedule_alarm_us(void *platform_handle, std::uint32_t delay_us);
  bool obstacle_cancel_alarm(void *platform_handle);
  bool obstacle_read_time_ms(void *platform_handle, std::uint32_t &time_ms_out);

  bool button_register_event_source(void *platform_handle);
  bool button_read_gpio_level(void *platform_handle, bool &is_high_out);
  bool button_read_gpio_event(void *platform_handle, button_api::gpio_event &event_out);

  bool comm_uart_configure(void *platform_handle, std::uint16_t tx_pin_id, std::uint16_t rx_pin_id, std::uint32_t baud_rate);
  bool comm_uart_register_event_callback(void *platform_handle, comm_uart_api::platform_event_callback_fn event_callback, void *event_context);
  bool comm_uart_enable_receive(void *platform_handle);
  bool comm_uart_transmit_byte(void *platform_handle, std::uint8_t byte);
}

