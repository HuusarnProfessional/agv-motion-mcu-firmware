#include "board/stm32l152discovery/main_board_stm32l152discovery.hpp"

#include "core/api/voltage_monitor_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
  static platform_stm32_hal::voltage_adc_handle k_voltage_adc_handle = { nullptr, 5U };

  static const voltage_monitor_api::adc_operations k_voltage_adc_ops =
  {
    platform_stm32_hal::voltage_read_adc_raw
  };

  static const voltage_monitor_api::voltage_input k_voltage_monitors[] =
  {
    // todo: replace channel/handle when adc pinout is ready
    { static_cast<void *>(&k_voltage_adc_handle), 0U, &k_voltage_adc_ops }
  };

  static constexpr std::size_t k_voltage_monitor_count = sizeof(k_voltage_monitors) / sizeof(k_voltage_monitors[0]);
}

void board_stm32l152discovery_init_voltage_monitor(void)
{
  voltage_monitor_api::init(k_voltage_monitors, k_voltage_monitor_count);
}
