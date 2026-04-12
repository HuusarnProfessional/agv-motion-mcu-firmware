#include "board/stm32g474re_agv/board_stm32g474re_agv.hpp"

#include "core/api/voltage_monitor_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

extern "C"
{
#include "main.h"
extern ADC_HandleTypeDef hadc1;
}

namespace
{
  static platform_stm32_hal::voltage_adc_handle k_voltage_adc_handle = { static_cast<void *>(&hadc1), 5U };

  static const voltage_monitor_api::adc_operations k_voltage_adc_ops =
  {
    platform_stm32_hal::voltage_read_adc_raw
  };

  static const voltage_monitor_api::voltage_input k_voltage_monitors[] =
  {
    { static_cast<void *>(&k_voltage_adc_handle), 11U, &k_voltage_adc_ops }
  };

  static constexpr std::size_t k_voltage_monitor_count = sizeof(k_voltage_monitors) / sizeof(k_voltage_monitors[0]);
}

void board_stm32g474re_agv_init_voltage_monitor(void)
{
  voltage_monitor_api::init(k_voltage_monitors, k_voltage_monitor_count);
}
