#include "platform/stm32/main_platform_stm32.hpp"
#include "platform/stm32/main_platform_stm32_includes.h"

namespace platform_stm32_hal
{
  bool voltage_read_adc_raw(void *platform_handle,
                            std::uint8_t channel,
                            std::uint16_t &raw_adc_out,
                            std::uint32_t &time_ms_out)
  {
    (void)channel;
    raw_adc_out = 0U;
    time_ms_out = HAL_GetTick();

    if (platform_handle == nullptr)
    {
      return false;
    }

    voltage_adc_handle *adc_bus = static_cast<voltage_adc_handle *>(platform_handle);
    if (adc_bus->adc_handle == nullptr)
    {
      return false;
    }

#if defined(HAL_ADC_MODULE_ENABLED)
    ADC_HandleTypeDef *adc_handle = static_cast<ADC_HandleTypeDef *>(adc_bus->adc_handle);
    const std::uint32_t timeout_ms = (adc_bus->transfer_timeout_ms == 0U) ? 5U : adc_bus->transfer_timeout_ms;

    if (HAL_ADC_Start(adc_handle) != HAL_OK)
    {
      return false;
    }

    const HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(adc_handle, timeout_ms);
    if (poll_status != HAL_OK)
    {
      (void)HAL_ADC_Stop(adc_handle);
      return false;
    }

    const std::uint32_t value = HAL_ADC_GetValue(adc_handle);
    (void)HAL_ADC_Stop(adc_handle);

    raw_adc_out = static_cast<std::uint16_t>(value & 0xFFFFU);
    time_ms_out = HAL_GetTick();
    return true;
#else
    return false;
#endif
  }
}
