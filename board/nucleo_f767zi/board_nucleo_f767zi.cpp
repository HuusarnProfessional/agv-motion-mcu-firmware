#include "board/nucleo_f767zi/board_nucleo_f767zi.hpp"
#include "app/app_entry.hpp"
#include "core/api/motor_api.hpp"
#include "platform/stm32/main_platform_stm32.hpp"

// HAL typer och handles kommer fran CubeMX-projektet
extern "C" {
#include "main.h"
extern TIM_HandleTypeDef htim1;
}

// board mapping (naming holders)

// motor 0 pwm_a: PE9  (TIM1_CH1)
static GPIO_TypeDef *k_motor0_pwm_a_port = GPIOE;
static constexpr std::uint16_t k_motor0_pwm_a_pin = GPIO_PIN_9;
static constexpr std::uint32_t k_motor0_pwm_a_channel = TIM_CHANNEL_1;

// motor 0 pwm_b: PE11 (TIM1_CH2)
static GPIO_TypeDef *k_motor0_pwm_b_port = GPIOE;
static constexpr std::uint16_t k_motor0_pwm_b_pin = GPIO_PIN_11;
static constexpr std::uint32_t k_motor0_pwm_b_channel = TIM_CHANNEL_2;

// motor 0 timer (CubeMX provides htim1)
static TIM_HandleTypeDef *k_motor0_tim = &htim1;

static const motor_api::motor_pwm2 k_motors[] =
{
  {
    { static_cast<void *>(k_motor0_tim), k_motor0_pwm_a_channel, platform_stm32_hal::get_pwm_ops() },
    { static_cast<void *>(k_motor0_tim), k_motor0_pwm_b_channel, platform_stm32_hal::get_pwm_ops() }
  }
};

static constexpr std::size_t k_motor_count = sizeof(k_motors) / sizeof(k_motors[0]);

extern "C" void board_init(void)
{
  motor_api::init(k_motors, k_motor_count);
  app_init();
}

extern "C" void board_tick(void)
{
	app_step(HAL_GetTick());
}
