// Include necessary libraries:
#include "initiate.h"
#include "motor.h"

// PWM state:
ftm_state_t flexTimer_pwm1_State;

// System init - all peripherals:
void sys_init(void) {
    CLOCK_DRV_Init(&clockMan1_InitConfig0);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    PINS_DRV_ClearPins(LEFT_DIR_PORT,  1U << LEFT_DIR_PIN);
    PINS_DRV_ClearPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);

    LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);

    FTM_DRV_Init(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_InitConfig, &flexTimer_pwm1_State);
    FTM_DRV_InitPwm(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_PwmConfig);

    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 1U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, 0U, 0U, true);
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 2U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, 0U, 0U, true);
}
