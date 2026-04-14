// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------

#include "initiate.h"
#include "motor.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// PWM state:
ftm_state_t flexTimer_pwm1_State;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Set all values to 0 for startup:
void S32K144EVB_startup(void) {
    // Reset pins (LOW):
    PINS_DRV_ClearPins(LEFT_DIR_PORT,  1U << LEFT_DIR_PIN);
    PINS_DRV_ClearPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);

    // Reset duty cycle (duty = 0):
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 1U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, 0U, 0U, true);
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 2U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, 0U, 0U, true);
}

// System initiation - all peripherals:
void sys_init(void) {
	// Initiate clock configuration:
    CLOCK_DRV_Init(&clockMan1_InitConfig0);

    // Initiate GPIOs:
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    // Initiate UART configuration:
    LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);

    // Initiate PWM configuration:
    FTM_DRV_Init(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_InitConfig, &flexTimer_pwm1_State);
    FTM_DRV_InitPwm(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_PwmConfig);
}
