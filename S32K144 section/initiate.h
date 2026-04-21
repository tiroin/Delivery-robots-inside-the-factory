#ifndef INITIATE_H_
#define INITIATE_H_
// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------

#include "Cpu.h"
#include "FreeRTOS.h"
#include "pin_mux.h"
#include "task.h"
#include "clockMan1.h"
#include "ftm_pwm_driver.h"
#include "flexTimer_pwm1.h"
#include "lpuart1.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// Getting the parameters needed from the Generator Expert:
extern ftm_state_t flexTimer_pwm1_State;
extern lpuart_state_t lpuart1_State;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// System initiation (all peripherals):
void sys_init(void);
// Set all values to 0 for startup:
void S32K144EVB_startup(void);

#endif
