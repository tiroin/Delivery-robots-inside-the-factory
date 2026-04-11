#ifndef INITIATE_H_
#define INITIATE_H_

// Include libraries:
#include "Cpu.h"
#include "FreeRTOS.h"
#include "pin_mux.h"
#include "task.h"
#include "clockMan1.h"
#include "ftm_pwm_driver.h"
#include "flexTimer_pwm1.h"
#include "lpuart1.h"

// Getting the parameters needed from the Generator Expert:
extern ftm_state_t flexTimer_pwm1_State;
extern lpuart_state_t lpuart1_State;

// System init (all peripherals):
void sys_init(void);

#endif
