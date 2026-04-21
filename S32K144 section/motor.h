#ifndef MOTOR_H_
#define MOTOR_H_

// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------
#include "Cpu.h"
#include "FreeRTOS.h"
#include "pin_mux.h"
#include "task.h"
#include "clockMan1.h"
#include "ftm_pwm_driver.h"
#include <stdio.h>
#include <string.h>
#include "lpuart1.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// Define both PWM channel indices for 2 motors:
#define LEFT_MOTOR_PWM      1U
#define RIGHT_MOTOR_PWM     2U

// Speed variables (exposed for monitoring):
extern uint16_t target_L;
extern uint16_t target_R;
extern uint16_t current_L;
extern uint16_t current_R;

#define FTM_PERIOD          20000U
#define MAX_SPEED           2000U
#define TURN_SPEED          1500U
#define RAMP_STEP           40U
#define MIN_RUNNING_SPEED   1000U

#define RAMP_DOWN_STEPS      ((MAX_SPEED  + RAMP_STEP - 1) / RAMP_STEP)
#define RAMP_DOWN_STEPS_TURN ((TURN_SPEED + RAMP_STEP - 1) / RAMP_STEP)

// DIR pins:
#define LEFT_DIR_PORT       PTD
#define LEFT_DIR_PIN        9U
#define RIGHT_DIR_PORT      PTD
#define RIGHT_DIR_PIN       10U

// Steps:
#define STEPS_FORWARD   250
#define STEPS_TURN      150
#define STEPS_BACKWARD  200

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

void set_speed_motors(uint16_t speed_left, uint16_t speed_right);
void update_motor_ramp(void);
uint8_t motors_stopped(void);
void motor_init(void);
void move_forward(void);
void move_backward(void);
void stop_robot(void);
void turn_left(void);
void turn_right(void);

#endif /* MOTOR_H_ */
