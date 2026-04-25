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
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include "lpuart1.h"

// ----------------------------------------------------
// SPEED PARAMETERS:
// ----------------------------------------------------

// PWM channel indices:
#define LEFT_MOTOR_PWM      	1U
#define RIGHT_MOTOR_PWM     	2U

// Handle threshold:
#define HANDLE_MIN          	1000U
#define HANDLE_MAX          	10000U

// Period:
#define FTM_PERIOD          	20000U

// Motor speed:
#define MAX_SPEED_L           	1500U
#define MAX_SPEED_R				1500U
#define MIN_RUNNING_SPEED   	900U

// Turning speed:
#define TURN_SPEED_L          	1000U
#define TURN_SPEED_R          	1000U
#define TURN_INNER          	900U

// ----------------------------------------------------
// RAMP:
// ----------------------------------------------------
#define RAMP_STEP_L           	500U
#define RAMP_STEP_R           	500U

#define RAMP_DOWN_STEPS_L     	((MAX_SPEED_L + RAMP_STEP_L - 1) / RAMP_STEP_L)
#define RAMP_DOWN_STEPS_R     	((MAX_SPEED_R + RAMP_STEP_R - 1) / RAMP_STEP_R)

#define RAMP_DOWN_STEPS_TURN_L 	((TURN_SPEED_L + RAMP_STEP_L - 1) / RAMP_STEP_L)
#define RAMP_DOWN_STEPS_TURN_R 	((TURN_SPEED_R + RAMP_STEP_L - 1) / RAMP_STEP_R)

// ----------------------------------------------------
// GAIN:
// ----------------------------------------------------
#define FF_GAIN_L 				(1.0f / MAX_SPEED_L)
#define FF_GAIN_R 				(1.0f / MAX_SPEED_R)
#define SPEED_SCALE_L 			(1500.0f / 385.0f)
#define SPEED_SCALE_R   		(1500.0f / 428.0f)
#define SPEED_ALPHA 			0.5f

// ----------------------------------------------------
// DIR PINS:
// ----------------------------------------------------
#define LEFT_DIR_PORT       	PTD
#define LEFT_DIR_PIN        	9U
#define RIGHT_DIR_PORT      	PTD
#define RIGHT_DIR_PIN       	10U

// ----------------------------------------------------
// HALL PORT:
// ----------------------------------------------------
#define LEFT_HALL_PORT      	PORTD
#define LEFT_HALL_PIN       	1U
#define RIGHT_HALL_PORT     	PORTD
#define RIGHT_HALL_PIN      	2U

// ----------------------------------------------------
// WINDOW ACCUMULATION:
// ----------------------------------------------------
#define ACC_WINDOW			 	12

// ----------------------------------------------------
// EXPOSED VARIABLES:
// ----------------------------------------------------
extern uint16_t target_L;
extern uint16_t target_R;
extern uint16_t current_L;
extern uint16_t current_R;
extern volatile uint32_t pulse_count_L;
extern volatile uint32_t pulse_count_R;
extern float actual_L_val;
extern float actual_R_val;

// ----------------------------------------------------
// DUTY:
// ----------------------------------------------------
#define DEADZONE_L 				0.12f
#define DEADZONE_R 				0.12f

#define MAX_DUTY 				1.0f
#define MIN_DUTY         		0.08f

// ----------------------------------------------------
// PID INSTANCE:
// ----------------------------------------------------
extern PID_t pid_L;
extern PID_t pid_R;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------
void    motor_init       (void);
void    set_speed_motors (uint16_t speed_left, uint16_t speed_right);
void    update_motor_ramp(void);
uint8_t motors_stopped   (void);
void    move_forward     (void);
void    move_backward    (void);
void    turn_left        (void);
void    turn_right       (void);
void    stop_robot       (void);
void 	Hall_Sensor_Handler(void);

#endif /* MOTOR_H_ */
