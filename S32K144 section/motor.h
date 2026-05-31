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
#define MAX_SPEED_L           	170U
#define MAX_SPEED_R				170U
#define MIN_RUNNING_SPEED   	65U

// Turning speed:
#define TURN_SPEED_L          	120U
#define TURN_SPEED_R          	120U
#define TURN_INNER          	70U
#define TURN_INNER_PERCENT   	55U
#define TURN_OUTER_PERCENT   	95U

// Yaw hold outer loop:
#define YAW_HOLD_ENABLE           1U
#define YAW_HOLD_BACKWARD_ENABLE  0U
#define YAW_CORR_SIGN             1
#define YAW_KP                    1.00f
#define YAW_KD                    0.04f
#define YAW_CORR_LIMIT            20


// Wheel balance trims:
#define SETPOINT_TRIM_L_PERCENT   85U
#define SETPOINT_TRIM_R_PERCENT   115U
#define DUTY_TRIM_L               0.85f
#define DUTY_TRIM_R               1.15f

// Per-wheel minimum duty:
#define MIN_DUTY_L                0.06f
#define MIN_DUTY_R                0.10f

// ----------------------------------------------------
// RAMP:
// ----------------------------------------------------
#define RAMP_STEP_L           	10U
#define RAMP_STEP_R           	10U

#define RAMP_DOWN_STEPS_L     	((MAX_SPEED_L + RAMP_STEP_L - 1) / RAMP_STEP_L)
#define RAMP_DOWN_STEPS_R     	((MAX_SPEED_R + RAMP_STEP_R - 1) / RAMP_STEP_R)

#define RAMP_DOWN_STEPS_TURN_L 	((TURN_SPEED_L + RAMP_STEP_L - 1) / RAMP_STEP_L)
#define RAMP_DOWN_STEPS_TURN_R 	((TURN_SPEED_R + RAMP_STEP_L - 1) / RAMP_STEP_R)

// ----------------------------------------------------
// GAIN:
// ----------------------------------------------------
#define FF_GAIN_L 				(1.0f / MAX_SPEED_L)
#define FF_GAIN_R 				(1.0f / MAX_SPEED_R)
#define FF_HEADROOM  			1.0f
#define SPEED_SCALE_L         	1.515f
#define SPEED_SCALE_R         	1.250f
#define SPEED_ALPHA 			0.3f

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
#define LEFT_HALL_PIN_A       	1U   // PTD1
#define LEFT_HALL_PIN_B       	3U   // PTD3
#define LEFT_HALL_PIN_C       	4U   // PTD4

#define RIGHT_HALL_PIN_A      	2U   // PTD2
#define RIGHT_HALL_PIN_B      	13U  // PTD13
#define RIGHT_HALL_PIN_C      	14U  // PTD14

// ----------------------------------------------------
// WINDOW ACCUMULATION:
// ----------------------------------------------------
#define ACC_WINDOW			 	4

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
// Dead zone:
#define DEADZONE_L 				0.0f
#define DEADZONE_R 				0.0f
// Duty for clamping:
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

// Initiate motor parameters:
void    motor_init       	(void);
// Set motor speed (raw PWM):
void    set_speed_motors 	(uint16_t speed_left, uint16_t speed_right);
// Update motor ramp + PID:
void    update_motor_ramp	(void);
// Check if motors stop:
uint8_t motors_stopped   	(void);

// Movement commands: all accept a target speed [MIN_RUNNING_SPEED, MAX_SPEED_L]:
void    move_forward     	(uint16_t speed);   // both wheels forward at speed
void    move_backward    	(uint16_t speed);   // both wheels backward at speed
void    turn_left        	(uint16_t speed);   // right wheel at speed, left stops
void    turn_right       	(uint16_t speed);   // left wheel at speed, right stops
void    stop_robot       	(void);             // both wheels stop

// Hall sensor handler:
void 	Hall_Sensor_Handler	(void);

#endif /* MOTOR_H_ */
