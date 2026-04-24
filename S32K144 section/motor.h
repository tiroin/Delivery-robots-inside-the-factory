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

// Throttle handle voltage: 0.8V -> 2.5V @ V_cc = 3.3V
// duty = (V / 3.3) * 20000
#define HANDLE_MIN          	4848U   // 0.8V
#define HANDLE_MAX          	15152U  // 2.5V

// Motor speed:
#define FTM_PERIOD          	20000U
#define MAX_SPEED           	5000U
#define TURN_SPEED          	5000U
#define TURN_INNER          	HANDLE_MIN
#define RAMP_STEP           	500U
#define MIN_RUNNING_SPEED   	HANDLE_MIN
#define RAMP_DOWN_STEPS      	((MAX_SPEED  + RAMP_STEP - 1) / RAMP_STEP)
#define RAMP_DOWN_STEPS_TURN 	((TURN_SPEED + RAMP_STEP - 1) / RAMP_STEP)

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
// SCALE FACTOR:
// ----------------------------------------------------
#define PULSE_PER_PERIOD_L    	5.2f
#define PULSE_PER_PERIOD_R    	6.0f
#define ACC_WINDOW			 	4
#define SPEED_DEADZONE 			0.02f

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

// PID instances:
extern PID_t pid_L;
extern PID_t pid_R;

// =========================
// CONFIG
// =========================
#define STOP_THRESHOLD   200.0f
#define MIN_DUTY         0.08f
#define LOW_SPEED_GAIN   0.8f

// motor gain (IMPORTANT)
#define MOTOR_GAIN_L     1.15f
#define MOTOR_GAIN_R     1.00f

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
