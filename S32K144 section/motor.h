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
#define TURN_INNER_PERCENT   	60U
#define TURN_OUTER_PERCENT   	90U
#define TURN_LEFT_INNER_PERCENT  68U
#define TURN_LEFT_OUTER_PERCENT  90U
#define TURN_RIGHT_INNER_PERCENT 68U
#define TURN_RIGHT_OUTER_PERCENT 88U

// Yaw hold outer loop:
#define YAW_HOLD_ENABLE           1U
#define YAW_HOLD_BACKWARD_ENABLE  0U
#define YAW_CORR_SIGN             1
#define YAW_PID_KP                0.45f
#define YAW_PID_KI                0.07f
#define YAW_PID_KD                0.03f
#define YAW_GZ_GAIN               0.010f
#define YAW_CORR_LIMIT            8
#define YAW_STARTUP_BLOCK_CYCLES  20U

/*
// ----------------------------------------------------
// ACCELERATION / SLOPE COMPENSATION PLACEHOLDER:
// ----------------------------------------------------
// This block is intentionally commented out.
// Use it later if ESP32 sends accelerometer or pitch data to S32K.
// Suggested use:
//   - yaw/gz keeps the robot going straight.
//   - accel or pitch compensates slope/load when going uphill or downhill.

#define ACCEL_COMP_ENABLE          0U
#define ACCEL_COMP_SIGN            1
#define ACCEL_KP                   0.00f
#define ACCEL_KD                   0.00f
#define ACCEL_CORR_LIMIT           10

// Expected future units:
// imu_ax_mg: acceleration X in milli-g.
// imu_az_mg: acceleration Z in milli-g.
// imu_pitch_cdeg: pitch angle in 0.01 degree.
// imu_pitch_rate_cdps: pitch rate in 0.01 degree per second.
*/

// Wheel balance trims:
#define SETPOINT_TRIM_L_PERCENT   96U
#define SETPOINT_TRIM_R_PERCENT   106U
#define DUTY_TRIM_L               0.94f
#define DUTY_TRIM_R               1.10f

// Per-wheel minimum duty:
#define MIN_DUTY_L                0.090f
#define MIN_DUTY_R                0.120f

// ----------------------------------------------------
// RAMP:
// ----------------------------------------------------
#define RAMP_UP_STEP_L          13U
#define RAMP_UP_STEP_R          15U
#define RAMP_DOWN_STEP_L        16U
#define RAMP_DOWN_STEP_R        15U

#define RAMP_STEP_L             RAMP_UP_STEP_L
#define RAMP_STEP_R             RAMP_UP_STEP_R

#define RAMP_DOWN_STEPS_L       ((MAX_SPEED_L + RAMP_DOWN_STEP_L - 1U) / RAMP_DOWN_STEP_L)
#define RAMP_DOWN_STEPS_R       ((MAX_SPEED_R + RAMP_DOWN_STEP_R - 1U) / RAMP_DOWN_STEP_R)

#define RAMP_DOWN_STEPS_TURN_L  ((TURN_SPEED_L + RAMP_DOWN_STEP_L - 1U) / RAMP_DOWN_STEP_L)
#define RAMP_DOWN_STEPS_TURN_R  ((TURN_SPEED_R + RAMP_DOWN_STEP_R - 1U) / RAMP_DOWN_STEP_R)

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
extern PID_t pid_yaw;

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
