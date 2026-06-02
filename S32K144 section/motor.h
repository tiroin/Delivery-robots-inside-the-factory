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
#define HANDLE_MIN_L        	1000U
#define HANDLE_MAX_L        	8600U
#define HANDLE_MIN_R        	1000U
#define HANDLE_MAX_R        	9000U

// Period:
#define FTM_PERIOD          	20000U

// Motor speed:
#define MAX_SPEED_L           	170U
#define MAX_SPEED_R				170U
#define MIN_RUNNING_SPEED   	65U

// Turning speed:
#define TURN_SPEED_L          	140U
#define TURN_SPEED_R          	140U
#define TURN_INNER          	70U
#define TURN_INNER_PERCENT   	60U
#define TURN_OUTER_PERCENT   	90U
#define TURN_LEFT_INNER_PERCENT  35U
#define TURN_LEFT_OUTER_PERCENT  170U
#define TURN_RIGHT_INNER_PERCENT 35U
#define TURN_RIGHT_OUTER_PERCENT 145U

// Yaw hold outer loop:
#define YAW_HOLD_ENABLE           1U
#define YAW_HOLD_BACKWARD_ENABLE  0U
#define YAW_CORR_SIGN             1

// Set to 0 when STOP -> FORWARD must return to the ESP32 zero yaw angle.
// Set to 1 only if every FORWARD command should use the current yaw as the new straight reference.
#define YAW_REF_CAPTURE_ON_FORWARD 0U
#define YAW_FIXED_REF_CDEG         0

// Set to 1 so LEFT/RIGHT -> FORWARD captures the current yaw as the new straight heading.
// This prevents the robot from trying to return to the old zero heading after a turn.
#define YAW_REF_CAPTURE_AFTER_TURN 1U

// LEFT/RIGHT -> FORWARD transition control.
// The robot first balances the wheel setpoints and waits briefly, then captures a new yaw reference.
#define TURN_TO_FORWARD_SETTLE_ENABLE     1U
#define TURN_TO_FORWARD_SETTLE_CYCLES     14U
#define TURN_TO_FORWARD_CAPTURE_DELAY     8U
#define TURN_TO_FORWARD_GZ_STABLE_CDPS    120
#define TURN_TO_FORWARD_BALANCE_MARGIN    4
#define TURN_TO_FORWARD_CATCHUP_CORR      10
#define TURN_TO_FORWARD_SLOWDOWN_CORR     8

// Legacy PID values are kept for future use, but the current yaw correction uses the if/else curve below.
#define YAW_PID_KP                1.02f
#define YAW_PID_KI                0.02f
#define YAW_PID_KD                0.000f
#define YAW_GZ_GAIN               0.025f
#define YAW_CORR_LIMIT            12
#define YAW_STARTUP_BLOCK_CYCLES  0U
#define FORWARD_START_BIAS         0
#define FORWARD_START_BIAS_CYCLES  4U

// Straight-line mechanical drift trim only for FORWARD.
// Positive value fixes right drift: target_L -= corr, target_R += corr.
// If the robot drifts left, reduce this value or make it negative.
#define FORWARD_RIGHT_DRIFT_CORR   8

#define YAW_DEADBAND_CDEG          20
#define YAW_CORR_SLEW_STEP         2

// If/else yaw correction curve.
// Positive yaw error produces negative correction.
// Negative yaw error produces positive correction.
// Increase these values if angle compensation is still weak.
#define YAW_IF_SMALL_CDEG          100
#define YAW_IF_MEDIUM_CDEG         200
#define YAW_IF_LARGE_CDEG          400
#define YAW_IF_SMALL_CORR          1
#define YAW_IF_MEDIUM_CORR         5
#define YAW_IF_LARGE_CORR          7
#define YAW_IF_MAX_CORR            10

// Asymmetric gain tuning.
// Tune here if one direction is weaker than the other.
// If positive yaw is weak, increase YAW_IF_POS_GAIN.
// If negative yaw is weak, increase YAW_IF_NEG_GAIN.
#define YAW_IF_POS_GAIN            1.20f
#define YAW_IF_NEG_GAIN            1.00f

// Gyro damping.
// Increase this if the robot overshoots after correcting a large yaw error.
// Decrease this if the correction feels too slow.
#define YAW_IF_GZ_GAIN             0.015f

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
#define SETPOINT_TRIM_L_PERCENT   100U
#define SETPOINT_TRIM_R_PERCENT   100U
#define DUTY_TRIM_L               1.00f
#define DUTY_TRIM_R               1.00f

// Per-wheel minimum duty:
#define MIN_DUTY_L                0.100f
#define MIN_DUTY_R                0.100f

// ----------------------------------------------------
// RAMP:
// ----------------------------------------------------
#define RAMP_UP_STEP_L          16U
#define RAMP_UP_STEP_R          14U
#define RAMP_DOWN_STEP_L        14U
#define RAMP_DOWN_STEP_R        16U

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
#define SPEED_ALPHA 			0.25f
/*
 * Forward wheel-speed synchronizer.
 * Yaw hold corrects heading angle, but this block corrects actual wheel feedback mismatch.
 *
 * Rule:
 *   actual_R > actual_L  -> increase L target, decrease R target.
 *   actual_L > actual_R  -> increase R target, decrease L target.
 */
#define WHEEL_SYNC_ENABLE          1U
#define WHEEL_SYNC_MIN_FB          8.0f
#define WHEEL_SYNC_DEADBAND        3.0f
#define WHEEL_SYNC_KP              0.45f
#define WHEEL_SYNC_CORR_LIMIT      22

// Use these with debug values to distinguish hardware from software.
#define WHEEL_FB_FAULT_MIN_FAST    25.0f
#define WHEEL_FB_FAULT_RATIO       5.0f


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

// Debug values for the phone/serial monitor:
extern uint16_t pwm_out_L;
extern uint16_t pwm_out_R;
extern uint32_t last_snap_L;
extern uint32_t last_snap_R;
extern int16_t wheel_sync_corr;

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
void    turn_left         	(uint16_t speed);   // left wheel slow, right wheel fast
void    turn_right        	(uint16_t speed);   // left wheel fast, right wheel slow
void    stop_robot       	(void);             // both wheels stop

// Hall sensor handler:
void 	Hall_Sensor_Handler	(void);

#endif /* MOTOR_H_ */
