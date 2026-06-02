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
#define HANDLE_MAX_L        	8500U
#define HANDLE_MIN_R        	1000U
#define HANDLE_MAX_R        	9000U

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
// Direction-specific turn tuning.
// Observed behaviour: RIGHT turn is stronger than LEFT turn.
// LEFT turn therefore uses a small reverse inner wheel to help it rotate,
// while RIGHT turn keeps the inner wheel moving forward to soften the turn.
#define TURN_LEFT_USE_REVERSE_INNER   1U
#define TURN_RIGHT_USE_REVERSE_INNER  0U
#define TURN_LEFT_INNER_PERCENT       52U
#define TURN_LEFT_OUTER_PERCENT       145U
#define TURN_RIGHT_INNER_PERCENT      45U
#define TURN_RIGHT_OUTER_PERCENT      68U

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
// LEFT -> FORWARD needs a stronger release because the robot keeps the left-turn arc.
// RIGHT -> FORWARD was already acceptable, so keep it gentler.
#define TURN_TO_FORWARD_SETTLE_CYCLES       14U
#define TURN_LEFT_TO_FORWARD_SETTLE_CYCLES  48U
#define TURN_RIGHT_TO_FORWARD_SETTLE_CYCLES 14U
#define TURN_TO_FORWARD_CAPTURE_DELAY       8U
#define TURN_TO_FORWARD_GZ_STABLE_CDPS      120
#define TURN_TO_FORWARD_BALANCE_MARGIN      4
#define TURN_TO_FORWARD_CATCHUP_CORR        10
#define TURN_TO_FORWARD_SLOWDOWN_CORR       8
#define TURN_LEFT_TO_FORWARD_CATCHUP_CORR   70
#define TURN_LEFT_TO_FORWARD_SLOWDOWN_CORR  60
#define TURN_RIGHT_TO_FORWARD_CATCHUP_CORR  18
#define TURN_RIGHT_TO_FORWARD_SLOWDOWN_CORR 14

// Extra LEFT -> FORWARD release stage.
// When LEFT turn used reverse inner wheel, the robot may keep the left arc if FORWARD is pressed immediately.
// During the first cycles, force the left wheel high and the right wheel low to cancel the remaining left rotation.
#define TURN_LEFT_TO_FORWARD_HARD_RELEASE_CYCLES  18U
#define TURN_LEFT_TO_FORWARD_HARD_L_TARGET        MAX_SPEED_L
#define TURN_LEFT_TO_FORWARD_HARD_R_TARGET        MIN_RUNNING_SPEED
#define TURN_TO_FORWARD_CAPTURE_STABLE_CYCLES     4U
#define TURN_TO_FORWARD_CAPTURE_TIMEOUT_CYCLES    60U

// Legacy PID values are kept for future use, but the current yaw correction uses the if/else curve below.
#define YAW_PID_KP                0.58f
#define YAW_PID_KI                0.000f
#define YAW_PID_KD                0.000f
#define YAW_GZ_GAIN               0.020f
#define YAW_CORR_LIMIT            20
#define YAW_STARTUP_BLOCK_CYCLES  0U
#define FORWARD_START_BIAS         0
#define FORWARD_START_BIAS_CYCLES  4U
#define YAW_DEADBAND_CDEG          12
#define YAW_CORR_SLEW_STEP         4

// If/else yaw correction curve.
// Positive yaw error produces negative correction.
// Negative yaw error produces positive correction.
// Increase these values if angle compensation is still weak.
#define YAW_IF_SMALL_CDEG          100
#define YAW_IF_MEDIUM_CDEG         200
#define YAW_IF_LARGE_CDEG          400
#define YAW_IF_SMALL_CORR          3
#define YAW_IF_MEDIUM_CORR         7
#define YAW_IF_LARGE_CORR          11
#define YAW_IF_MAX_CORR            16

// Asymmetric gain tuning.
// Tune here if one direction is weaker than the other.
// If positive yaw is weak, increase YAW_IF_POS_GAIN.
// If negative yaw is weak, increase YAW_IF_NEG_GAIN.
#define YAW_IF_POS_GAIN            1.20f
#define YAW_IF_NEG_GAIN            1.15f

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
#define SETPOINT_TRIM_L_PERCENT   96U
#define SETPOINT_TRIM_R_PERCENT   104U
#define DUTY_TRIM_L               0.95f
#define DUTY_TRIM_R               1.05f

// Per-wheel minimum duty:
#define MIN_DUTY_L                0.090f
#define MIN_DUTY_R                0.110f

// ----------------------------------------------------
// RAMP:
// ----------------------------------------------------
#define RAMP_UP_STEP_L          12U
#define RAMP_UP_STEP_R          15U
#define RAMP_DOWN_STEP_L        14U
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
#define SPEED_ALPHA 			0.25f

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
void    turn_left        	(uint16_t speed);   // left turn, tuned for stronger/smoother recovery
void    turn_right       	(uint16_t speed);   // right turn, softened to match left turn
void    stop_robot       	(void);             // both wheels stop

// Hall sensor handler:
void 	Hall_Sensor_Handler	(void);

#endif /* MOTOR_H_ */
