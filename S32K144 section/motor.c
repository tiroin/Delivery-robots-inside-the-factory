#include "motor.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

uint16_t target_L = 0, target_R = 0;
uint16_t current_L = 0, current_R = 0;
static uint8_t dir_left = 0, dir_right = 0;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Apply directions:
static void apply_direction(uint8_t left_dir, uint8_t right_dir) {
    dir_left = left_dir; dir_right = right_dir;
    if (left_dir)  PINS_DRV_SetPins(LEFT_DIR_PORT, 1U << LEFT_DIR_PIN);
    else           PINS_DRV_ClearPins(LEFT_DIR_PORT, 1U << LEFT_DIR_PIN);
    if (right_dir) PINS_DRV_SetPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);
    else           PINS_DRV_ClearPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);
}

// Initiate motor:
void motor_init(void) {
    apply_direction(0, 0);
    current_L = 0; current_R = 0;
    target_L = 0; target_R = 0;
    set_speed_motors(0, 0);
}

// Set speed:
void set_speed_motors(uint16_t speed_left, uint16_t speed_right) {
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, LEFT_MOTOR_PWM, FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_left, 0U, true);
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, RIGHT_MOTOR_PWM, FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_right, 0U, true);
}

// Update ramp:
void update_motor_ramp(void) {
    // Left Motor Ramp
    if (current_L < target_L) {
        current_L = ((uint32_t)current_L + RAMP_STEP >= target_L) ? target_L : current_L + RAMP_STEP;
    } else if (current_L > target_L) {
        current_L = (current_L <= RAMP_STEP) ? 0 : current_L - RAMP_STEP;
    }
    // Right Motor Ramp
    if (current_R < target_R) {
        current_R = ((uint32_t)current_R + RAMP_STEP >= target_R) ? target_R : current_R + RAMP_STEP;
    } else if (current_R > target_R) {
        current_R = (current_R <= RAMP_STEP) ? 0 : current_R - RAMP_STEP;
    }
    // Set speed for both motors:
    set_speed_motors(current_L, current_R);
}

// Moving according to directions:
void move_forward(void) {
	apply_direction(0, 0);
	target_L = MAX_SPEED;
	target_R = MAX_SPEED;
}
void move_backward(void) {
	apply_direction(1, 1);
	target_L = MAX_SPEED;
	target_R = MAX_SPEED;
}
void turn_left(void) {
	apply_direction(0, 0);
	target_L = 800U;
	target_R = TURN_SPEED;
}
void turn_right(void) {
	apply_direction(0, 0);
	target_L = TURN_SPEED;
	target_R = 800U;
}
void stop_robot(void) {
	target_L = 0;
	target_R = 0;
}
