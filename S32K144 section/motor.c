#include "motor.h"

uint16_t target_L  = 0;
uint16_t target_R  = 0;
uint16_t current_L = 0;
uint16_t current_R = 0;

static uint8_t dir_left  = 0;
static uint8_t dir_right = 0;

/* ------------------------------------------------------------------ */

static void apply_direction(uint8_t left_dir, uint8_t right_dir) {
    dir_left  = left_dir;
    dir_right = right_dir;

    if (left_dir)  PINS_DRV_SetPins(LEFT_DIR_PORT,  1U << LEFT_DIR_PIN);
    else           PINS_DRV_ClearPins(LEFT_DIR_PORT, 1U << LEFT_DIR_PIN);

    if (right_dir) PINS_DRV_SetPins(RIGHT_DIR_PORT,  1U << RIGHT_DIR_PIN);
    else           PINS_DRV_ClearPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);
}

/* ------------------------------------------------------------------ */

void motor_init(void) {
    apply_direction(0, 0);
    current_L = 0; current_R = 0;
    target_L  = 0; target_R  = 0;
    set_speed_motors(0, 0);
}

void set_speed_motors(uint16_t speed_left, uint16_t speed_right) {
    if (speed_left  > FTM_PERIOD) speed_left  = FTM_PERIOD;
    if (speed_right > FTM_PERIOD) speed_right = FTM_PERIOD;
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, LEFT_MOTOR_PWM,
                             FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_left,  0U, true);
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, RIGHT_MOTOR_PWM,
                             FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_right, 0U, true);
}

void update_motor_ramp(void) {
    uint16_t prev_L = current_L;
    uint16_t prev_R = current_R;

    /* Ramp left */
    if (current_L < target_L) {
        uint16_t step = (current_L < MIN_RUNNING_SPEED) ? RAMP_STEP / 4 : RAMP_STEP;
        current_L = ((uint32_t)current_L + step >= target_L) ? target_L : current_L + step;
    } else if (current_L > target_L) {
        current_L = (current_L <= RAMP_STEP) ? 0 : current_L - RAMP_STEP;
    }

    /* Ramp right */
    if (current_R < target_R) {
        uint16_t step = (current_R < MIN_RUNNING_SPEED) ? RAMP_STEP / 4 : RAMP_STEP;
        current_R = ((uint32_t)current_R + step >= target_R) ? target_R : current_R + step;
    } else if (current_R > target_R) {
        current_R = (current_R <= RAMP_STEP) ? 0 : current_R - RAMP_STEP;
    }

    if (current_L != prev_L || current_R != prev_R)
        set_speed_motors(current_L, current_R);
}

uint8_t motors_stopped(void) {
    return (current_L == 0 && current_R == 0) ? 1U : 0U;
}

/* ------------------------------------------------------------------ */
static void safe_set_direction(uint8_t left_dir, uint8_t right_dir) {
    if (dir_left == left_dir && dir_right == right_dir && motors_stopped())
        return;

    target_L = 0;
    target_R = 0;
    uint32_t timeout = 500;
    while (!motors_stopped() && timeout--) {
        update_motor_ramp();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    apply_direction(left_dir, right_dir);
}

/* ------------------------------------------------------------------ */

void move_forward(void) {
    safe_set_direction(0, 0);
    target_L = MAX_SPEED;
    target_R = MAX_SPEED;
}

void move_backward(void) {
    safe_set_direction(1, 1);
    target_L = MAX_SPEED;
    target_R = MAX_SPEED;
}

void stop_robot(void) {
    target_L = 0;
    target_R = 0;
}

void turn_left(void) {
    safe_set_direction(0, 0);
    target_L = 0;
    target_R = TURN_SPEED;
}

void turn_right(void) {
    safe_set_direction(0, 0);
    target_L = TURN_SPEED;
    target_R = 0;
}
