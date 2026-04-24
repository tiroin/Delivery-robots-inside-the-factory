#include "motor.h"
#include "pins_driver.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

uint16_t target_L = 0, target_R = 0;
uint16_t current_L = 0, current_R = 0;
static uint8_t dir_left = 0, dir_right = 0;
volatile uint32_t pulse_count_L = 0;
volatile uint32_t pulse_count_R = 0;
float actual_L_val = 0.0f;
float actual_R_val = 0.0f;
float duty_L_dbg = 0.0f, duty_R_dbg = 0.0f;

// PID instances:
PID_t pid_L, pid_R;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

static void apply_direction(uint8_t left_dir, uint8_t right_dir) {
    dir_left = left_dir; dir_right = right_dir;
    if (left_dir)  PINS_DRV_SetPins(LEFT_DIR_PORT, 1U << LEFT_DIR_PIN);
    else           PINS_DRV_ClearPins(LEFT_DIR_PORT, 1U << LEFT_DIR_PIN);
    if (right_dir) PINS_DRV_SetPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);
    else           PINS_DRV_ClearPins(RIGHT_DIR_PORT, 1U << RIGHT_DIR_PIN);
}

void motor_init(void) {
    apply_direction(0, 0);
    current_L = 0; current_R = 0;
    target_L  = 0; target_R  = 0;
    set_speed_motors(0, 0);
    // PID initiate:
    PID_Init(&pid_L, PID_KP_L, PID_KI_L, PID_KD_L, -0.5f, 0.5f);
    PID_Init(&pid_R, PID_KP_R, PID_KI_R, PID_KD_R, -0.5f, 0.5f);
}

void set_speed_motors(uint16_t speed_left, uint16_t speed_right) {
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, LEFT_MOTOR_PWM,  FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_left,  0U, true);
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, RIGHT_MOTOR_PWM, FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_right, 0U, true);
}

uint8_t motors_stopped(void) {
    return (current_L == 0U && current_R == 0U) ? 1U : 0U;
}

// Hall sensor handler:
void Hall_Sensor_Handler(void) {
    // Interrupt flag:
    uint32_t interruptFlags = PINS_DRV_GetPortIntFlag(PORTD);

    // Left hall:
    if ((interruptFlags >> LEFT_HALL_PIN) & 1U) {
        pulse_count_L++;
        PORTD->ISFR = (1U << LEFT_HALL_PIN);
    }

    // Right hall:
    if ((interruptFlags >> RIGHT_HALL_PIN) & 1U) {
        pulse_count_R++;
        PORTD->ISFR = (1U << RIGHT_HALL_PIN);
    }
}

void update_motor_ramp(void) {
    // =========================
    // RAMP TARGET
    // =========================
    if (current_L < target_L) {
        current_L = ((uint32_t)current_L + RAMP_STEP >= target_L) ? target_L : current_L + RAMP_STEP;
    } else if (current_L > target_L) {
        current_L = (current_L <= RAMP_STEP) ? 0U : current_L - RAMP_STEP;
    }

    if (current_R < target_R) {
        current_R = ((uint32_t)current_R + RAMP_STEP >= target_R) ? target_R : current_R + RAMP_STEP;
    } else if (current_R > target_R) {
        current_R = (current_R <= RAMP_STEP) ? 0U : current_R - RAMP_STEP;
    }

    // =========================
    // FEEDBACK (pulse average)
    // =========================
    static uint32_t acc_L = 0, acc_R = 0;
    static uint8_t  acc_cnt = 0;

    uint32_t snap_L = pulse_count_L;
    uint32_t snap_R = pulse_count_R;
    pulse_count_L = 0;
    pulse_count_R = 0;

    acc_L += snap_L;
    acc_R += snap_R;
    acc_cnt++;

    if (acc_cnt >= ACC_WINDOW) {
        actual_L_val = (float)acc_L / ACC_WINDOW;
        actual_R_val = (float)acc_R / ACC_WINDOW;

        acc_L = 0;
        acc_R = 0;
        acc_cnt = 0;
    }

    // =========================
    // LOW-PASS FILTER
    // =========================
    static float filt_L = 0.0f, filt_R = 0.0f;

    filt_L = 0.8f * filt_L + 0.2f * actual_L_val;
    filt_R = 0.8f * filt_R + 0.2f * actual_R_val;

    actual_L_val = filt_L;
    actual_R_val = filt_R;

    // =========================
    // NORMALIZE
    // =========================
    float target_norm_L = (float)current_L / (float)MAX_SPEED;
    float target_norm_R = (float)current_R / (float)MAX_SPEED;

    float speed_norm_L  = actual_L_val / PULSE_PER_PERIOD_L;
    float speed_norm_R  = actual_R_val / PULSE_PER_PERIOD_R;

    // clamp
    if (speed_norm_L > 1.0f) speed_norm_L = 1.0f;
    if (speed_norm_L < 0.0f) speed_norm_L = 0.0f;

    if (speed_norm_R > 1.0f) speed_norm_R = 1.0f;
    if (speed_norm_R < 0.0f) speed_norm_R = 0.0f;

    // =========================
    // DUTY
    // =========================
    static float duty_L = 0.3f;
    static float duty_R = 0.3f;

    float delta_L = 0.0f;
    float delta_R = 0.0f;

    // =========================
    // STOP REGION
    // =========================
    if (target_L == 0U && target_R == 0U &&
        actual_L_val < STOP_THRESHOLD &&
        actual_R_val < STOP_THRESHOLD)
    {
        duty_L = 0.0f;
        duty_R = 0.0f;

        PID_Reset(&pid_L);
        PID_Reset(&pid_R);
    }
    else
    {
        // PID
        delta_L = PID_Compute(&pid_L, target_norm_L, speed_norm_L, PID_DT);
        delta_R = PID_Compute(&pid_R, target_norm_R, speed_norm_R, PID_DT);

        // low-speed soften
        if (speed_norm_L < 0.2f) delta_L *= LOW_SPEED_GAIN;
        if (speed_norm_R < 0.2f) delta_R *= LOW_SPEED_GAIN;

        duty_L += delta_L;
        duty_R += delta_R;
    }

    // =========================
    // CLAMP DUTY
    // =========================
    if (duty_L > 1.0f) duty_L = 1.0f;
    if (duty_L < 0.0f) duty_L = 0.0f;

    if (duty_R > 1.0f) duty_R = 1.0f;
    if (duty_R < 0.0f) duty_R = 0.0f;

    // =========================
    // MIN DUTY
    // =========================
    if (duty_L > 0.0f && duty_L < MIN_DUTY) duty_L = MIN_DUTY;
    if (duty_R > 0.0f && duty_R < MIN_DUTY) duty_R = MIN_DUTY;

    // =========================
    // APPLY MOTOR GAIN
    // =========================
    float duty_L_comp = duty_L * MOTOR_GAIN_L;
    float duty_R_comp = duty_R * MOTOR_GAIN_R;

    duty_L_dbg = duty_L_comp;
    duty_R_dbg = duty_R_comp;

    if (duty_L_comp > 1.0f) duty_L_comp = 1.0f;
    if (duty_R_comp > 1.0f) duty_R_comp = 1.0f;

    // =========================
    // DUTY -> PWM
    // =========================
    uint16_t pwm_L = 0;
    uint16_t pwm_R = 0;

    if (current_L > 0U) {
        pwm_L = HANDLE_MIN + (uint16_t)(duty_L_comp * (HANDLE_MAX - HANDLE_MIN));
    }

    if (current_R > 0U) {
        pwm_R = HANDLE_MIN + (uint16_t)(duty_R_comp * (HANDLE_MAX - HANDLE_MIN));
    }

    set_speed_motors(pwm_L, pwm_R);
}

// ----------------------------------------------------
// MOVEMENT COMMANDS:
// ----------------------------------------------------

void move_forward(void) {
    apply_direction(0, 0);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = MAX_SPEED;
    target_R = MAX_SPEED;
}

void move_backward(void) {
    apply_direction(1, 1);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = MAX_SPEED;
    target_R = MAX_SPEED;
}

void turn_left(void) {
    apply_direction(0, 0);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = 0U;
    target_R = TURN_SPEED;
}

void turn_right(void) {
    apply_direction(0, 0);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = TURN_SPEED;
    target_R = 0U;
}

void stop_robot(void) {
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = 0U;
    target_R = 0U;
}
