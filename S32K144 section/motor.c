#include "motor.h"
#include "pins_driver.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// Set point:
uint16_t target_L = 0, target_R = 0;
uint16_t current_L = 0, current_R = 0;

// Direction:
static uint8_t dir_left = 0, dir_right = 0;

typedef enum {
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_FORWARD,
    MOTOR_MODE_BACKWARD,
    MOTOR_MODE_LEFT,
    MOTOR_MODE_RIGHT
} MotorMode_t;

static MotorMode_t motor_mode = MOTOR_MODE_STOP;
static uint16_t base_target_L = 0U;
static uint16_t base_target_R = 0U;

// Filter:
static float filtered_L = 0.0f, filtered_R = 0.0f;

// Pulse count:
volatile uint32_t pulse_count_L = 0;
volatile uint32_t pulse_count_R = 0;

// Actual values:
float actual_L_val = 0.0f;
float actual_R_val = 0.0f;

extern volatile int16_t imu_yaw_cdeg;
extern volatile int16_t imu_gz_cdps;
extern volatile uint8_t imu_angle_valid;

// PID instances:
PID_t pid_L, pid_R, pid_yaw;

static int16_t yaw_ref_cdeg = 0;
static uint16_t yaw_startup_block = 0U;

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

// Clamp speed to valid range:
static uint16_t clamp_speed(uint16_t spd) {
    if (spd < MIN_RUNNING_SPEED) spd = MIN_RUNNING_SPEED;
    if (spd > MAX_SPEED_L)       spd = MAX_SPEED_L;
    return spd;
}

static uint16_t scale_speed_percent(uint16_t spd, uint16_t percent) {
    uint32_t val = ((uint32_t)spd * (uint32_t)percent) / 100U;
    if (val > 0U && val < MIN_RUNNING_SPEED) val = MIN_RUNNING_SPEED;
    if (val > MAX_SPEED_L) val = MAX_SPEED_L;
    return (uint16_t)val;
}

static void set_motion_mode(MotorMode_t mode, uint8_t left_dir, uint8_t right_dir) {
    if (motor_mode != mode || dir_left != left_dir || dir_right != right_dir) {
        PID_Reset(&pid_L);
        PID_Reset(&pid_R);
        PID_Reset(&pid_yaw);
        filtered_L = 0.0f;
        filtered_R = 0.0f;
        apply_direction(left_dir, right_dir);
        motor_mode = mode;
        if (mode == MOTOR_MODE_FORWARD) {
            yaw_ref_cdeg = imu_yaw_cdeg;
            yaw_startup_block = YAW_STARTUP_BLOCK_CYCLES;
        } else {
            yaw_startup_block = 0U;
        }
    }
}

// Initiate motor parameters:
void motor_init(void) {
    apply_direction(0, 0);
    current_L = 0; current_R = 0;
    target_L  = 0; target_R  = 0;
    base_target_L = 0U; base_target_R = 0U;
    motor_mode = MOTOR_MODE_STOP;
    set_speed_motors(0, 0);
    // PID initiate:
    PID_Init(&pid_L, PID_KP_L, PID_KI_L, PID_KD_L, -1.0f, 1.0f);
    PID_Init(&pid_R, PID_KP_R, PID_KI_R, PID_KD_R, -1.0f, 1.0f);
    PID_Init(&pid_yaw, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, -(float)YAW_CORR_LIMIT, (float)YAW_CORR_LIMIT);
    yaw_ref_cdeg = imu_yaw_cdeg;
}

// Set motor speed through PWM:
void set_speed_motors(uint16_t speed_left, uint16_t speed_right) {
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, LEFT_MOTOR_PWM,  FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_left,  0U, true);
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, RIGHT_MOTOR_PWM, FTM_PWM_UPDATE_IN_DUTY_CYCLE, speed_right, 0U, true);
}

// Check if motors stop:
uint8_t motors_stopped(void) {
    return (current_L == 0U && current_R == 0U) ? 1U : 0U;
}

// Hall sensor handler (for counting how many pulses):
void Hall_Sensor_Handler(void) {
    // Interrupt flag:
    uint32_t interruptFlags = PINS_DRV_GetPortIntFlag(PORTD);

    // Left hall:
    if (interruptFlags & ((1U << LEFT_HALL_PIN_A) | (1U << LEFT_HALL_PIN_B) | (1U << LEFT_HALL_PIN_C))) {
        pulse_count_L++;
        PORTD->ISFR = (1U << LEFT_HALL_PIN_A) | (1U << LEFT_HALL_PIN_B) | (1U << LEFT_HALL_PIN_C);
    }

    // Right hall:
    if (interruptFlags & ((1U << RIGHT_HALL_PIN_A) | (1U << RIGHT_HALL_PIN_B) | (1U << RIGHT_HALL_PIN_C))) {
        pulse_count_R++;
        PORTD->ISFR = (1U << RIGHT_HALL_PIN_A) | (1U << RIGHT_HALL_PIN_B) | (1U << RIGHT_HALL_PIN_C);
    }
}

static int16_t clamp_i16(int16_t val, int16_t min_val, int16_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static uint16_t add_signed_to_speed(uint16_t base, int16_t delta) {
    int32_t val = (int32_t)base + (int32_t)delta;
    if (val <= 0) return 0U;
    if (val < (int32_t)MIN_RUNNING_SPEED) val = (int32_t)MIN_RUNNING_SPEED;
    if (val > (int32_t)MAX_SPEED_L) val = (int32_t)MAX_SPEED_L;
    return (uint16_t)val;
}

static float clamp_duty(float duty) {
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < 0.0f) duty = 0.0f;
    return duty;
}

static int16_t yaw_error_cdeg(int16_t ref_cdeg, int16_t now_cdeg) {
    int32_t err = (int32_t)ref_cdeg - (int32_t)now_cdeg;
    while (err > 18000) err -= 36000;
    while (err < -18000) err += 36000;
    return (int16_t)err;
}

static int16_t compute_yaw_correction(void) {
#if YAW_HOLD_ENABLE
    if (imu_angle_valid == 0U) return 0;
    if (yaw_startup_block > 0U) return 0;

    int16_t err_cdeg = yaw_error_cdeg(yaw_ref_cdeg, imu_yaw_cdeg);
    float err_deg = ((float)err_cdeg) * 0.01f;
    float gz_dps = ((float)imu_gz_cdps) * 0.01f;

    float corr_f = PID_Compute(&pid_yaw, err_deg, 0.0f, PID_DT);
    corr_f -= (YAW_GZ_GAIN * gz_dps);
    corr_f *= (float)YAW_CORR_SIGN;

    if (corr_f > (float)YAW_CORR_LIMIT) corr_f = (float)YAW_CORR_LIMIT;
    if (corr_f < -(float)YAW_CORR_LIMIT) corr_f = -(float)YAW_CORR_LIMIT;

    if (corr_f >= 0.0f) return (int16_t)(corr_f + 0.5f);
    return (int16_t)(corr_f - 0.5f);
#else
    return 0;
#endif
}

static void apply_yaw_hold_to_targets(void) {
    if (motor_mode == MOTOR_MODE_FORWARD) {
        int16_t corr = compute_yaw_correction();
        corr = clamp_i16(corr, (int16_t)-YAW_CORR_LIMIT, (int16_t)YAW_CORR_LIMIT);
        target_L = add_signed_to_speed(base_target_L, (int16_t)(-corr));
        target_R = add_signed_to_speed(base_target_R, corr);
    } else if (motor_mode == MOTOR_MODE_BACKWARD) {
#if YAW_HOLD_BACKWARD_ENABLE
        int16_t corr = compute_yaw_correction();
        corr = clamp_i16(corr, (int16_t)-YAW_CORR_LIMIT, (int16_t)YAW_CORR_LIMIT);
        target_L = add_signed_to_speed(base_target_L, corr);
        target_R = add_signed_to_speed(base_target_R, (int16_t)(-corr));
#else
        target_L = base_target_L;
        target_R = base_target_R;
#endif
    } else {
        target_L = base_target_L;
        target_R = base_target_R;
    }
}

// Update motor:
void update_motor_ramp(void) {
    if (yaw_startup_block > 0U) {
        yaw_startup_block--;
    }
    apply_yaw_hold_to_targets();

    // RAMP SETPOINT:
    // Left motor:
    if (current_L < target_L)
        current_L = (current_L + RAMP_UP_STEP_L >= target_L) ? target_L : current_L + RAMP_UP_STEP_L;
    else if (current_L > target_L)
        current_L = (current_L <= RAMP_DOWN_STEP_L) ? 0 : current_L - RAMP_DOWN_STEP_L;
    // Right motor:
    if (current_R < target_R)
        current_R = (current_R + RAMP_UP_STEP_R >= target_R) ? target_R : current_R + RAMP_UP_STEP_R;
    else if (current_R > target_R)
        current_R = (current_R <= RAMP_DOWN_STEP_R) ? 0 : current_R - RAMP_DOWN_STEP_R;

    // READ SENSOR (HALL):
    static uint32_t acc_L = 0, acc_R = 0;
    static uint8_t acc_cnt = 0;
    static float speed_L = 0.0f, speed_R = 0.0f;
    uint32_t snap_L, snap_R;

    taskENTER_CRITICAL();
    snap_L = pulse_count_L;
    snap_R = pulse_count_R;
    pulse_count_L = 0;
    pulse_count_R = 0;
    taskEXIT_CRITICAL();

    acc_L += snap_L;
    acc_R += snap_R;
    acc_cnt++;

    if (acc_cnt >= ACC_WINDOW) {
        // FILTERING:
        float raw_L = ((float)acc_L / ACC_WINDOW) * SPEED_SCALE_L;
        float raw_R = ((float)acc_R / ACC_WINDOW) * SPEED_SCALE_R;

        filtered_L = SPEED_ALPHA * raw_L + (1.0f - SPEED_ALPHA) * filtered_L;
        filtered_R = SPEED_ALPHA * raw_R + (1.0f - SPEED_ALPHA) * filtered_R;

        float clamped_L = filtered_L;
        float clamped_R = filtered_R;
        if (current_L > 0 && clamped_L > (float)current_L * 1.5f) clamped_L = (float)current_L * 1.5f;
        if (current_R > 0 && clamped_R > (float)current_R * 1.5f) clamped_R = (float)current_R * 1.5f;

        speed_L = clamped_L;
        speed_R = clamped_R;
        actual_L_val = clamped_L;
        actual_R_val = clamped_R;

        // PID ONLY
        static float duty_L = 0.0f, duty_R = 0.0f;
        duty_L = PID_Compute(&pid_L, (float)current_L, speed_L, PID_DT);
        duty_R = PID_Compute(&pid_R, (float)current_R, speed_R, PID_DT);

        // SATURATION + ANTI-WINDUP
        duty_L = clamp_duty(duty_L * DUTY_TRIM_L);
        duty_R = clamp_duty(duty_R * DUTY_TRIM_R);

        if (duty_L < MIN_DUTY_L && current_L > 0) duty_L = MIN_DUTY_L;
        if (duty_R < MIN_DUTY_R && current_R > 0) duty_R = MIN_DUTY_R;
        if (current_L == 0) { duty_L = 0.0f; PID_Reset(&pid_L); filtered_L = 0.0f; }
        if (current_R == 0) { duty_R = 0.0f; PID_Reset(&pid_R); filtered_R = 0.0f; }

        // PWM OUTPUT
        uint16_t pwm_L = 0, pwm_R = 0;
        if (duty_L > 0) {
            pwm_L = HANDLE_MIN + (uint16_t)(duty_L * (HANDLE_MAX - HANDLE_MIN));
        }
        if (duty_R > 0) {
            pwm_R = HANDLE_MIN + (uint16_t)(duty_R * (HANDLE_MAX - HANDLE_MIN));
        }

        // SET PWM TO BOTH MOTORS:
        set_speed_motors(pwm_L, pwm_R);

        // RESET:
        acc_L = acc_R = 0;
        acc_cnt = 0;
    }
}

// ----------------------------------------------------
// MOVEMENT COMMANDS:
// ----------------------------------------------------

// Moving forward:
void move_forward(uint16_t speed) {
    set_motion_mode(MOTOR_MODE_FORWARD, 0U, 0U);
    speed = clamp_speed(speed);
    base_target_L = scale_speed_percent(speed, SETPOINT_TRIM_L_PERCENT);
    base_target_R = scale_speed_percent(speed, SETPOINT_TRIM_R_PERCENT);
    apply_yaw_hold_to_targets();
}

// Moving backward:
void move_backward(uint16_t speed) {
    set_motion_mode(MOTOR_MODE_BACKWARD, 1U, 1U);
    speed = clamp_speed(speed);
    base_target_L = scale_speed_percent(speed, SETPOINT_TRIM_L_PERCENT);
    base_target_R = scale_speed_percent(speed, SETPOINT_TRIM_R_PERCENT);
    apply_yaw_hold_to_targets();
}

// Turning left:
void turn_left(uint16_t speed) {
    set_motion_mode(MOTOR_MODE_LEFT, 0U, 0U);
    speed = clamp_speed(speed);
//    base_target_L = scale_speed_percent(scale_speed_percent(speed, TURN_LEFT_INNER_PERCENT), SETPOINT_TRIM_L_PERCENT);
    base_target_R = scale_speed_percent(scale_speed_percent(speed, TURN_LEFT_OUTER_PERCENT), SETPOINT_TRIM_R_PERCENT);
    base_target_L = 0;
    target_L = base_target_L;
    target_R = base_target_R;
}

// Turning right:
void turn_right(uint16_t speed) {
    set_motion_mode(MOTOR_MODE_RIGHT, 0U, 0U);
    speed = clamp_speed(speed);
    base_target_L = scale_speed_percent(scale_speed_percent(speed, TURN_RIGHT_OUTER_PERCENT), SETPOINT_TRIM_L_PERCENT);
//    base_target_R = scale_speed_percent(scale_speed_percent(speed, TURN_RIGHT_INNER_PERCENT), SETPOINT_TRIM_R_PERCENT);
    base_target_R = 0;
    target_L = base_target_L;
    target_R = base_target_R;
}

// Stopping robot:
void stop_robot(void) {
    if (motor_mode != MOTOR_MODE_STOP) {
        PID_Reset(&pid_L);
        PID_Reset(&pid_R);
        PID_Reset(&pid_yaw);
        filtered_L = 0.0f;
        filtered_R = 0.0f;
        motor_mode = MOTOR_MODE_STOP;
    }
    base_target_L = 0U;
    base_target_R = 0U;
    target_L = 0U;
    target_R = 0U;
}
