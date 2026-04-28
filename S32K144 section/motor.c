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

// Filter:
static float filtered_L = 0.0f, filtered_R = 0.0f;

// Pulse count:
volatile uint32_t pulse_count_L = 0;
volatile uint32_t pulse_count_R = 0;

// Actual values:
float actual_L_val = 0.0f;
float actual_R_val = 0.0f;

// PID instances:
PID_t pid_L, pid_R;

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

// Initiate motor parameters:
void motor_init(void) {
    apply_direction(0, 0);
    current_L = 0; current_R = 0;
    target_L  = 0; target_R  = 0;
    set_speed_motors(0, 0);
    // PID initiate:
    PID_Init(&pid_L, PID_KP_L, PID_KI_L, PID_KD_L, -1.0f, 1.0f);
    PID_Init(&pid_R, PID_KP_R, PID_KI_R, PID_KD_R, -1.0f, 1.0f);
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

// Update motor:
void update_motor_ramp(void) {
    // RAMP SETPOINT:
    // Left motor:
	if (current_L < target_L)
        current_L = (current_L + RAMP_STEP_L >= target_L) ? target_L : current_L + RAMP_STEP_L;
    else if (current_L > target_L)
        current_L = (current_L <= RAMP_STEP_L) ? 0 : current_L - RAMP_STEP_L;
    // Right motor:
    if (current_R < target_R)
        current_R = (current_R + RAMP_STEP_R >= target_R) ? target_R : current_R + RAMP_STEP_R;
    else if (current_R > target_R)
        current_R = (current_R <= RAMP_STEP_R) ? 0 : current_R - RAMP_STEP_R;

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

    	speed_L = filtered_L;
    	speed_R = filtered_R;
    	actual_L_val = filtered_L;
    	actual_R_val = filtered_R;

        // FEEDFORWARD
        float ff_L = 0.0f;
        float ff_R = 0.0f;
        if (current_L > 0) ff_L = DEADZONE_L + (MAX_DUTY - DEADZONE_L - FF_HEADROOM) * ((float)current_L / MAX_SPEED_L);
        if (current_R > 0) ff_R = DEADZONE_R + (MAX_DUTY - DEADZONE_R - FF_HEADROOM) * ((float)current_R / MAX_SPEED_R);

        // PID
        float pid_L_out = PID_Compute(&pid_L, (float)current_L, speed_L, PID_DT);
        float pid_R_out = PID_Compute(&pid_R, (float)current_R, speed_R, PID_DT);

        // COMBINE
        float duty_L = ff_L + pid_L_out;
        float duty_R = ff_R + pid_R_out;

        // SATURATION + ANTI-WINDUP
        // Left motor:
        if (duty_L > MAX_DUTY) duty_L = MAX_DUTY;
        if (duty_L < MIN_DUTY && current_L > 0) duty_L = MIN_DUTY;
        if (current_L == 0) duty_L = 0;
        // Right motor:
        if (duty_R > MAX_DUTY) duty_R = MAX_DUTY;
        if (duty_R < MIN_DUTY && current_R > 0) duty_R = MIN_DUTY;
        if (current_R == 0) duty_R = 0;

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
void move_forward(void) {
    apply_direction(0, 0);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = MAX_SPEED_L;
    target_R = MAX_SPEED_R;
}

// Moving backward:
void move_backward(void) {
    apply_direction(1, 1);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = MAX_SPEED_L;
    target_R = MAX_SPEED_R;
}

// Turning left:
void turn_left(void) {
    apply_direction(0, 0);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = 0;
    target_R = TURN_SPEED_R;
}

// Moving right:
void turn_right(void) {
    apply_direction(0, 0);
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = TURN_SPEED_L;
    target_R = 0;
}

// Stopping robot:
void stop_robot(void) {
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    target_L = 0U;
    target_R = 0U;
    filtered_L = 0.0f;
    filtered_R = 0.0f;
}
