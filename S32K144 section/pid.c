#include "pid.h"

// ----------------------------------------------------
// PID FUNCTIONS:
// ----------------------------------------------------

// Initiate PID:
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max) {
    pid->Kp         = Kp;
    pid->Ki         = Ki;
    pid->Kd         = Kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

// Reset PID:
void PID_Reset(PID_t *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

// Compute PID:
float PID_Compute(PID_t *pid, float target, float actual, float dt) {
    float error = target - actual;

    // P
    float P = pid->Kp * error;

    // I (with clamp):
    pid->integral += error * dt;
    if (pid->integral > 2.0f) pid->integral = 2.0f;
    if (pid->integral < -2.0f) pid->integral = -2.0f;
    float I = pid->Ki * pid->integral;

    // D:
    float D = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // P + I + D:
    float output = P + I + D;

    // Clamp (within its range):
    if (output > pid->output_max) {
        output = pid->output_max;
        if (error > 0.0f) pid->integral -= error * dt;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        if (error < 0.0f) pid->integral -= error * dt;
    }

    return output;
}
