#ifndef PID_H_
#define PID_H_

// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------
#include <stdint.h>

// ----------------------------------------------------
// PID PARAMETERS:
// ----------------------------------------------------
#define PID_DT          0.02f

// PID parameters for the left motor:
#define PID_KP_L  		5.0f
#define PID_KI_L  		3.5f
#define PID_KD_L  		0.06f

// PID parameters for the right motor:
#define PID_KP_R  		4.3f
#define PID_KI_R  		1.5f
#define PID_KD_R  		0.05f

// ----------------------------------------------------
// PID STRUCTURE:
// ----------------------------------------------------
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PID_t;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Initiate PID:
void PID_Init (PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
// Compute PID:
float PID_Compute (PID_t *pid, float target, float actual, float dt);
// Reset PID:
void PID_Reset (PID_t *pid);

#endif /* PID_H_ */
