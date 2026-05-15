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
#define PID_KP_L  		0.0080f
#define PID_KI_L  		0.0035f
#define PID_KD_L  		0.00005f

// PID parameters for the right motor:
#define PID_KP_R  		0.006f
#define PID_KI_R  		0.0035f
#define PID_KD_R  		0.00005f

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
