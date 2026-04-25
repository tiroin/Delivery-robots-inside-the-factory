#ifndef PID_H_
#define PID_H_

// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------
#include <stdint.h>

// ----------------------------------------------------
// PID PARAMETERS:
// ----------------------------------------------------
#define PID_DT          0.02f   // 20ms loop

// Left motor:
#define PID_KP_L  		2.0f
#define PID_KI_L  		1.5f
#define PID_KD_L  		0.08f

// Right motor:
#define PID_KP_R  		1.8f
#define PID_KI_R  		1.2f
#define PID_KD_R  		0.08f

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

void PID_Init (PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Compute (PID_t *pid, float target, float actual, float dt);
void PID_Reset (PID_t *pid);

#endif /* PID_H_ */
