/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
// Including necessary libraries:
#include "Cpu.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include "lpuart0.h"
#include "adc_driver.h"

// Volatile variables:
volatile int exit_code = 0;

// =========================================================================
// USER VARIABLES
// =========================================================================
uint8_t LED_logic = 0;
uint8_t button_logic = 0;
ftm_state_t flexTimer_pwm1_State;
uint16_t cache_duty_cycle;
uint16_t adc_value = 0;
// =========================================================================

// =========================================================================
// GLOBAL VARIABLES FOR FREERTOS:
// =========================================================================
// Defining user tasks:
TaskHandle_t Task_UART;
TaskHandle_t Task_ADC;
TaskHandle_t Task_PWM;
// =========================================================================

// =========================================================================
// GLOBAL VARIABLES FOR PID AND MOTORS:
// =========================================================================
// Phases for both motors:
#define PHASE_A_LOW_LEFT  10U
#define PHASE_B_LOW_LEFT  11U
#define PHASE_C_LOW_LEFT  12U
#define PHASE_A_LOW_RIGHT  10U
#define PHASE_B_LOW_RIGHT  11U
#define PHASE_C_LOW_RIGHT  12U

// Channels for both motors:
#define channel_A_Left   	0U
#define channel_B_Left    	1U
#define channel_C_Left   	2U
#define channel_A_Right    	0U
#define channel_B_Right    	1U
#define channel_C_Right	    2U

// States:
typedef enum {STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT} robot_state_t;
// Steps:
uint8_t current_step_Left = 1;
uint8_t current_step_Right = 1;
// Actual and computed speed using PID:
uint16_t speed_actual_Left = 0;
uint16_t motor_speed_Left = 0;
uint16_t speed_actual_Right = 0;
uint16_t motor_speed_Right = 0;
robot_state_t robot_dir = STOP;

// Delta t:
float dt = 0.01;

// Parameters for PID:
typedef struct {
	float Kp, Ki, Kd;		// PID metrics
	float setpoint;         // Target
	float error_sum;        // Error sum (I)
	float last_error;       // Last error (D)
	uint16_t out_max;		// Max duty cycle
} PID_controller;

// Left and right motor (We should adjust Kp, Ki, Kd as well as duty cycle):
PID_controller motor_Left = {1.5f, 0.1f, 0.05f, 0, 0, 0, 10000}; // 10000 will be adjusted
PID_controller motor_Right = {1.5f, 0.1f, 0.05f, 0, 0, 0, 10000}; // 10000 will be adjusted
// =========================================================================

// =========================================================================
// FORMAT CODE FOR ADC AND PWM:
// =========================================================================
// PWM (useful for controlling motors):
void write_pwm(uint8_t channel, uint16_t duty_cycle) {
	// Set duty cycle:
	FTM_DRV_UpdatePwmChannel(
		INST_FLEXTIMER_PWM1,
		channel,
		FTM_PWM_UPDATE_IN_DUTY_CYCLE,
		duty_cycle,
		duty_cycle,
		true
	);
}
// ADC Format (read ADC value):
uint16_t read_ADC() {
	uint16_t adc_raw_value;
	ADC_DRV_ConfigChan(INST_ADCONV1, 0U, &adConv1_ChnConfig0);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
	ADC_DRV_GetChanResult(INST_ADCONV1, 0U, &adc_raw_value);
	return adc_raw_value;
}
// =========================================================================

// =========================================================================
// FORMAT CODE FOR CAN TX AND RX:
// =========================================================================

// =========================================================================

// =========================================================================
// FORMAT CODE FOR CONTROLLING MOTORS:
// =========================================================================
// Set duty cycle for 3-phase motor:
void write_pwm_3_phase_Left(uint16_t dutyA_left, uint16_t dutyB_left, uint16_t dutyC_left) {
	write_pwm(channel_A_Left, dutyA_left);
	write_pwm(channel_B_Left, dutyB_left);
	write_pwm(channel_C_Left, dutyC_left);
}
void write_pwm_3_phase_Right(uint16_t dutyA_right, uint16_t dutyB_right, uint16_t dutyC_right) {
	write_pwm(channel_A_Right, dutyA_right);
	write_pwm(channel_B_Right, dutyB_right);
	write_pwm(channel_C_Right, dutyC_right);
}
// Set commutation steps for each motor:
// Left motor:
void set_commutation_step_Left(uint8_t step, uint16_t speed) {
	// Reset:
	write_pwm_3_phase_Left(0, 0, 0);
    PINS_DRV_ClearPins(PTx, (1 << PHASE_A_LOW_LEFT) | (1 << PHASE_B_LOW_LEFT) | (1 << PHASE_C_LOW_LEFT));
    // Control each phase according to steps:
    switch(step) {
        case 1: write_pwm_3_phase_Left(speed, 0, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_B_LOW_LEFT); break;
        case 2: write_pwm_3_phase_Left(speed, 0, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_C_LOW_LEFT); break;
        case 3: write_pwm_3_phase_Left(0, speed, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_C_LOW_LEFT); break;
        case 4: write_pwm_3_phase_Left(0, speed, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_A_LOW_LEFT); break;
        case 5: write_pwm_3_phase_Left(0, 0, speed); PINS_DRV_SetPins(PTx, 1 << PHASE_A_LOW_LEFT); break;
        case 6: write_pwm_3_phase_Left(0, 0, speed); PINS_DRV_SetPins(PTx, 1 << PHASE_B_LOW_LEFT); break;
        default: write_pwm_3_phase_Left(0, 0, 0); break;
    }
}
// Right motor:
void set_commutation_step_Right(uint8_t step, uint16_t speed) {
	// Reset:
	write_pwm_3_phase_Right(0, 0, 0);
    PINS_DRV_ClearPins(PTx, (1 << PHASE_A_LOW_RIGHT) | (1 << PHASE_B_LOW_RIGHT) | (1 << PHASE_C_LOW_RIGHT));
    // Control each phase according to steps:
    switch(step) {
        case 1: write_pwm_3_phase_Right(speed, 0, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_B_LOW_RIGHT); break;
        case 2: write_pwm_3_phase_Right(speed, 0, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_C_LOW_RIGHT); break;
        case 3: write_pwm_3_phase_Right(0, speed, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_C_LOW_RIGHT); break;
        case 4: write_pwm_3_phase_Right(0, speed, 0); PINS_DRV_SetPins(PTx, 1 << PHASE_A_LOW_RIGHT); break;
        case 5: write_pwm_3_phase_Right(0, 0, speed); PINS_DRV_SetPins(PTx, 1 << PHASE_A_LOW_RIGHT); break;
        case 6: write_pwm_3_phase_Right(0, 0, speed); PINS_DRV_SetPins(PTx, 1 << PHASE_B_LOW_RIGHT); break;
        default: write_pwm_3_phase_Right(0, 0, 0); break;
    }
}
// =========================================================================

// =========================================================================
// PID SECTION (COMPUTE PID):
// =========================================================================
uint16_t compute_PID(PID_controller *pid, float actual_speed, float dt) {
	// Compute error:
    float error = pid->setpoint - actual_speed;

    // PROPORTIONAL (P):
    float Pout = pid->Kp * error;

    // INTEGRATION (I):
    pid->error_sum += (error*dt);
    // Padding:
    if (pid->error_sum * pid->Ki > pid->out_max) pid->error_sum = pid->out_max / pid->Ki;
    if (pid->error_sum < 0) pid->error_sum = 0;
    float Iout = pid->Ki * pid->error_sum;

    // DERIVATIVE (D):
    float Dout = 0;
    if (dt > 0.0f) {
    	Dout = pid->Kd * ((error - pid->last_error) / dt);
    }

    // Cached value:
    pid->last_error = error;

    // Output sum for adjustment:
    float output = Pout + Iout + Dout;
    if (output > (float)pid->out_max) output = (float)pid->out_max;
    if (output < 0.0f) output = 0.0f;

    // Return the output value:
    return (uint16_t)output;
}
// =========================================================================

// =========================================================================
// UPDATE MOTOR CONTROL (WITH PID):
// =========================================================================
void update_motor_control() {
	if (robot_dir == STOP) {
		set_commutation_step_Left(0, 0);
		set_commutation_step_Right(0, 0);
		return;
	}

	// Compute motor speed while using PID:
	motor_speed_Left = compute_PID(&motor_Left, (float)speed_actual_Left, dt);
	motor_speed_Right = compute_PID(&motor_Right, (float)speed_actual_Right, dt);

	// LEFT MOTOR SECTION:
	// Moving forward or turning right:
	if (robot_dir == FORWARD || robot_dir == TURN_RIGHT) {
		current_step_Left++;
		if (current_step_Left > 6) current_step_Left = 1;
	}
	// moving backward or turning left:
	else if (robot_dir == BACKWARD || robot_dir == TURN_LEFT) {
		current_step_Left--;
		if (current_step_Left < 1) current_step_Left = 6;
	}

	// RIGHT MOTOR SECTION:
	// Moving forward or turning left:
	if (robot_dir == FORWARD || robot_dir == TURN_LEFT) {
		current_step_Right++;
		if (current_step_Right > 6) current_step_Right = 1;
	}
	// moving backward or turning right:
	else if (robot_dir == BACKWARD || robot_dir == TURN_RIGHT) {
		current_step_Right--;
		if (current_step_Right < 1) current_step_Right = 6;
	}

	// Set PWM using the motor speed value:
	set_commutation_step_Left(current_step_Left, motor_speed_Left);
	set_commutation_step_Right(current_step_Right, motor_speed_Right);
}
// =========================================================================

// =========================================================================
// LET ROBOT MOVE ACCORDING TO SUGGESTED DIRECTIONS:
// =========================================================================
// Forward:
void move_forward(uint16_t speed) {
    robot_dir = FORWARD;
    motor_Left.setpoint = (float)speed;
    motor_Right.setpoint = (float)speed;
}
// Backward:
void move_backward(uint16_t speed) {
    robot_dir = BACKWARD;
    motor_Left.setpoint = (float)speed;
    motor_Right.setpoint = (float)speed;
}
// Turn left:
void turn_left(uint16_t speed) {
	robot_dir = TURN_LEFT;
	motor_Left.setpoint = 0;
	motor_Right.setpoint = (float)speed;
}
// Turn right:
void turn_right(uint16_t speed) {
	robot_dir = TURN_RIGHT;
	motor_Left.setpoint = (float)speed;
	motor_Right.setpoint = 0;
}
// Pivot turn left:
void pivot_turn_left(uint16_t speed) {
	robot_dir = TURN_LEFT;
	motor_Left.setpoint = (float)speed;
	motor_Right.setpoint = (float)speed;
}
// Pivot turn right:
void pivot_turn_right(uint16_t speed) {
	robot_dir = TURN_RIGHT;
	motor_Left.setpoint = (float)speed;
	motor_Right.setpoint = (float)speed;
}
// Stop robot:
void stop_robot() {
	robot_dir = STOP;
	// Reset setpoint:
	motor_Left.setpoint = 0;
	motor_Right.setpoint = 0;
	// Reset PID:
	motor_Left.error_sum = 0;
	motor_Left.last_error = 0;
	motor_Right.last_error = 0;
	motor_Right.error_sum = 0;
	// Set all commutation step to 0:
	set_commutation_step_Left(0, 0);
	set_commutation_step_Right(0, 0);
}
// =========================================================================

// =========================================================================
// RTOS SECTION:
// =========================================================================
// UART task function:
//void Task_UART_handle(void *pvParameters) {
//    (void)pvParameters;
//    // Parameters and variables:
//    char msg[128];
//    // Loop:
//    for (;;) {
//        // Print ADC value as well as duty cycle:
//        snprintf(msg, sizeof(msg), "ADC Read Value: %u\r\n", (unsigned int)adc_value);
//        LPUART_DRV_SendDataPolling(INST_LPUART0, (uint8_t *)msg, strlen(msg));
//        snprintf(msg, sizeof(msg), "Duty cycle: %u\r\n", (unsigned int)cache_duty_cycle);
//        LPUART_DRV_SendDataPolling(INST_LPUART0, (uint8_t *)msg, strlen(msg));
//        // Delay:
//        vTaskDelay(pdMS_TO_TICKS(1000));
//    }
//}
//
//// PWM task function:
//void Task_PWM_handle(void *pvParameters) {
//	(void)pvParameters;
//	// Short delay:
//	vTaskDelay(pdMS_TO_TICKS(20));
//	// Loop:
//	for (;;) {
//		// Switching from the actual ADC to the final duty cycle and save the given value:
//		uint16_t duty_cycle = (uint16_t)(adc_value << 3);
//		write_pwm(INST_FLEXTIMER_PWM1, duty_cycle);
//		// Dimming an LED, a motor, etc..:
//		cache_duty_cycle = duty_cycle;
//		// Delay:
//		vTaskDelay(pdMS_TO_TICKS(10));
//	}
//}
//
//// ADC function:
//void Task_ADC_handle(void *pvParameters) {
//    (void)pvParameters;
//    // Short delay:
//    vTaskDelay(pdMS_TO_TICKS(10));
//    // Variables:
//    uint16_t adc_raw_value;
//    // Loop:
//    for (;;) {
//    	// Getting the ADC value:
//        adc_raw_value = read_ADC();
//        adc_value = adc_raw_value;
//        // Delay:
//        vTaskDelay(pdMS_TO_TICKS(10));
//    }
//}
// =========================================================================

/* User includes (#include below this line is not maintained by Processor Expert) */

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */

//    // Configuration (ADC, GPIO, PWM, UART pins and clock):
//    // Clock:
//    CLOCK_DRV_Init(&clockMan1_InitConfig0);
//    // Pins:
//    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
//    // PWM:
//    FTM_DRV_Init(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_InitConfig, &flexTimer_pwm1_State);
//    FTM_DRV_InitPwm(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_PwmConfig);
//    // ADC:
//    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);
//    // UART:
//    LPUART_DRV_Init(INST_LPUART0, &lpuart0_State, &lpuart0_InitConfig0);
//
//    // Initiate tasks:
//    xTaskCreate(Task_UART_handle, "Task_UART", 256, NULL, 1, &Task_UART);
//    xTaskCreate(Task_PWM_handle, "Task_PWM", 256, NULL, 2, &Task_PWM);
//    xTaskCreate(Task_ADC_handle, "Task_ADC", 256, NULL, 3, &Task_ADC);
//
//    // Start scheduling (based on each task's priority):
//    vTaskStartScheduler();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP S32K series of microcontrollers.
**
** ###################################################################
*/
