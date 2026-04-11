/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
** ###################################################################*/

#include "Cpu.h"
#include "motor.h"
#include "initiate.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

volatile int exit_code = 0;

// Task handles:
TaskHandle_t Task_Motor;

extern uint16_t target_L;
extern uint16_t target_R;
extern uint16_t current_L;
extern uint16_t current_R;

static void run_motion(int total_steps, int ramp_steps)
{
    int stop_at = total_steps - ramp_steps;

    for (int i = 0; i < total_steps; i++) {
        if (i == stop_at)
            stop_robot();
        update_motor_ramp();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    uint32_t timeout = 100;
    while (!motors_stopped() && timeout--) {
        update_motor_ramp();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Motor ramp task - calls update_motor_ramp() every 20ms:
void task_motor_handle(void *pvParameters) {
    (void)pvParameters;
    motor_init();
    vTaskDelay(pdMS_TO_TICKS(7000));

    for (;;) {

        move_forward();
        run_motion(250, RAMP_DOWN_STEPS);
        vTaskDelay(pdMS_TO_TICKS(1000));

        turn_left();
        run_motion(150, RAMP_DOWN_STEPS_TURN);
        vTaskDelay(pdMS_TO_TICKS(1000));

        move_forward();
        run_motion(250, RAMP_DOWN_STEPS);
        vTaskDelay(pdMS_TO_TICKS(1000));

        turn_right();
        run_motion(150, RAMP_DOWN_STEPS_TURN);
        vTaskDelay(pdMS_TO_TICKS(1000));

        move_backward();
        run_motion(200, RAMP_DOWN_STEPS);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

int main(void)
{
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    // System init:
    sys_init();

    // Create motor task:
    xTaskCreate(task_motor_handle, "Task Motor", 256, NULL, 1, &Task_Motor);

    // Start scheduler:
    vTaskStartScheduler();

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
}
