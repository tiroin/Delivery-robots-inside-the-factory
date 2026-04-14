/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
** ###################################################################*/

#include "Cpu.h"
#include "motor.h"
#include "initiate.h"
#include "can.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

volatile int exit_code = 0;

// Task handles:
TaskHandle_t Task_CAN_Rx;
TaskHandle_t Task_CAN_Tx;
TaskHandle_t Task_Motor;

// Parameters:
extern uint16_t target_L;
extern uint16_t target_R;
extern uint16_t current_L;
extern uint16_t current_R;

static void run_motion(int total_steps, int ramp_steps) {
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

// Send string to PuTTY via USB to TTL (UART):
void uart_print(const char* str) {
    LPUART_DRV_SendDataPolling(INST_LPUART1, (uint8_t*)str, strlen(str));
}

// Motor ramp task - calls update_motor_ramp() every 20ms:
void task_motor_handle(void *pvParameters) {
    (void)pvParameters;
    motor_init();
    vTaskDelay(pdMS_TO_TICKS(7000));
    for (;;) {
    	// Move forward:
        move_forward();
        run_motion(250, RAMP_DOWN_STEPS);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Turn left:
        turn_left();
        run_motion(150, RAMP_DOWN_STEPS_TURN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Move forward:
        move_forward();
        run_motion(250, RAMP_DOWN_STEPS);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Turn right:
        turn_right();
        run_motion(150, RAMP_DOWN_STEPS_TURN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Move backward:
        move_backward();
        run_motion(200, RAMP_DOWN_STEPS);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// Receiving task:
void task_can_rx(void *pvParameters) {
    (void)pvParameters;
    // Initiate CAN configuration and start receiving:
    can_init();
    can_start_receiving();
    // Loop:
    for (;;) {
        if (can_is_received()) {
        	// Receiving data:
            char buf[9] = {0};
            memcpy(buf, rxData.data, 8);
            // Monitoring (Log):
            char msg[32] = {0};
            snprintf(msg, sizeof(msg), "RX: %s\r\n", buf);
            uart_print(msg);
            // Start receiving data:
            can_start_receiving();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Sending task:
void task_can_tx(void *pvParameters) {
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(500));
    for (;;) {
    	// Send text to another node:
        can_send_text("NiceToMt");
        // Log:
        uart_print("TX: NiceToMt\r\n");
        // Delay:
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    // System initiation:
    sys_init();

    // Create tasks:
    xTaskCreate(task_can_rx,       "CAN_RX", 256, NULL, 2, &Task_CAN_Rx);
    xTaskCreate(task_can_tx,       "CAN_TX", 256, NULL, 2, &Task_CAN_Tx);
    xTaskCreate(task_motor_handle, "Motor",  256, NULL, 1, &Task_Motor);

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
