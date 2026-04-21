/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
** ###################################################################*/

// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------

#include "Cpu.h"
#include "motor.h"
#include "initiate.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

// ----------------------------------------------------
// DEFAULT AND COMPULSORY PARAMETERS:
// ----------------------------------------------------

volatile int exit_code = 0;

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

TaskHandle_t Task_CAN_Rx, Task_CAN_Tx, Task_Motor;
QueueHandle_t xMotorCmdQueue;
extern uint16_t target_L, target_R, current_L, current_R;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Print:
static void uart_log(const char* str) {
    LPUART_DRV_SendDataPolling(INST_LPUART1, (uint8_t*)str, strlen(str));
}

// ----------------------------------------------------
// TASK FUNCTIONS:
// ----------------------------------------------------

// Receiver:
void task_can_rx(void *pvParameters) {
    (void)pvParameters;
    can_init();
    uart_log("S32K DUAL-RX MODE: ACTIVE\r\n");
    const char* cmd_names[] = {"INVALID", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"};
    // For loop:
    for (;;) {
    	// Inspect emergency message:
        if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_EMERGENCY) == STATUS_SUCCESS) {
            uint8_t emg_val = rxData_emergency.data[0];
            // If received 0xFF (activation command):
            if (emg_val == 0xFF) {
            	// Trigger this emergency flag and stop robot:
                emergency_flag = 1;
                uint8_t stop_cmd = 5;
                xQueueOverwrite(xMotorCmdQueue, &stop_cmd);
                uart_log("!!! [RX-EMG] KICH HOAT EMERGENCY !!!\r\n");
            } else {
            	// Release:
                emergency_flag = 0;
                uart_log("--- [RX-EMG] GIAI PHONG EMERGENCY ---\r\n");
            }
            // Continue receiving:
            FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);
        }
        // Inspect control message:
        if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_CONTROL) == STATUS_SUCCESS) {
            uint8_t cmd = rxData_control.data[0];
            char log_buf[64];
            // Inspect commands:
            if (cmd >= 1 && cmd <= 5) {
                sprintf(log_buf, "[RX-CTRL] Command: %s", cmd_names[cmd]);
            } else {
            	sprintf(log_buf, "[RX-CTRL] Command: UNKNOWN (%d)", cmd);
            }
            uart_log(log_buf);
            // Inspect if there is any emergency flag:
            if (emergency_flag == 0) {
            	xQueueOverwrite(xMotorCmdQueue, &cmd);
                uart_log(" -> [ACCEPTED]\r\n");
            } else {
                uart_log(" -> [REJECTED] - System Locked!\r\n");
            }
            FLEXCAN_DRV_Receive(INST_CAN, MB_RX_CONTROL, &rxData_control);
        }
        // Delay:
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Control section:
void task_motor_handle(void *pvParameters) {
    (void)pvParameters;
    // Parameters:
    motor_init();
    uint8_t cmd = 0;
    for (;;) {
    	// Inspect commands:
    	xQueueReceive(xMotorCmdQueue, &cmd, 0);
        switch(cmd) {
        	case 1: move_forward();  break;
            case 2: move_backward(); break;
            case 3: turn_left();     break;
            case 4: turn_right();    break;
            case 5: stop_robot();    break;
        }
        // Update speed:
        update_motor_ramp();
        // Delay:
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Transmitter:
void task_can_tx(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
    	// Information on the motors' speed:
        can_send_status(current_L, current_R, (uint8_t)emergency_flag);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ----------------------------------------------------
// MAIN:
// ----------------------------------------------------

int main(void) {
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();
  #endif
  /*** End of Processor Expert internal initialization. ***/

    // Startup:
    sys_init();
    S32K144EVB_startup();
    xMotorCmdQueue = xQueueCreate(1, sizeof(uint8_t));

    // Create tasks:
    xTaskCreate(task_motor_handle, "Motor", 256, NULL, 4, &Task_Motor);
    xTaskCreate(task_can_rx,       "RX",    256, NULL, 3, &Task_CAN_Rx);
    xTaskCreate(task_can_tx,       "TX",    256, NULL, 2, &Task_CAN_Tx);

    // Start scheduling:
    vTaskStartScheduler();

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
