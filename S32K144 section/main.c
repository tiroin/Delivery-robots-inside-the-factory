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
extern float duty_L_dbg, duty_R_dbg;

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

TaskHandle_t Task_CAN_Rx, Task_CAN_Tx, Task_Motor;
QueueHandle_t xMotorCmdQueue;
extern uint16_t target_L, target_R, current_L, current_R;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

static void float_to_str(char* buf, float val) {
    if (val < 0.0f) {
        *buf++ = '-';
        val = -val;
    }
    uint32_t integer = (uint32_t)val;
    uint32_t decimal = (uint32_t)((val - (float)integer) * 10.0f + 0.5f);
    if (decimal >= 10U) { integer++; decimal = 0U; }
    char tmp[12];
    int i = 0;
    if (integer == 0U) {
        tmp[i++] = '0';
    } else {
        uint32_t n = integer;
        int start = i;
        while (n > 0U) { tmp[i++] = (char)('0' + (n % 10U)); n /= 10U; }
        int end = i - 1;
        while (start < end) {
            char c = tmp[start]; tmp[start] = tmp[end]; tmp[end] = c;
            start++; end--;
        }
    }
    tmp[i++] = '.';
    tmp[i++] = (char)('0' + decimal);
    tmp[i]   = '\0';
    int j = 0;
    while (tmp[j]) { *buf++ = tmp[j++]; }
    *buf = '\0';
}

// Print:
static void uart_log(const char* str) {
    LPUART_DRV_SendDataPolling(INST_LPUART1, (uint8_t*)str, strlen(str));
}

// ----------------------------------------------------
// ISR FUNCTION:
// ----------------------------------------------------
void PORTD_IRQHandler(void) {
    Hall_Sensor_Handler();
}

// ----------------------------------------------------
// TASK FUNCTIONS:
// ----------------------------------------------------

// Receiver:
void task_can_rx(void *pvParameters) {
    (void)pvParameters;
    // Initiate CAN:
    can_init();

    // Parameter:
//    const char* cmd_names[] = {"INVALID", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"};

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
            } else {
            	// Release:
                emergency_flag = 0;
            }
            // Continue receiving:
            FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);
        }
        // Inspect control message:
        if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_CONTROL) == STATUS_SUCCESS) {
            uint8_t cmd = rxData_control.data[0];
//            char log_buf[64];
            // Inspect commands:
            if (cmd >= 1 && cmd <= 5) {
//                sprintf(log_buf, "[RX-CTRL] Command: %s", cmd_names[cmd]);
            } else {
//            	sprintf(log_buf, "[RX-CTRL] Command: UNKNOWN (%d)", cmd);
            }
//            uart_log(log_buf);
            // Inspect if there is any emergency flag:
            if (emergency_flag == 0) {
            	xQueueOverwrite(xMotorCmdQueue, &cmd);
//                uart_log(" -> [ACCEPTED]\r\n");
            } else {
//                uart_log(" -> [REJECTED] - System Locked!\r\n");
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
    // Initiate motor parameters as well as PID:
    motor_init();

    // Parameters:
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    char log_buf[128];
    char str_L[16], str_R[16];
    uint8_t log_divider = 0;
    uint8_t cmd = 0;
    uint8_t last_cmd = 0;

    // For loop:
    for (;;) {
    	// Inspect commands:
    	if (xQueueReceive(xMotorCmdQueue, &cmd, 0) == pdPASS) {
    		if (cmd != last_cmd) {
				switch(cmd) {
					case 1: move_forward();  break;
					case 2: move_backward(); break;
					case 3: turn_left();     break;
					case 4: turn_right();    break;
					case 5: stop_robot();    break;
				}
				last_cmd = cmd;
    		}
    	}

        // Update speed:
        update_motor_ramp();

        // UART log:
        if (++log_divider >= 5) {
            float actual_scaled_L = (actual_L_val / 6.0f) * MAX_SPEED_L;
            float actual_scaled_R = (actual_R_val / 6.0f) * MAX_SPEED_R;

            float_to_str(str_L, actual_scaled_L);
            float_to_str(str_R, actual_scaled_R);

            sprintf(log_buf, "%u,%s,%u,%s\n", current_L, str_L, current_R, str_R);

            uart_log(log_buf);
            log_divider = 0;
        }

        // Delay:
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
