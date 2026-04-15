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

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// Task handles:
TaskHandle_t Task_CAN_Rx;
TaskHandle_t Task_CAN_Tx;
TaskHandle_t Task_Motor;

// Parameters:
extern uint16_t target_L;
extern uint16_t target_R;
extern uint16_t current_L;
extern uint16_t current_R;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

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
static void uart_print(const char* str) {
    LPUART_DRV_SendDataPolling(INST_LPUART1, (uint8_t*)str, strlen(str));
}

static void can_restart_rx_mb(uint8_t mb_idx, uint32_t msg_id, flexcan_msgbuff_t *rx_buf) {
    flexcan_data_info_t rxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };
    FLEXCAN_DRV_ConfigRxMb(INST_CAN, mb_idx, &rxInfo, msg_id);
    FLEXCAN_DRV_Receive(INST_CAN, mb_idx, rx_buf);
}

// ----------------------------------------------------
// TASK FUNCTIONS:
// ----------------------------------------------------

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
    // Initiate CAN and start receiving message:
    can_init();
    can_start_receiving();
    // For loop:
    for (;;) {
    	// Inspect emergency message:
        if (can_emergency_received()) {
        	uint8_t cmd = rxData_emergency.data[0];
        	char em_msg[64];
        	snprintf(em_msg, sizeof(em_msg), "S32K RX (EMRG): ID=0x001, Data[0]=0x%02X\r\n", cmd);
        	// Inspect the message content (0xFF means stop)
        	if (rxData_emergency.data[0] == 0xFF) {
        		uart_print(em_msg);
        		// Set emergency flag:
        		emergency_flag = 1;
        		// Stop robot:
        		stop_robot();
        		uart_print("[!!!] EMERGENCY STOP!\r\n");
        	} else if (rxData_emergency.data[0] == 0x00) {
        		// Reset emergency flag:
        		emergency_flag = 0;
        		uart_print("[OK] Clear Emergency. Robot ready.\r\n");
        	}
            // Restart and wait for other messages:
            can_restart_rx_mb(MB_RX_EMERGENCY, CAN_ID_EMERGENCY, &rxData_emergency);
        }
        // Inspect control message:
        if (can_control_received()) {
            // Only execute if there aren't any emergency messages:
            if (!emergency_flag) {
                char buf[9] = {0};
                memcpy(buf, rxData_control.data, 8);
                char msg[32] = {0};
                snprintf(msg, sizeof(msg), "RX: %s\r\n", buf);
                uart_print(msg);
            }
            // Restart and wait for other messages:
            can_restart_rx_mb(MB_RX_CONTROL, CAN_ID_CONTROL, &rxData_control);
        }
        // Delay:
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Sending task:
void task_can_tx(void *pvParameters) {
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(500));
    // Send:
    for (;;) {
        if (!emergency_flag) {
            can_send_text("MOVING");
        } else {
        	can_send_text("STOPPED");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    // System initiation and quick startup:
    sys_init();
    S32K144EVB_startup();

    // Create tasks:
    xTaskCreate(task_can_rx,       "CAN_RX", 256, NULL, 3, &Task_CAN_Rx);
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
