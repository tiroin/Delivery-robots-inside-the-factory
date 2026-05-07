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
#include <stdlib.h>

// ----------------------------------------------------
// DEFAULT PARAMETER:
// ----------------------------------------------------
volatile int exit_code = 0;

// ----------------------------------------------------
// EXTERNAL PARAMETERS:
// ----------------------------------------------------
extern uint16_t target_L, target_R, current_L, current_R;

// ----------------------------------------------------
// TASK AND QUEUE PARAMETERS:
// ----------------------------------------------------
TaskHandle_t Task_CAN_Rx, Task_CAN_Tx, Task_Motor;
QueueHandle_t xMotorCmdQueue;

// ----------------------------------------------------
// MOTOR COMMAND STRUCT:
// ----------------------------------------------------
typedef struct {
    uint8_t  cmd;      // 1..5
    uint16_t speed_L;  // 0 or [MIN_RUNNING_SPEED, MAX_SPEED_L]
    uint16_t speed_R;  // 0 or [MIN_RUNNING_SPEED, MAX_SPEED_R]
} MotorCmd_t;

// ----------------------------------------------------
// PARSER:
// Converts raw CAN bytes to MotorCmd_t.
// Returns 1 on success, 0 if the frame is malformed.
// ----------------------------------------------------
static uint8_t parse_can_frame(const uint8_t *data, uint8_t dlc, MotorCmd_t *out) {
    if (dlc < 5) return 0U;
    // Parse:
    out->cmd     = data[0];
    out->speed_L = (uint16_t)((data[1] << 8) | data[2]);
    out->speed_R = (uint16_t)((data[3] << 8) | data[4]);
    // Clamp:
    if (out->speed_L > (uint16_t)MAX_SPEED_L) out->speed_L = MAX_SPEED_L;
    if (out->speed_R > (uint16_t)MAX_SPEED_R) out->speed_R = MAX_SPEED_R;
    return 1U;
}

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------
static void float_to_str(char* buf, float val) {
    if (val < 0.0f) { *buf++ = '-'; val = -val; }
    uint32_t integer = (uint32_t)val;
    uint32_t decimal = (uint32_t)((val - (float)integer) * 10.0f + 0.5f);
    if (decimal >= 10U) { integer++; decimal = 0U; }
    char tmp[12]; int i = 0;
    if (integer == 0U) { tmp[i++] = '0'; }
    else {
        uint32_t n = integer; int start = i;
        while (n > 0U) { tmp[i++] = (char)('0' + (n % 10U)); n /= 10U; }
        int end = i - 1;
        while (start < end) { char c = tmp[start]; tmp[start]=tmp[end]; tmp[end]=c; start++; end--; }
    }
    tmp[i++] = '.'; tmp[i++] = (char)('0' + decimal); tmp[i] = '\0';
    int j = 0; while (tmp[j]) { *buf++ = tmp[j++]; } *buf = '\0';
}

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
    // For loop:
    for (;;) {
        // --- Emergency message ---
        if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_EMERGENCY) == STATUS_SUCCESS) {
            uint8_t emg_val = rxData_emergency.data[0];
            if (emg_val == 0xFF) {
                emergency_flag = 1;
                MotorCmd_t stop_cmd = { .cmd = 5U, .speed_L = 0U, .speed_R = 0U };
                xQueueOverwrite(xMotorCmdQueue, &stop_cmd);
            } else {
                emergency_flag = 0;
            }
            FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);
        }
        // --- Control message ---
        if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_CONTROL) == STATUS_SUCCESS) {
            MotorCmd_t motor_cmd;
            // rxData_control.dataLen holds the number of valid bytes in the frame:
            if (parse_can_frame(rxData_control.data, rxData_control.dataLen, &motor_cmd)) {
                if (emergency_flag == 0) {
                    xQueueOverwrite(xMotorCmdQueue, &motor_cmd);
                }
            }
            FLEXCAN_DRV_Receive(INST_CAN, MB_RX_CONTROL, &rxData_control);
        }
        // Task delay:
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Motor control task:
void task_motor_handle(void *pvParameters) {
    (void)pvParameters;
    // Initiate motor task:
    motor_init();

    // Tick parameters:
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    // Parameters:
    char log_buf[128];
    char str_L[16], str_R[16];
    uint8_t log_divider = 0;

    // Commands:
    MotorCmd_t cmd      = { .cmd = 0U, .speed_L = 0U, .speed_R = 0U };
    MotorCmd_t last_cmd = { .cmd = 0U, .speed_L = 0U, .speed_R = 0U };

    // For loop:
    for (;;) {
        if (xQueueReceive(xMotorCmdQueue, &cmd, 0) == pdPASS) {
            if (cmd.cmd     != last_cmd.cmd     ||
                cmd.speed_L != last_cmd.speed_L ||
                cmd.speed_R != last_cmd.speed_R) {
                // Call movement function to set direction + reset PID,
                // then immediately override target_L / target_R with the
                // individual wheel speeds received over CAN:
                switch (cmd.cmd) {
                    case 1:
                        move_forward(cmd.speed_L);  // sets direction fwd + PID reset
                        target_L = cmd.speed_L;
                        target_R = cmd.speed_R;
                        break;
                    case 2:
                        move_backward(cmd.speed_L); // sets direction rev + PID reset
                        target_L = cmd.speed_L;
                        target_R = cmd.speed_R;
                        break;
                    case 3:
                        turn_left(cmd.speed_R);     // sets direction + PID reset
                        target_L = cmd.speed_L;
                        target_R = cmd.speed_R;
                        break;
                    case 4:
                        turn_right(cmd.speed_L);    // sets direction + PID reset
                        target_L = cmd.speed_L;
                        target_R = cmd.speed_R;
                        break;
                    case 5:
                        stop_robot();               // target_L = target_R = 0 inside
                        break;
                    default:
                        break;
                }
                last_cmd = cmd;
            }
        }
        // Update motor:
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
        // Task delay:
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Transmitter:
void task_can_tx(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        can_send_status(current_L, current_R, (uint8_t)emergency_flag);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ----------------------------------------------------
// MAIN:
// ----------------------------------------------------
int main(void) {
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();
  #endif

    // Initiate system:
    sys_init();
    S32K144EVB_startup();

    // Create queue:
    xMotorCmdQueue = xQueueCreate(1, sizeof(MotorCmd_t));

    // Initiate tasks:
    xTaskCreate(task_motor_handle, "Motor", 256, NULL, 4, &Task_Motor);
    xTaskCreate(task_can_rx,       "RX",    256, NULL, 3, &Task_CAN_Rx);
    xTaskCreate(task_can_tx,       "TX",    256, NULL, 2, &Task_CAN_Tx);

    // Start scheduling:
    vTaskStartScheduler();

  #ifdef PEX_RTOS_START
    PEX_RTOS_START();
  #endif
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
}
