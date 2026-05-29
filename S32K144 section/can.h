#ifndef CAN_H_
#define CAN_H_

// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------

#include "Cpu.h"
#include "flexcan_driver.h"

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// Message buffers:
extern flexcan_msgbuff_t rxData_control;
extern flexcan_msgbuff_t rxData_emergency;
extern flexcan_msgbuff_t rxData_angle;

// Message IDs:
#define CAN_ID_EMERGENCY 0x001U
#define CAN_ID_CONTROL   0x111U
#define CAN_ID_STATUS    0x222U
#define CAN_ID_ANGLE     0x050U

// Data length codes:
#define CAN_DLC_EMERGENCY 1U
#define CAN_DLC_CONTROL   5U
#define CAN_DLC_STATUS    5U
#define CAN_DLC_TEXT      8U
#define CAN_DLC_ANGLE     6U

// Emergency flag:
extern volatile uint16_t emergency_flag;

// Message buffer indices:
#define MB_RX_CONTROL     1U
#define MB_RX_EMERGENCY   2U
#define MB_TX_STATUS      3U
#define MB_TX_TEXT        4U
#define MB_RX_ANGLE       5U

// CAN instance:
#define INST_CAN INST_CANCOM1

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

void can_init(void);
void can_send_text(const char* str);
void can_start_receiving(void);
uint8_t can_control_received(void);
uint8_t can_emergency_received(void);
uint8_t can_angle_received(void);
void can_send_status(uint16_t speedL, uint16_t speedR, uint8_t emg);

#endif
