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

// Getting the parameters from the C file:
extern flexcan_msgbuff_t rxData_control;				// 0x111
extern flexcan_msgbuff_t rxData_emergency;				// 0x001 (Highest priority)
// Message ID:
#define CAN_ID_EMERGENCY 0x001U
#define CAN_ID_CONTROL   0x111U
// Emergency flag:
extern uint16_t emergency_flag;
// MB indices:
#define MB_TX             0U
#define MB_RX_CONTROL     1U
#define MB_RX_EMERGENCY   2U
// Instantiate CAN:
#define INST_CAN INST_CANCOM1

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Initiate CAN configurations:
void can_init(void);
// Send text (or any kind of data):
void can_send_text(const char* str);
// Receive data:
void can_start_receiving(void);
// Received message logic:
uint8_t can_control_received(void);
uint8_t can_emergency_received(void);

#endif
