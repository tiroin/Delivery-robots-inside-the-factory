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

// Message ID (from other nodes to S32):
#define CAN_ID_EMERGENCY 0x001U
#define CAN_ID_CONTROL   0x111U

// Message ID (from S32 to other nodes):
#define CAN_ID_STATUS    0x222U

// Emergency flag:
extern volatile uint16_t emergency_flag;

// MB indices:
#define MB_RX_CONTROL     1U
#define MB_RX_EMERGENCY   2U
#define MB_TX_STATUS      3U
#define MB_TX_TEXT        4U

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
// Send status:
void can_send_status(uint16_t speedL, uint16_t speedR, uint8_t emg);

#endif
