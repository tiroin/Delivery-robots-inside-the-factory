#ifndef CAN_H_
#define CAN_H_

// Include necessary libraries:
#include "Cpu.h"
#include "flexcan_driver.h"

// Getting the parameters from the C file:
extern flexcan_msgbuff_t rxData;

// Message ID:
#define CAN_ID_EMERGENCY 0x001U
#define CAN_ID_CONTROL   0x111U

// Emergency flag:
extern uint16_t emergency_flag;

// Instantiate CAN:
#define INST_CAN INST_CANCOM1

// Initiate CAN configurations:
void can_init(void);

// Send text (or any kind of data):
void can_send_text(const char* str);

// Receive data:
void can_start_receiving(void);

// Received or not:
uint8_t can_is_received(void);

// Follow the flowchart:
uint8_t can_process_logic(void);

#endif
