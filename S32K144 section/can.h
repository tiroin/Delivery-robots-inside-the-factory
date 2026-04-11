#ifndef CAN_H_
#define CAN_H_

// Include necessary libraries:
#include "Cpu.h"
#include "flexcan_driver.h"

// Extern to main.c:
extern flexcan_msgbuff_t rxData;

// Instantiate CAN:
#define INST_CAN    INST_CANCOM1

// Initiate CAN configurations:
void can_init(void);

// Send text (or any kind of data):
void can_send_text(const char* str);

// Receive data:
void can_start_receiving(void);

// Received or not:
uint8_t can_is_received(void);

#endif
