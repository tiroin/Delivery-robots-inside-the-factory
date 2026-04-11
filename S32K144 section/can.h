#ifndef CAN_H_
#define CAN_H_

#include "Cpu.h"
#include "flexcan_driver.h"

#define INST_CAN    INST_CANCOM1

void can_init(void);
void can_send_text(const char* str);
void can_start_receiving(void);

#endif
