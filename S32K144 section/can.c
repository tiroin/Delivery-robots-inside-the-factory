// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------

#include "can.h"
#include <string.h>

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// States:
flexcan_state_t  	canState;

// Message buffer:
flexcan_msgbuff_t 	rxData_control;
flexcan_msgbuff_t 	rxData_emergency;

// Emergency flag:
volatile uint16_t emergency_flag = 0;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Initiate CAN configurations:
void can_init(void) {
    FLEXCAN_DRV_Init(INST_CAN, &canState, &canCom1_InitConfig0);
    FLEXCAN_DRV_SetRxMaskType(INST_CAN, FLEXCAN_RX_MASK_INDIVIDUAL);

    flexcan_data_info_t rxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 1U,
        .is_remote   = false
    };

    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_CONTROL, &rxInfo, CAN_ID_CONTROL);
    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_CONTROL, 0x7FFU);

    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_EMERGENCY, &rxInfo, CAN_ID_EMERGENCY);
    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_EMERGENCY, 0x7FFU);

    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_CONTROL, &rxData_control);
    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);
}

// Send data:
void can_send_text(const char* str) {
	// Configure transmitting data format:
    flexcan_data_info_t dataInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };
    // Send this data to another node:
    uint8_t payload[8] = {0};
    strncpy((char*)payload, str, 8);
    FLEXCAN_DRV_Send(INST_CAN, MB_TX_TEXT, &dataInfo, CAN_ID_CONTROL, payload);
}

// Receive data:
void can_start_receiving(void) {
    // Configure receiving data format:
	flexcan_data_info_t rxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };

	// MB1: ID 0x111 (control message), Mask: 0x7FF = exact match (11-bit standard ID):
	FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_CONTROL, 0x7FFU);
	FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_CONTROL, &rxInfo, CAN_ID_CONTROL);
	FLEXCAN_DRV_Receive(INST_CAN, MB_RX_CONTROL, &rxData_control);

	// MB2: ID 0x001 (emergency message)
	FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_EMERGENCY, 0x7FFU);
	FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_EMERGENCY, &rxInfo, CAN_ID_EMERGENCY);
	FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);
}

// Received message logic (MB1):
uint8_t can_control_received(void) {
	if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_CONTROL) != STATUS_BUSY) {
		return 1U;
	}
	return 0U;
}

// Received message logic (MB2):
uint8_t can_emergency_received(void) {
	if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_EMERGENCY) != STATUS_BUSY) {
		return 1U;
	}
	return 0U;
}

// Send status (later, let current = left-motor speed, battery = right-motor speed):
void can_send_status(uint16_t speedL, uint16_t speedR, uint8_t emg) {
    flexcan_data_info_t txInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 5U, // 2 byte L + 2 byte R + 1 byte EMG
        .is_remote   = false
    };
    uint8_t data[5];
    // Left speed:
    data[0] = (uint8_t)(speedL >> 8);
    data[1] = (uint8_t)(speedL & 0xFF);
    // Right speed:
    data[2] = (uint8_t)(speedR >> 8);
    data[3] = (uint8_t)(speedR & 0xFF);
    // Emergency status:
    data[4] = emg;
    // Send message with ID 0x222:
    FLEXCAN_DRV_Send(INST_CAN, MB_TX_STATUS, &txInfo, 0x222U, data);
}
