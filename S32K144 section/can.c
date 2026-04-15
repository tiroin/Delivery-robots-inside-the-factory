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
uint16_t emergency_flag = 0;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Initiate CAN configurations:
void can_init(void) {
    FLEXCAN_DRV_Init(INST_CAN, &canState, &canCom1_InitConfig0);
    FLEXCAN_DRV_SetRxMaskType(INST_CAN, FLEXCAN_RX_MASK_INDIVIDUAL);
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
    FLEXCAN_DRV_Send(INST_CAN, 0U, &dataInfo, CAN_ID_CONTROL, payload);
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
    return (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_CONTROL) != STATUS_BUSY) ? 1U : 0U;
}

// Received message logic (MB2):
uint8_t can_emergency_received(void) {
    return (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_EMERGENCY) != STATUS_BUSY) ? 1U : 0U;
}
