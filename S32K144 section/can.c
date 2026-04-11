// Include necessary libraries:
#include "can.h"
#include <string.h>

// States:
flexcan_state_t  canState;
flexcan_msgbuff_t rxData;

// Initiate CAN configurations:
void can_init(void) {
    FLEXCAN_DRV_Init(INST_CAN, &canState, &canCom1_InitConfig0);
}

// Send data:0
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
    FLEXCAN_DRV_Send(INST_CAN, 0U, &dataInfo, 0x111U, payload);
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
    // Configure RX:
    FLEXCAN_DRV_ConfigRxMb(INST_CAN, 1U, &rxInfo, 0x123U);
    // Receive data:
    FLEXCAN_DRV_Receive(INST_CAN, 1U, &rxData);
}

// Received or not:
uint8_t can_is_received(void) {
    // Polling transfer status:
    if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, 1U) != STATUS_BUSY) {
        return 1U;
    }
    return 0U;
}
