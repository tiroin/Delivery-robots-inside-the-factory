#include "can.h"
#include <string.h>

flexcan_state_t  canState;
flexcan_msgbuff_t rxData;

void can_init(void) {
    FLEXCAN_DRV_Init(INST_CAN, &canState, &canCom1_InitConfig0);
}

void can_send_text(const char* str) {
    flexcan_data_info_t dataInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };

    uint8_t payload[8] = {0};
    strncpy((char*)payload, str, 8);
    FLEXCAN_DRV_Send(INST_CAN, 0U, &dataInfo, 0x111U, payload);
}

void can_start_receiving(void) {
    flexcan_data_info_t rxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };
    FLEXCAN_DRV_ConfigRxMb(INST_CAN, 1U, &rxInfo, 0x123U);
    FLEXCAN_DRV_Receive(INST_CAN, 1U, &rxData);
}
