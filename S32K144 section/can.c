// ----------------------------------------------------
// NECESSARY LIBRARIES:
// ----------------------------------------------------

#include "can.h"
#include <string.h>

// ----------------------------------------------------
// PARAMETERS:
// ----------------------------------------------------

// States:
flexcan_state_t canState;

// Message buffer:
flexcan_msgbuff_t rxData_control;
flexcan_msgbuff_t rxData_emergency;
flexcan_msgbuff_t rxData_angle;

// Emergency flag:
volatile uint16_t emergency_flag = 0;

// ----------------------------------------------------
// SUPPORTED FUNCTIONS:
// ----------------------------------------------------

// Initiate CAN configurations:
void can_init(void) {
    FLEXCAN_DRV_Init(INST_CAN, &canState, &canCom1_InitConfig0);
    FLEXCAN_DRV_SetRxMaskType(INST_CAN, FLEXCAN_RX_MASK_INDIVIDUAL);

    flexcan_data_info_t rxControlInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_CONTROL,
        .is_remote   = false
    };

    flexcan_data_info_t rxEmergencyInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_EMERGENCY,
        .is_remote   = false
    };

    flexcan_data_info_t rxAngleInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_ANGLE,
        .is_remote   = false
    };

    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_CONTROL, &rxControlInfo, CAN_ID_CONTROL);
    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_CONTROL, 0x7FFU);

    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_EMERGENCY, &rxEmergencyInfo, CAN_ID_EMERGENCY);
    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_EMERGENCY, 0x7FFU);

    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_ANGLE, &rxAngleInfo, CAN_ID_ANGLE);
    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_ANGLE, 0x7FFU);

    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_CONTROL, &rxData_control);
    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);
    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_ANGLE, &rxData_angle);
}

// Send text data:
void can_send_text(const char* str) {
    flexcan_data_info_t dataInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_TEXT,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };

    uint8_t payload[CAN_DLC_TEXT] = {0};
    strncpy((char*)payload, str, CAN_DLC_TEXT);
    FLEXCAN_DRV_Send(INST_CAN, MB_TX_TEXT, &dataInfo, CAN_ID_CONTROL, payload);
}

// Receive data:
void can_start_receiving(void) {
    flexcan_data_info_t rxControlInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_CONTROL,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };

    flexcan_data_info_t rxEmergencyInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_EMERGENCY,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };

    flexcan_data_info_t rxAngleInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_ANGLE,
        .fd_enable   = false,
        .enable_brs  = false,
        .fd_padding  = 0U,
        .is_remote   = false
    };

    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_CONTROL, 0x7FFU);
    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_CONTROL, &rxControlInfo, CAN_ID_CONTROL);
    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_CONTROL, &rxData_control);

    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_EMERGENCY, 0x7FFU);
    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_EMERGENCY, &rxEmergencyInfo, CAN_ID_EMERGENCY);
    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_EMERGENCY, &rxData_emergency);

    FLEXCAN_DRV_SetRxIndividualMask(INST_CAN, FLEXCAN_MSG_ID_STD, MB_RX_ANGLE, 0x7FFU);
    FLEXCAN_DRV_ConfigRxMb(INST_CAN, MB_RX_ANGLE, &rxAngleInfo, CAN_ID_ANGLE);
    FLEXCAN_DRV_Receive(INST_CAN, MB_RX_ANGLE, &rxData_angle);
}

// Control message status:
uint8_t can_control_received(void) {
    if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_CONTROL) != STATUS_BUSY) {
        return 1U;
    }
    return 0U;
}

// Emergency message status:
uint8_t can_emergency_received(void) {
    if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_EMERGENCY) != STATUS_BUSY) {
        return 1U;
    }
    return 0U;
}

// Angle message status:
uint8_t can_angle_received(void) {
    if (FLEXCAN_DRV_GetTransferStatus(INST_CAN, MB_RX_ANGLE) != STATUS_BUSY) {
        return 1U;
    }
    return 0U;
}

// Send status:
void can_send_status(uint16_t speedL, uint16_t speedR, uint8_t emg) {
    flexcan_data_info_t txInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_DLC_STATUS,
        .is_remote   = false
    };

    uint8_t data[CAN_DLC_STATUS];

    data[0] = (uint8_t)(speedL >> 8);
    data[1] = (uint8_t)(speedL & 0xFF);
    data[2] = (uint8_t)(speedR >> 8);
    data[3] = (uint8_t)(speedR & 0xFF);
    data[4] = emg;

    FLEXCAN_DRV_Send(INST_CAN, MB_TX_STATUS, &txInfo, CAN_ID_STATUS, data);
}
