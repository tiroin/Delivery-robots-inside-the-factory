#include "driver/twai.h"

#define TX_PIN  GPIO_NUM_4
#define RX_PIN  GPIO_NUM_5

#define CAN_ID_EMERGENCY  0x001
#define CAN_ID_CONTROL    0x111
#define CAN_ID_STATUS     0x222

void setup() {
    Serial.begin(115200);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println(">>> ESP32 STRESS TESTER READY <<<");
    }
}

void send_control(uint8_t code) {
    twai_message_t msg;
    msg.identifier = CAN_ID_CONTROL;
    msg.data_length_code = 1;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data[0] = code;
    // Gửi với timeout ngắn để không làm treo loop nếu bus bận
    twai_transmit(&msg, pdMS_TO_TICKS(5)); 
}

void send_emergency(bool active) {
    twai_message_t msg;
    msg.identifier = CAN_ID_EMERGENCY;
    msg.data_length_code = 1;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data[0] = active ? 0xFF : 0x00;
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    Serial.printf("\n--- [TX] %s EMERGENCY ---\n", active ? "KICH HOAT" : "TAT");
}

void receive_feedback() {
    twai_message_t rx_msg;
    // Đọc sạch hàng đợi nhận
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_STATUS) {
            uint16_t sL = (rx_msg.data[0] << 8) | rx_msg.data[1];
            uint16_t sR = (rx_msg.data[2] << 8) | rx_msg.data[3];
            uint8_t emg = rx_msg.data[4];
            Serial.printf("   [Feedback] Speed L:%d R:%d | EMG_Flag:%d\n", sL, sR, emg);
        }
    }
}

void loop() {
    // -----------------------------------------------------------------
    // GIAI ĐOẠN 1: QUÉT TOÀN BỘ 5 LỆNH CHUẨN (1-5)
    // -----------------------------------------------------------------
    Serial.println("\n>>> PHASE 1: TESTING ALL STANDARD COMMANDS (1-5) <<<");
    const char* names[] = {"", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"};
    
    for(int i = 1; i <= 5; i++) {
        Serial.printf("\n--- Gui lenh %d: %s ---\n", i, names[i]);
        send_control(i);
        // Đợi 1.5s để xem motor tăng tốc/giảm tốc mượt không
        unsigned long t = millis();
        while(millis() - t < 1500) {
            receive_feedback();
            delay(100);
        }
    }

    // -----------------------------------------------------------------
    // GIAI ĐOẠN 2: TEST LỆNH LẠ (UNKNOWN)
    // -----------------------------------------------------------------
    Serial.println("\n>>> PHASE 2: TESTING UNKNOWN COMMAND (CMD = 9) <<<");
    send_control(9); 
    delay(500);
    receive_feedback();

    // -----------------------------------------------------------------
    // GIAI ĐOẠN 3: STRESS TEST [REJECTED] - KHÓA VÀ SPAM LỆNH
    // -----------------------------------------------------------------
    Serial.println("\n>>> PHASE 3: STRESS TEST [REJECTED] LOG <<<");
    
    send_emergency(true); // Khóa hệ thống
    delay(200);

    Serial.println("Spam tat ca cac huong khi dang Emergency (Check PuTTY)...");
    for(int i = 1; i <= 4; i++) {
        Serial.printf("Spam lenh %d (%s)...\n", i, names[i]);
        for(int j = 0; j < 5; j++) {
            send_control(i); 
            receive_feedback();
            delay(50); // Tốc độ cao 50ms/lệnh để xem UART S32K có kịp in không
        }
    }

    send_emergency(false); // Mở khóa
    Serial.println("Da mo khoa. Hệ thống phải ACCEPT trở lại.");
    delay(1000);

    // -----------------------------------------------------------------
    // GIAI ĐOẠN 4: EMERGENCY GIỮA LUỒNG (RACE CONDITION)
    // -----------------------------------------------------------------
    Serial.println("\n>>> PHASE 4: EMERGENCY SURPRISE DURING MOVEMENT <<<");
    
    Serial.println("Robot dang TIEN (1)...");
    for(int i = 0; i < 20; i++) {
        send_control(1);
        if(i == 10) {
            send_emergency(true); // Cắt ngang khi đang chạy
        }
        receive_feedback();
        delay(100);
    }
    
    send_emergency(false);
    send_control(5); // Ép dừng hẳn
    Serial.println("Finish Loop. Waiting for next cycle...");
    delay(3000);
}
