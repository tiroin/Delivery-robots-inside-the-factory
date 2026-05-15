#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <stdio.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define TX_PIN  GPIO_NUM_4
#define RX_PIN  GPIO_NUM_5
#define SDA_PIN 22
#define SCL_PIN 23

#define CAN_ID_EMERGENCY  0x001
#define CAN_ID_CONTROL    0x111
#define CAN_ID_STATUS     0x222

// -------------------------------------------------------
// CMD ENUM (khớp với S32K144 - main.c)
// -------------------------------------------------------
#define CMD_STOP        0x00
#define CMD_FORWARD     0x01
#define CMD_BACKWARD    0x02
#define CMD_TURN_LEFT   0x03
#define CMD_TURN_RIGHT  0x04

// -------------------------------------------------------
// TỐC ĐỘ MẶC ĐỊNH
// -------------------------------------------------------
#define DEFAULT_SPEED   150

// Biến trạng thái
uint16_t speedL = 0, speedR = 0;
unsigned long last_receive_time = 0;

// Đếm vòng lặp để xen kẽ LEFT / RIGHT
static uint8_t turn_toggle = 0;

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED failed"));
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.display();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
        twai_start() == ESP_OK) {
        Serial.println(">>> CAN READY (CLASSIC MODE) <<<");
    }
}

void send_cmd(uint8_t cmd, uint16_t spd_L, uint16_t spd_R) {
    twai_message_t msg;
    msg.identifier       = CAN_ID_CONTROL;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 5;

    msg.data[0] = cmd;
    msg.data[1] = (uint8_t)(spd_L >> 8);
    msg.data[2] = (uint8_t)(spd_L & 0xFF);
    msg.data[3] = (uint8_t)(spd_R >> 8);
    msg.data[4] = (uint8_t)(spd_R & 0xFF);

    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        const char* cmd_str = "UNKNOWN";
        switch (cmd) {
            case CMD_STOP:       cmd_str = "STOP";       break;
            case CMD_FORWARD:    cmd_str = "FORWARD";    break;
            case CMD_BACKWARD:   cmd_str = "BACKWARD";   break;
            case CMD_TURN_LEFT:  cmd_str = "TURN_LEFT";  break;
            case CMD_TURN_RIGHT: cmd_str = "TURN_RIGHT"; break;
        }
        Serial.printf("[TX] CMD:%s | L:%u | R:%u\n", cmd_str, spd_L, spd_R);
    } else {
        Serial.println("[ERROR] Failed to send CAN message");
    }
}

void send_emergency(bool active) {
    twai_message_t msg;
    msg.identifier       = CAN_ID_EMERGENCY;
    msg.data_length_code = 1;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data[0]          = active ? 0xFF : 0x00;
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    Serial.printf("[TX EMG] %s\n", active ? "LOCK (0xFF)" : "RELEASE (0x00)");
}

void receive_feedback() {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_STATUS) {
            speedL = ((uint16_t)rx_msg.data[0] << 8) | rx_msg.data[1];
            speedR = ((uint16_t)rx_msg.data[2] << 8) | rx_msg.data[3];
            last_receive_time = millis();
        }
    }
}

void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("STATUS:");
    display.println("---------------------");

    if (millis() - last_receive_time > 2000) {
        display.setCursor(20, 35);
        display.println("NO FEEDBACK");
    } else {
        display.setTextSize(2);
        display.setCursor(0, 25);
        display.printf("L:%d\n", speedL);
        display.printf("R:%d",   speedR);
    }
    display.display();
}

// Gửi lệnh liên tục trong duration_ms, refresh mỗi interval_ms
void run_cmd_for(uint8_t cmd, uint16_t spd_L, uint16_t spd_R,
                 unsigned long duration_ms, unsigned long interval_ms) {
    send_cmd(cmd, spd_L, spd_R);
    unsigned long t_start = millis();
    unsigned long t_last  = t_start;
    while (millis() - t_start < duration_ms) {
        if (millis() - t_last >= interval_ms) {
            send_cmd(cmd, spd_L, spd_R);
            t_last = millis();
        }
        receive_feedback();
        updateOLED();
        delay(50);
    }
}

// Dừng và giữ trong duration_ms (vẫn nhận feedback & cập nhật OLED)
void run_stop_for(unsigned long duration_ms) {
    send_cmd(CMD_STOP, 0, 0);
    unsigned long t_start = millis();
    while (millis() - t_start < duration_ms) {
        receive_feedback();
        updateOLED();
        delay(50);
    }
}

void loop() {
    // 1. Quẹo trái 5 giây
    Serial.println(">>> TURN LEFT 5s <<<");
    run_cmd_for(CMD_TURN_LEFT, 70, DEFAULT_SPEED, 5000, 200);

    // 2. Quẹo phải 5 giây
    // Serial.println(">>> TURN RIGHT 5s <<<");
    // run_cmd_for(CMD_TURN_RIGHT, DEFAULT_SPEED, 70, 5000, 200);

    // 3. Dừng 5 giây
    Serial.println(">>> STOP 5s <<<");
    run_stop_for(5000);
}
