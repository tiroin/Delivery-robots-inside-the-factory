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

// Tốc độ mặc định
#define DEFAULT_SPEED_L  150
#define DEFAULT_SPEED_R  150
#define INNER_SPEED      100

// Biến trạng thái
uint16_t speedL = 0, speedR = 0;
uint8_t  last_cmd = 0;
unsigned long last_receive_time = 0;
const char* cmd_names[] = {"NONE", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"};

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED failed"));
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.display();

    // Cấu hình TWAI cho Classic CAN (500kbps)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
        twai_start() == ESP_OK) {
        Serial.println(">>> CAN READY (CLASSIC MODE) <<<");
    }
}

// -------------------------------------------------------
// GỬI LỆNH DẠNG BINARY (5 BYTES)
// Byte 0: Command
// Byte 1-2: Speed_L (High, Low)
// Byte 3-4: Speed_R (High, Low)
// -------------------------------------------------------
void send_control(uint8_t cmd, uint16_t spd_L, uint16_t spd_R) {
    twai_message_t msg;
    msg.identifier       = CAN_ID_CONTROL;
    msg.flags            = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 5; // Chỉ dùng 5 bytes

    msg.data[0] = cmd;
    msg.data[1] = (uint8_t)(spd_L >> 8);   // Byte cao của Speed_L
    msg.data[2] = (uint8_t)(spd_L & 0xFF); // Byte thấp của Speed_L
    msg.data[3] = (uint8_t)(spd_R >> 8);   // Byte cao của Speed_R
    msg.data[4] = (uint8_t)(spd_R & 0xFF); // Byte thấp của Speed_R

    last_cmd = cmd;
    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        Serial.printf("[TX CTRL BINARY] CMD:%u | L:%u | R:%u\n", cmd, spd_L, spd_R);
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
            // S32K144 cũng gửi về dạng Binary 5 bytes
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
    display.print("CMD: ");
    display.println(cmd_names[last_cmd]);
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

void loop() {
    // 1. Quẹo trái:
    Serial.println(">>> TURN LEFT <<<");
    send_control(3, INNER_SPEED, DEFAULT_SPEED_R);

    unsigned long start_forward = millis();
    while (millis() - start_forward < 10000) {
        receive_feedback();
        updateOLED();
        delay(100);
    }

    // 2. DỪNG
    Serial.println(">>> STOP <<<");
    send_control(5, 0, 0);

    unsigned long start_stop = millis();
    while (millis() - start_stop < 5000) {
        receive_feedback();
        updateOLED();
        delay(100);
    }

    delay(1000);
}
