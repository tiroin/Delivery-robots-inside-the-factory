#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

// Bien trang thai
uint16_t speedL = 0, speedR = 0;
uint8_t last_cmd = 0;
unsigned long last_receive_time = 0;
const char* cmd_names[] = {"NONE", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"};

void setup() {
    Serial.begin(115200);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED failed"));
    }
    
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.display();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println(">>> CAN READY <<<");
    }
}

void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("CMD: ");
    display.println(cmd_names[last_cmd]);
    display.println("---------------------");

    // Kiem tra timeout 2s tu S32K144
    if (millis() - last_receive_time > 2000) {
        display.setTextSize(1);
        display.setCursor(20, 35);
        display.println("CHUA NHAN LENH");
    } else {
        display.setTextSize(2);
        display.setCursor(0, 25);
        display.printf("L: %d\n", speedL);
        display.printf("R: %d", speedR);
    }
    display.display();
}

void send_control(uint8_t code) {
    twai_message_t msg;
    msg.identifier = CAN_ID_CONTROL;
    msg.data_length_code = 1;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data[0] = code;
    last_cmd = code;
    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void send_emergency(bool active) {
    twai_message_t msg;
    msg.identifier = CAN_ID_EMERGENCY;
    msg.data_length_code = 1;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data[0] = active ? 0xFF : 0x00;
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void receive_feedback() {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_STATUS) {
            speedL = (rx_msg.data[0] << 8) | rx_msg.data[1];
            speedR = (rx_msg.data[2] << 8) | rx_msg.data[3];
            last_receive_time = millis();
        }
    }
}

void loop() {
    // --- GIAI DOAN 1: TEST CAC LENH CO BAN (10s moi lenh) ---
    for(int i = 1; i <= 5; i++) {
        send_control((uint8_t)i);
        unsigned long start_time = millis();
        
        while(millis() - start_time < 10000) {
            receive_feedback();
            updateOLED();
            delay(100);
        }

        if(i < 5) {
            send_control(5); // STOP tam nghi 2s
            unsigned long start_stop = millis();
            while(millis() - start_stop < 2000) {
                receive_feedback();
                updateOLED();
                delay(100);
            }
        }
    }

    // --- GIAI DOAN 2: EMERGENCY TEST ---
    send_control(1); // Chay tien 3s roi ngat
    delay(3000);
    
    send_emergency(true); // Kich hoat dung khan cap
    unsigned long start_e = millis();
    while(millis() - start_e < 5000) { 
        receive_feedback();
        updateOLED();
        delay(100);
    }

    // --- FIX LOI VE 0 ---
    send_emergency(false); // Giai phong khan cap
    
    // Duy tri hien thi 3 giay truoc khi lap lai loop
    unsigned long wait_loop = millis();
    while(millis() - wait_loop < 3000) { 
        receive_feedback();
        updateOLED();
        delay(100);
    }
}