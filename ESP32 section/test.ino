#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX  5
#define CAN_RX  4

void setup() {
    Serial.begin(115200);

    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));  // 500 kbps

    if (!ESP32Can.begin()) {
        Serial.println("CAN init failed!");
        while (1);
    }
    Serial.println("CAN Ready!");
}

void sendHello() {
    CanFrame frame;
    frame.identifier = 0x111;
    frame.extd       = 0;        // Standard frame
    frame.data_length_code = 8;
    memcpy(frame.data, "Hello wo", 8);
    ESP32Can.writeFrame(frame);
    Serial.println("TX: Hello wo");
}

void loop() {
    // Gửi mỗi 1 giây
    static uint32_t lastSend = 0;
    if (millis() - lastSend >= 1000) {
        sendHello();
        lastSend = millis();
    }

    // Nhận
    CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 0)) {
        char buf[9] = {0};
        memcpy(buf, rxFrame.data, 8);
        Serial.print("RX: ");
        Serial.println(buf);
    }
}