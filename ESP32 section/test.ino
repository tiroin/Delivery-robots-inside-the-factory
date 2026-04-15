// =============================================================
//  ESP32 CAN Test Sender - for S32K144EVB
//  Framework : Arduino (ESP-IDF TWAI driver wrapped)
//  Wiring    : ESP32 TX_PIN -> CAN transceiver TXD
//              ESP32 RX_PIN -> CAN transceiver RXD
//              CAN transceiver H/L  -> S32K144 CAN H/L bus
// =============================================================

#include "driver/twai.h"

// -------------------------------------------------------
// PIN CONFIG
// -------------------------------------------------------
#define TX_PIN  GPIO_NUM_4
#define RX_PIN  GPIO_NUM_5

// -------------------------------------------------------
// CAN ID - phải khớp với firmware S32K144
// -------------------------------------------------------
#define CAN_ID_CONTROL    0x111   // ID bình thường  (S32K nhận trên filter này)
#define CAN_ID_EMERGENCY  0x001   // ID khẩn cấp     (S32K check trong logic)

// -------------------------------------------------------
// KỊCH BẢN TEST - chọn 1 trong 4 bằng cách uncomment
// -------------------------------------------------------
// #define TEST_MODE_NORMAL      // Chỉ gửi msg thường 0x111
// #define TEST_MODE_EMERGENCY   // Chỉ gửi emergency 0x001
#define TEST_MODE_MIXED       // Xen kẽ normal + emergency
// #define TEST_MODE_SEQUENCE       // Full sequence: nhiều ID, đa dạng payload

// -------------------------------------------------------
// HELPER: gửi 1 CAN frame
// -------------------------------------------------------
bool can_send(uint32_t id, const char* text, bool is_extended = false) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.identifier       = id;
    msg.extd             = is_extended ? 1 : 0;
    msg.data_length_code = 8;

    // Copy payload (tối đa 8 byte, pad 0 nếu ngắn hơn)
    memset(msg.data, 0, 8);
    size_t len = strlen(text);
    if (len > 8) len = 8;
    memcpy(msg.data, text, len);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        Serial.printf("[TX OK]  ID=0x%03X  data=\"%.8s\"\n", id, text);
        return true;
    } else {
        Serial.printf("[TX ERR] ID=0x%03X  err=%d\n", id, err);
        return false;
    }
}

// -------------------------------------------------------
// HELPER: in warning nếu ID khác 0x111
// (vì S32K filter chỉ accept 0x111 trong code hiện tại)
// -------------------------------------------------------
void warn_if_filtered(uint32_t id) {
    if (id != CAN_ID_CONTROL) {
        Serial.printf(
            "[WARN] ID=0x%03X se bi filter o S32K144! "
            "MB chi accept 0x111. Can mo rong filter de nhan.\n", id
        );
    }
}

// =============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32 CAN Test Sender ===");

    // Cấu hình TWAI (CAN) driver
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);

    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    // Nếu S32K144 dùng baudrate khác, đổi ở đây:
    // TWAI_TIMING_CONFIG_250KBITS()
    // TWAI_TIMING_CONFIG_1MBITS()

    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERROR] TWAI driver install failed!");
        while (1);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERROR] TWAI start failed!");
        while (1);
    }

    Serial.println("[OK] CAN bus ready @ 500 kbps");
    Serial.println("-----------------------------");

#ifdef TEST_MODE_NORMAL
    Serial.println("Mode: NORMAL only (0x111)");
#endif
#ifdef TEST_MODE_EMERGENCY
    Serial.println("Mode: EMERGENCY only (0x001)");
#endif
#ifdef TEST_MODE_MIXED
    Serial.println("Mode: MIXED - normal + emergency");
#endif
#ifdef TEST_MODE_SEQUENCE
    Serial.println("Mode: SEQUENCE - full multi-ID test");
#endif
}

// =============================================================
void loop() {

// -------------------------------------------------------
#ifdef TEST_MODE_NORMAL
// -------------------------------------------------------
    // Gửi message bình thường lên ID 0x111 mỗi 1 giây
    can_send(CAN_ID_CONTROL, "NiceToMt");
    delay(1000);
    can_send(CAN_ID_CONTROL, "Hello   ");
    delay(1000);
    can_send(CAN_ID_CONTROL, "TestMsg1");
    delay(1000);

// -------------------------------------------------------
#elif defined(TEST_MODE_EMERGENCY)
// -------------------------------------------------------
    // Gửi emergency message ID 0x001 liên tục
    // LƯU Ý: S32K144 filter chỉ nhận 0x111 → cần sửa firmware
    warn_if_filtered(CAN_ID_EMERGENCY);
    can_send(CAN_ID_EMERGENCY, "EMRGNCY!");
    delay(500);

// -------------------------------------------------------
#elif defined(TEST_MODE_MIXED)
// -------------------------------------------------------
    static uint32_t last_action_time = 0;
    static int counter = 0;
    static enum { STATE_NORMAL, STATE_STOPPING, STATE_WAIT_CLEAR } current_state = STATE_NORMAL;

    // --- PHẦN NHẬN: Luôn lắng nghe S32K gửi gì về ---
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, 0) == ESP_OK) { // Timeout = 0 để không chặn code
        Serial.printf("[RX từ S32K] ID=0x%03X | Data: ", rx_msg.identifier);
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            Serial.print((char)rx_msg.data[i]);
        }
        Serial.println();
    }

    // --- PHẦN GỬI: Điều khiển theo chu kỳ (Non-blocking) ---
    uint32_t now = millis();
    
    switch (current_state) {
        case STATE_NORMAL:
            if (now - last_action_time >= 800) { // Gửi mỗi 800ms
                last_action_time = now;
                counter++;
                
                if (counter % 5 == 0) {
                    // Chuyển sang gửi lệnh STOP
                    twai_message_t msg;
                    memset(&msg, 0, sizeof(msg));
                    msg.identifier = CAN_ID_EMERGENCY;
                    msg.data_length_code = 1;
                    msg.data[0] = 0xFF; 
                    twai_transmit(&msg, pdMS_TO_TICKS(10));
                    Serial.println("[!!!] SENT STOP (0xFF)");
                    current_state = STATE_STOPPING;
                } else {
                    char buf[9];
                    snprintf(buf, sizeof(buf), "RUN %03d", counter);
                    can_send(CAN_ID_CONTROL, buf);
                }
            }
            break;

        case STATE_STOPPING:
            // Đợi 5 giây (không dùng delay)
            if (now - last_action_time >= 5000) {
                last_action_time = now;
                // Gửi lệnh CLEAR
                twai_message_t msg;
                memset(&msg, 0, sizeof(msg));
                msg.identifier = CAN_ID_EMERGENCY;
                msg.data_length_code = 1;
                msg.data[0] = 0x00; 
                twai_transmit(&msg, pdMS_TO_TICKS(10));
                Serial.println("[OK] SENT CLEAR (0x00)");
                current_state = STATE_NORMAL;
            }
            break;
    }

// -------------------------------------------------------
#elif defined(TEST_MODE_SEQUENCE)
// -------------------------------------------------------
    // Kịch bản đầy đủ: gửi nhiều loại msg, dừng để quan sát
    Serial.println("\n--- [STEP 1] Warm-up: 3x normal messages ---");
    can_send(CAN_ID_CONTROL, "NiceToMt");  delay(300);
    can_send(CAN_ID_CONTROL, "Hello   ");  delay(300);
    can_send(CAN_ID_CONTROL, "TestData");  delay(1000);

    Serial.println("\n--- [STEP 2] Send EMERGENCY (ID 0x001) ---");
    warn_if_filtered(CAN_ID_EMERGENCY);
    can_send(CAN_ID_EMERGENCY, "EMRGNCY!");  delay(500);
    can_send(CAN_ID_EMERGENCY, "STOP!!! ");  delay(500);
    delay(1000);

    Serial.println("\n--- [STEP 3] Post-emergency normal messages ---");
    // Kiểm tra S32K144 có tiếp tục nhận sau emergency không
    can_send(CAN_ID_CONTROL, "AfterEMR");  delay(300);
    can_send(CAN_ID_CONTROL, "StillOK?");  delay(1000);

    Serial.println("\n--- [STEP 4] Unknown IDs (sẽ bị filter) ---");
    // Test các ID khác - S32K sẽ không nhận nếu không mở filter
    warn_if_filtered(0x200);
    can_send(0x200, "ID_0x200");  delay(300);
    warn_if_filtered(0x300);
    can_send(0x300, "ID_0x300");  delay(300);
    delay(1000);

    Serial.println("\n--- [STEP 5] Rapid fire normal messages ---");
    for (int i = 0; i < 10; i++) {
        char buf[9];
        snprintf(buf, sizeof(buf), "RAPID%03d", i);
        can_send(CAN_ID_CONTROL, buf);
        delay(100);
    }
    delay(2000);

    Serial.println("\n=== Sequence complete. Repeating in 5s... ===");
    delay(5000);

#endif
}
