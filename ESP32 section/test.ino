#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

#define TX_PIN  GPIO_NUM_4
#define RX_PIN  GPIO_NUM_5
#define SDA_PIN 22
#define SCL_PIN 23

#define CAN_ID_EMERGENCY  0x001
#define CAN_ID_CONTROL    0x111
#define CAN_ID_STATUS     0x222

#define CMD_STOP        0x00
#define CMD_FORWARD     0x01
#define CMD_BACKWARD    0x02
#define CMD_TURN_LEFT   0x03
#define CMD_TURN_RIGHT  0x04

#define DEFAULT_SPEED          150U
#define MIN_SPEED              50U
#define MAX_SPEED              255U
#define CMD_REFRESH_PERIOD_MS  200UL
#define OLED_UPDATE_PERIOD_MS  300UL
#define CAN_TIMEOUT_MS         1500UL
#define GYRO_UPDATE_PERIOD_MS  50UL

#define MPU6050_ADDR            0x68
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_SMPLRT_DIV   0x19
#define MPU6050_RA_CONFIG       0x1A
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_WHO_AM_I     0x75
#define MPU6050_RA_GYRO_XOUT_H  0x43

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

const char* ssid = "Khanh";
const char* password = "khanhtran2004";

static uint8_t current_cmd = CMD_STOP;
static uint16_t current_spd_l = 0U;
static uint16_t current_spd_r = 0U;
static uint16_t feedback_l = 0U;
static uint16_t feedback_r = 0U;
static uint8_t emergency_flag = 0U;

static bool can_ready = false;
static unsigned long last_status_ms = 0UL;
static unsigned long last_cmd_tx_ms = 0UL;
static unsigned long last_oled_ms = 0UL;
static unsigned long last_gyro_ms = 0UL;
static unsigned long boot_ms = 0UL;

static uint32_t tx_ok_count = 0UL;
static uint32_t tx_fail_count = 0UL;
static uint32_t rx_status_count = 0UL;
static uint32_t rx_other_count = 0UL;

static uint32_t twai_state_dbg = 0UL;
static uint32_t tx_err_dbg = 0UL;
static uint32_t rx_err_dbg = 0UL;
static uint32_t bus_err_dbg = 0UL;

static bool gyro_ok = false;
static float gyro_x_dps = 0.0f;
static float gyro_y_dps = 0.0f;
static float gyro_z_dps = 0.0f;
static float yaw_deg = 0.0f;
static float gyro_z_offset = 0.0f;

static const char* cmd_name(uint8_t cmd) {
    switch (cmd) {
        case CMD_FORWARD:    return "FORWARD";
        case CMD_BACKWARD:   return "BACKWARD";
        case CMD_TURN_LEFT:  return "LEFT";
        case CMD_TURN_RIGHT: return "RIGHT";
        case CMD_STOP:
        default:             return "STOP";
    }
}

static void set_command(uint8_t cmd, uint16_t spd_l, uint16_t spd_r) {
    if (cmd == CMD_STOP) {
        current_cmd = CMD_STOP;
        current_spd_l = 0U;
        current_spd_r = 0U;
        return;
    }

    if (spd_l < MIN_SPEED) spd_l = MIN_SPEED;
    if (spd_r < MIN_SPEED) spd_r = MIN_SPEED;
    if (spd_l > MAX_SPEED) spd_l = MAX_SPEED;
    if (spd_r > MAX_SPEED) spd_r = MAX_SPEED;

    current_cmd = cmd;
    current_spd_l = spd_l;
    current_spd_r = spd_r;
}

static bool send_cmd(uint8_t cmd, uint16_t spd_l, uint16_t spd_r) {
    if (!can_ready) return false;

    twai_message_t msg = {};
    msg.identifier = CAN_ID_CONTROL;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 5;
    msg.data[0] = cmd;
    msg.data[1] = (uint8_t)(spd_l >> 8);
    msg.data[2] = (uint8_t)(spd_l & 0xFFU);
    msg.data[3] = (uint8_t)(spd_r >> 8);
    msg.data[4] = (uint8_t)(spd_r & 0xFFU);

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (ret == ESP_OK) {
        tx_ok_count++;
        return true;
    }

    tx_fail_count++;
    return false;
}

static void send_current_command_if_due(void) {
    unsigned long now = millis();
    if (now - last_cmd_tx_ms >= CMD_REFRESH_PERIOD_MS) {
        send_cmd(current_cmd, current_spd_l, current_spd_r);
        last_cmd_tx_ms = now;
    }
}

static void receive_feedback(void) {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_STATUS && rx_msg.data_length_code >= 5) {
            feedback_l = ((uint16_t)rx_msg.data[0] << 8) | rx_msg.data[1];
            feedback_r = ((uint16_t)rx_msg.data[2] << 8) | rx_msg.data[3];
            emergency_flag = rx_msg.data[4];
            last_status_ms = millis();
            rx_status_count++;
        } else {
            rx_other_count++;
        }
    }
}

static void update_twai_debug(void) {
    twai_status_info_t info;
    if (twai_get_status_info(&info) == ESP_OK) {
        twai_state_dbg = (uint32_t)info.state;
        tx_err_dbg = info.tx_error_counter;
        rx_err_dbg = info.rx_error_counter;
        bus_err_dbg = info.bus_error_count;
    }
}

static bool can_status_ok(void) {
    return (last_status_ms != 0UL) && ((millis() - last_status_ms) < CAN_TIMEOUT_MS);
}

static bool mpu_write_byte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool mpu_read_bytes(uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t n = Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
    if (n != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

static bool gyro_read_raw(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[6] = {0};
    if (!mpu_read_bytes(MPU6050_RA_GYRO_XOUT_H, data, 6)) return false;
    *gx = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *gy = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *gz = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    return true;
}

static void gyro_setup(void) {
    uint8_t who = 0U;
    gyro_ok = false;
    if (!mpu_read_bytes(MPU6050_RA_WHO_AM_I, &who, 1)) return;
    if (who != 0x68U) return;

    if (!mpu_write_byte(MPU6050_RA_PWR_MGMT_1, 0x00U)) return;
    delay(100);
    mpu_write_byte(MPU6050_RA_SMPLRT_DIV, 0x04U);
    mpu_write_byte(MPU6050_RA_CONFIG, 0x03U);
    mpu_write_byte(MPU6050_RA_GYRO_CONFIG, 0x00U);

    long z_sum = 0L;
    int16_t gx = 0, gy = 0, gz = 0;
    for (uint8_t i = 0; i < 80U; i++) {
        receive_feedback();
        if (gyro_read_raw(&gx, &gy, &gz)) {
            z_sum += gz;
        }
        delay(3);
    }

    gyro_z_offset = (float)z_sum / 80.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;
    yaw_deg = 0.0f;
    last_gyro_ms = millis();
    gyro_ok = true;
}

static void update_gyro_if_due(void) {
    unsigned long now = millis();
    if (now - last_gyro_ms < GYRO_UPDATE_PERIOD_MS) return;

    float dt = (last_gyro_ms == 0UL) ? 0.05f : ((float)(now - last_gyro_ms) / 1000.0f);
    last_gyro_ms = now;
    if (dt <= 0.001f || dt > 0.25f) dt = 0.05f;

    if (!gyro_ok) return;

    int16_t gx = 0, gy = 0, gz = 0;
    if (!gyro_read_raw(&gx, &gy, &gz)) {
        gyro_ok = false;
        return;
    }

    gyro_x_dps = (float)gx / 131.0f;
    gyro_y_dps = (float)gy / 131.0f;
    gyro_z_dps = ((float)gz - gyro_z_offset) / 131.0f;

    if (gyro_z_dps > -0.3f && gyro_z_dps < 0.3f) {
        gyro_z_dps = 0.0f;
    }

    yaw_deg += gyro_z_dps * dt;
    if (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    if (yaw_deg < -180.0f) yaw_deg += 360.0f;
}

static void update_oled_if_due(void) {
    unsigned long now = millis();
    if (now - last_oled_ms < OLED_UPDATE_PERIOD_MS) return;
    last_oled_ms = now;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("IP:");
    display.print(WiFi.localIP().toString());

    display.setCursor(0, 12);
    display.print("CMD:");
    display.print(cmd_name(current_cmd));
    display.print(" ");
    display.print(current_spd_l);

    display.setCursor(0, 24);
    display.print("CAN:");
    display.print(can_status_ok() ? "OK" : "TIMEOUT");
    display.print(" A:");
    display.print((last_status_ms == 0UL) ? 0UL : (now - last_status_ms));

    display.setCursor(0, 36);
    display.print("L:");
    display.print(feedback_l);
    display.print(" R:");
    display.print(feedback_r);

    display.setCursor(0, 48);
    display.print("GZ:");
    display.print(gyro_z_dps, 1);
    display.print(" Y:");
    display.print(yaw_deg, 1);

    display.display();
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot CAN Control</title>
<style>
body{font-family:monospace;background:#101010;color:#00ff88;margin:0;padding:18px;display:flex;flex-direction:column;align-items:center}h2{margin:8px}.box{width:330px;background:#1b1b1b;border:1px solid #333;border-radius:12px;padding:14px;margin:8px}.bad{border-color:#ff4444}.row{display:flex;justify-content:space-between;margin:6px 0}.v{color:white}button{font-family:monospace;font-size:18px;margin:6px;padding:16px 22px;border-radius:12px;border:2px solid #333;background:#202020;color:#00ff88}.stop{color:#ff4444;border-color:#ff4444}input{width:100%;accent-color:#00ff88}
</style></head><body>
<h2>ROBOT CAN CONTROL</h2>
<div class="box"><div class="row"><span>SPEED</span><span id="spdv" class="v">150</span></div><input id="spd" type="range" min="50" max="255" value="150" oninput="spdv.textContent=this.value"></div>
<div><button onclick="send('forward')">FORWARD</button></div>
<div><button onclick="send('left')">LEFT</button><button class="stop" onclick="send('stop')">STOP</button><button onclick="send('right')">RIGHT</button></div>
<div><button onclick="send('backward')">BACKWARD</button></div>
<div class="box" id="canbox"><h3>// CAN</h3><div class="row"><span>STATUS</span><span id="ok" class="v">--</span></div><div class="row"><span>AGE</span><span id="age" class="v">--</span></div><div class="row"><span>TWAI</span><span id="twai" class="v">--</span></div><div class="row"><span>TX/RX ERR</span><span id="err" class="v">--</span></div><div class="row"><span>BUS ERR</span><span id="bus" class="v">--</span></div><div class="row"><span>RX STATUS</span><span id="rxs" class="v">--</span></div><div class="row"><span>RX OTHER</span><span id="rxo" class="v">--</span></div><div class="row"><span>TX OK/FAIL</span><span id="txc" class="v">--</span></div></div>
<div class="box"><h3>// MOTOR</h3><div class="row"><span>CMD</span><span id="cmd" class="v">--</span></div><div class="row"><span>SET L/R</span><span id="set" class="v">--</span></div><div class="row"><span>FB L/R</span><span id="fb" class="v">--</span></div><div class="row"><span>EMG</span><span id="emg" class="v">--</span></div></div>
<div class="box" id="gyrobox"><h3>// GYRO LOCAL</h3><div class="row"><span>MPU</span><span id="mpu" class="v">--</span></div><div class="row"><span>GX</span><span id="gx" class="v">--</span></div><div class="row"><span>GY</span><span id="gy" class="v">--</span></div><div class="row"><span>GZ</span><span id="gz" class="v">--</span></div><div class="row"><span>YAW</span><span id="yaw" class="v">--</span></div><button onclick="fetch('/zero_gyro')">ZERO YAW</button></div>
<script>
function send(c){const s=document.getElementById('spd').value;fetch('/cmd?c='+c+'&s='+s)}
setInterval(()=>{fetch('/status').then(r=>r.json()).then(d=>{ok.textContent=d.ok?'OK':'TIMEOUT';age.textContent=d.age+' ms';twai.textContent=d.twai;err.textContent=d.txerr+'/'+d.rxerr;bus.textContent=d.buserr;rxs.textContent=d.rxs;rxo.textContent=d.rxo;txc.textContent=d.txok+'/'+d.txfail;cmd.textContent=d.cmd;set.textContent=d.sl+'/'+d.sr;fb.textContent=d.fl+'/'+d.fr;emg.textContent=d.emg;mpu.textContent=d.mpu?'OK':'ERR';gx.textContent=d.gx+' dps';gy.textContent=d.gy+' dps';gz.textContent=d.gz+' dps';yaw.textContent=d.yaw+' deg';canbox.className=d.ok?'box':'box bad';gyrobox.className=d.mpu?'box':'box bad';});},300);
</script></body></html>
)rawliteral";

static void handle_root(void) {
    server.send(200, "text/html", INDEX_HTML);
}

static void handle_cmd(void) {
    String c = server.arg("c");
    uint16_t spd = (uint16_t)server.arg("s").toInt();
    uint8_t cmd = CMD_STOP;

    if (c == "forward") cmd = CMD_FORWARD;
    else if (c == "backward") cmd = CMD_BACKWARD;
    else if (c == "left") cmd = CMD_TURN_LEFT;
    else if (c == "right") cmd = CMD_TURN_RIGHT;
    else cmd = CMD_STOP;

    if (cmd == CMD_STOP) {
        set_command(CMD_STOP, 0U, 0U);
    } else {
        set_command(cmd, spd, spd);
    }

    send_cmd(current_cmd, current_spd_l, current_spd_r);
    last_cmd_tx_ms = millis();
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handle_zero_gyro(void) {
    yaw_deg = 0.0f;
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handle_status(void) {
    unsigned long now = millis();
    unsigned long age = (last_status_ms == 0UL) ? (now - boot_ms) : (now - last_status_ms);
    bool ok = can_status_ok();

    String json = "{";
    json += "\"ok\":" + String(ok ? "true" : "false");
    json += ",\"age\":" + String(age);
    json += ",\"twai\":" + String(twai_state_dbg);
    json += ",\"txerr\":" + String(tx_err_dbg);
    json += ",\"rxerr\":" + String(rx_err_dbg);
    json += ",\"buserr\":" + String(bus_err_dbg);
    json += ",\"rxs\":" + String(rx_status_count);
    json += ",\"rxo\":" + String(rx_other_count);
    json += ",\"txok\":" + String(tx_ok_count);
    json += ",\"txfail\":" + String(tx_fail_count);
    json += ",\"cmd\":\"" + String(cmd_name(current_cmd)) + "\"";
    json += ",\"sl\":" + String(current_spd_l);
    json += ",\"sr\":" + String(current_spd_r);
    json += ",\"fl\":" + String(feedback_l);
    json += ",\"fr\":" + String(feedback_r);
    json += ",\"emg\":" + String(emergency_flag);
    json += ",\"mpu\":" + String(gyro_ok ? "true" : "false");
    json += ",\"gx\":" + String(gyro_x_dps, 2);
    json += ",\"gy\":" + String(gyro_y_dps, 2);
    json += ",\"gz\":" + String(gyro_z_dps, 2);
    json += ",\"yaw\":" + String(yaw_deg, 2);
    json += "}";

    server.send(200, "application/json", json);
}

static void can_setup(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            can_ready = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    boot_ms = millis();
    last_status_ms = 0UL;

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("CAN web control");
    display.setCursor(0, 12);
    display.print("Gyro local only");
    display.display();

    can_setup();
    set_command(CMD_STOP, 0U, 0U);
    gyro_setup();

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        receive_feedback();
        update_twai_debug();
        update_gyro_if_due();
        delay(100);
    }

    server.on("/", handle_root);
    server.on("/cmd", handle_cmd);
    server.on("/status", handle_status);
    server.on("/zero_gyro", handle_zero_gyro);
    server.begin();
}

void loop() {
    server.handleClient();
    receive_feedback();
    update_twai_debug();
    update_gyro_if_due();
    send_current_command_if_due();
    update_oled_if_due();
    delay(1);
}
