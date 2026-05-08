// ------------------------------------------------------------
// Target: ESP32 (PlatformIO esp32dev env)
//
// Flow:
//   MCR aux contact (24V via voltage divider) → GPIO 34
//   ESP32 FSM reads MCR state and sends:
//     - Modbus RTU RUN/STOP over RS-485 (UART2) to GK3000 VFD
//     - Digital RUN/STOP signal to VFD X1 terminal (hardwire backup)
// ------------------------------------------------------------

#include <Arduino.h>

// --- Pin assignments ---
#define MCR_INPUT_PIN   34  // MCR aux contact: 24V → 6.8kΩ/3.3kΩ divider → 3.3V
#define RS485_DE_PIN     4  // MAX485 DE+RE tied together (HIGH=transmit, LOW=receive)
#define VFD_TX_PIN      17  // UART2 TX → MAX485 DI → VFD RS-485
#define VFD_RX_PIN      16  // UART2 RX ← MAX485 RO ← VFD RS-485
#define VFD_X1_PIN      26  // Digital out → VFD X1 terminal (hardwire RUN/STOP backup)
#define STATUS_LED_PIN   2  // Onboard LED: HIGH = motor running

// --- GK3000 Modbus RTU frames (slave address 0x01) ---
// Function code 0x06 = Write Single Register
// Register 0x2000 = command word: 0x0001 = RUN, 0x0005 = STOP
// Last 2 bytes = CRC16
const uint8_t MODBUS_RUN[]  = {0x01, 0x06, 0x20, 0x00, 0x00, 0x01, 0x43, 0xCA};
const uint8_t MODBUS_STOP[] = {0x01, 0x06, 0x20, 0x00, 0x00, 0x05, 0x42, 0x09};

// --- FSM states ---
enum class State { IDLE, RUNNING };
State currentState = State::IDLE;

// UART2 used for VFD communication (UART0 = USB debug Serial)
HardwareSerial vfdSerial(2);

// --- Send a Modbus RTU frame over RS-485 ---
// Toggles DE/RE pin HIGH for transmit, then LOW to return to receive mode
void rs485Send(const uint8_t *frame, size_t len) {
    digitalWrite(RS485_DE_PIN, HIGH);
    vfdSerial.write(frame, len);
    vfdSerial.flush();                  // wait until all bytes are sent
    digitalWrite(RS485_DE_PIN, LOW);    // back to receive so we can read VFD response

    // Print frame to Serial Monitor for debugging
    Serial.print("[Modbus TX] ");
    for (size_t i = 0; i < len; i++) {
        if (frame[i] < 0x10) Serial.print("0");
        Serial.print(frame[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// --- FSM transition: IDLE → RUNNING ---
void sendRUN() {
    digitalWrite(VFD_X1_PIN, HIGH);         // hardwire backup signal to VFD X1
    digitalWrite(STATUS_LED_PIN, HIGH);
    rs485Send(MODBUS_RUN, sizeof(MODBUS_RUN));
    Serial.println("[FSM] -> RUNNING: Modbus RUN sent, X1 HIGH");
}

// --- FSM transition: RUNNING → IDLE ---
void sendSTOP() {
    digitalWrite(VFD_X1_PIN, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);
    rs485Send(MODBUS_STOP, sizeof(MODBUS_STOP));
    Serial.println("[FSM] -> IDLE: Modbus STOP sent, X1 LOW");
}

void setup() {
    Serial.begin(115200);
    vfdSerial.begin(9600, SERIAL_8N1, VFD_RX_PIN, VFD_TX_PIN); // match GK3000 P14.01

    pinMode(MCR_INPUT_PIN,  INPUT);
    pinMode(RS485_DE_PIN,   OUTPUT);
    pinMode(VFD_X1_PIN,     OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    // Safe default state — VFD stopped, RS-485 in receive mode
    digitalWrite(RS485_DE_PIN,   LOW);
    digitalWrite(VFD_X1_PIN,     LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.println("[BOOT] POC ready — waiting for MCR signal");
}

void loop() {
    // Read MCR aux contact (HIGH = MCR energized = motor should run)
    bool mcrActive = digitalRead(MCR_INPUT_PIN) == HIGH;

    switch (currentState) {
        case State::IDLE:
            if (mcrActive) {
                currentState = State::RUNNING;
                sendRUN();
            }
            break;

        case State::RUNNING:
            if (!mcrActive) {
                currentState = State::IDLE;
                sendSTOP();
            }
            break;
    }

    delay(100); // poll MCR every 100ms
}
