// ESP32 firmware (PlatformIO esp32dev). Reads MCR aux contact, sequences VFD1 (cutter) and
// VFD2 (pump) over Modbus RTU/RS-485, with an X1 hardwire backup for VFD1.
// Start: pump RUN -> wait PUMP_LEAD_MS -> cutter RUN. Stop: cutter STOP -> wait -> pump STOP.
// Every write is ACK-verified against the VFD's echoed response and retried on failure.
// Cutter speed: send "F <hz>" over USB Serial (e.g. "F 45.5") -- a bring-up/test input,
// not the final remote-operator interface.

#include <Arduino.h>
#include "config.h"
#include "modbus_protocol.h"
#include "modbus_command.h"
#include "rs485_transport.h"

enum class State { IDLE, PUMP_LEAD_WAIT, RUNNING, CUTTER_SETTLE_WAIT, FAULT };
State currentState = State::IDLE;
unsigned long stateEnteredAt = 0;

HardwareSerial vfdSerial(2);  // UART2 for VFD bus; UART0 = USB debug Serial

uint8_t vfd1RunFrame[FRAME_LEN];
uint8_t vfd1StopFrame[FRAME_LEN];
uint8_t vfd2RunFrame[FRAME_LEN];
uint8_t vfd2StopFrame[FRAME_LEN];

class Esp32Rs485Transport : public Rs485Transport {
public:
    void transmit(const uint8_t *frame, size_t len) override {
        while (vfdSerial.available()) vfdSerial.read();  // drop stale bytes before this exchange

        digitalWrite(RS485_DE_PIN, HIGH);
        vfdSerial.write(frame, len);
        vfdSerial.flush();
        digitalWrite(RS485_DE_PIN, LOW);

        Serial.print("[Modbus TX] ");
        for (size_t i = 0; i < len; i++) {
            if (frame[i] < 0x10) Serial.print("0");
            Serial.print(frame[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Reads until timeoutMs elapses or the bus is quiet for INTERBYTE_SILENCE_MS (frame boundary).
    size_t receive(uint8_t *buf, size_t bufLen, unsigned long timeoutMs) override {
        size_t count = 0;
        unsigned long start = millis();
        unsigned long lastByteAt = start;

        while (millis() - start < timeoutMs) {
            if (vfdSerial.available()) {
                uint8_t b = vfdSerial.read();
                if (count < bufLen) buf[count++] = b;
                lastByteAt = millis();
            } else if (count > 0 && millis() - lastByteAt >= INTERBYTE_SILENCE_MS) {
                break;
            }
        }
        return count;
    }
};

Esp32Rs485Transport vfdTransport;

ModbusResult sendVfdCommand(const uint8_t *frame, size_t len, const char *label) {
    ModbusCommandOutcome outcome = sendModbusCommand(vfdTransport, frame, len, MAX_RETRIES, RESPONSE_TIMEOUT_MS);

    Serial.print("[Modbus] "); Serial.print(label);
    if (outcome.result == ModbusResult::OK) {
        Serial.print(" OK (attempt "); Serial.print(outcome.attempts); Serial.println(")");
    } else {
        Serial.print(" FAILED after "); Serial.print(outcome.attempts);
        Serial.print(" attempt(s): "); Serial.println(resultToString(outcome.result));
    }
    return outcome.result;
}

bool attemptPumpStart() {
    return sendVfdCommand(vfd2RunFrame, FRAME_LEN, "VFD2 (pump) RUN") == ModbusResult::OK;
}

bool attemptCutterStart() {
    bool ok = sendVfdCommand(vfd1RunFrame, FRAME_LEN, "VFD1 (cutter) RUN") == ModbusResult::OK;
    if (ok) {
        digitalWrite(VFD1_X1_PIN, HIGH);
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
    return ok;
}

void attemptCutterStop() {
    digitalWrite(VFD1_X1_PIN, LOW);     // unconditional: doesn't wait on Modbus ack
    digitalWrite(STATUS_LED_PIN, LOW);
    sendVfdCommand(vfd1StopFrame, FRAME_LEN, "VFD1 (cutter) STOP");
}

void attemptPumpStop() {
    sendVfdCommand(vfd2StopFrame, FRAME_LEN, "VFD2 (pump) STOP");
}

// Writes VFD1's frequency setpoint. Can be called whether the cutter is running or not --
// the VFD applies it once running. Clamped here (not just at the Serial parser) so no
// caller can push an out-of-range value onto the register.
bool setCutterFrequency(int centiHz) {
    centiHz = clampInt(centiHz, VFD1_MIN_CENTIHERTZ, VFD1_MAX_CENTIHERTZ);

    uint8_t frame[FRAME_LEN];
    buildModbusFrame(frame, VFD1_ADDR, REG_COMMAND_FREQUENCY, (uint16_t)centiHz);
    return sendVfdCommand(frame, FRAME_LEN, "VFD1 (cutter) frequency setpoint") == ModbusResult::OK;
}

// Bring-up input: "F <hz>" over USB Serial, e.g. "F 45.5". Not the final remote interface.
void handleSerialCommand() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("F ") || line.startsWith("f ")) {
        float hz = line.substring(2).toFloat();
        if (hz < 0.0f || hz > 60.0f) {
            Serial.println("[CMD] Frequency must be 0-60 Hz");
            return;
        }
        int centiHz = (int)(hz * 100.0f + 0.5f);
        if (setCutterFrequency(centiHz)) {
            Serial.print("[CMD] VFD1 frequency set to "); Serial.print(hz); Serial.println(" Hz");
        } else {
            Serial.println("[CMD] Frequency setpoint FAILED");
        }
    } else {
        Serial.print("[CMD] Unknown command: "); Serial.println(line);
    }
}

void enterState(State s) {
    currentState = s;
    stateEnteredAt = millis();
}

void setup() {
    Serial.begin(115200);
    vfdSerial.begin(9600, SERIAL_8N1, VFD_RX_PIN, VFD_TX_PIN);

    pinMode(MCR_INPUT_PIN,  INPUT);
    pinMode(RS485_DE_PIN,   OUTPUT);
    pinMode(VFD1_X1_PIN,    OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    digitalWrite(RS485_DE_PIN,   LOW);
    digitalWrite(VFD1_X1_PIN,    LOW);
    digitalWrite(STATUS_LED_PIN, LOW);

    buildModbusFrame(vfd1RunFrame,  VFD1_ADDR, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);
    buildModbusFrame(vfd1StopFrame, VFD1_ADDR, REG_OPERATION_COMMAND, CMD_STOP);
    buildModbusFrame(vfd2RunFrame,  VFD2_ADDR, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);
    buildModbusFrame(vfd2StopFrame, VFD2_ADDR, REG_OPERATION_COMMAND, CMD_STOP);

    Serial.println("[BOOT] POC ready - waiting for MCR signal");
}

void loop() {
    handleSerialCommand();

    bool mcrActive = digitalRead(MCR_INPUT_PIN) == HIGH;

    switch (currentState) {
        case State::IDLE:
            if (mcrActive) {
                if (attemptPumpStart()) {
                    enterState(State::PUMP_LEAD_WAIT);
                } else {
                    Serial.println("[FSM] -> FAULT: pump failed to start");
                    enterState(State::FAULT);
                }
            }
            break;

        case State::PUMP_LEAD_WAIT:
            if (!mcrActive) {
                Serial.println("[FSM] MCR dropped during pump lead - aborting start");
                attemptPumpStop();
                enterState(State::IDLE);
            } else if (millis() - stateEnteredAt >= PUMP_LEAD_MS) {
                if (attemptCutterStart()) {
                    Serial.println("[FSM] -> RUNNING");
                    enterState(State::RUNNING);
                } else {
                    Serial.println("[FSM] -> FAULT: cutter failed to start; stopping pump");
                    attemptPumpStop();
                    enterState(State::FAULT);
                }
            }
            break;

        case State::RUNNING:
            if (!mcrActive) {
                attemptCutterStop();
                enterState(State::CUTTER_SETTLE_WAIT);
            }
            break;

        case State::CUTTER_SETTLE_WAIT:
            if (millis() - stateEnteredAt >= CUTTER_STOP_SETTLE_MS) {
                attemptPumpStop();
                Serial.println("[FSM] -> IDLE");
                enterState(State::IDLE);
            }
            break;

        case State::FAULT:
            digitalWrite(STATUS_LED_PIN, ((millis() / 250) % 2) ? HIGH : LOW);
            if (!mcrActive) {
                digitalWrite(STATUS_LED_PIN, LOW);
                Serial.println("[FSM] FAULT cleared (MCR cycled) -> IDLE");
                enterState(State::IDLE);
            }
            break;
    }

    delay(100);
}
