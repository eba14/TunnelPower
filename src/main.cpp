// ESP32 firmware (PlatformIO esp32dev). Reads MCR aux contact, sequences VFD1 (cutter) and
// VFD2 (pump) over Modbus RTU/RS-485, with an X1 hardwire backup for VFD1.
// Start: pump RUN -> wait PUMP_LEAD_MS -> cutter RUN. Stop: cutter STOP -> wait -> pump STOP.
// Every write is ACK-verified against the VFD's echoed response and retried on failure.

#include <Arduino.h>
#include "config.h"
#include "modbus_protocol.h"

enum class State { IDLE, PUMP_LEAD_WAIT, RUNNING, CUTTER_SETTLE_WAIT, FAULT };
State currentState = State::IDLE;
unsigned long stateEnteredAt = 0;

HardwareSerial vfdSerial(2);  // UART2 for VFD bus; UART0 = USB debug Serial

uint8_t vfd1RunFrame[FRAME_LEN];
uint8_t vfd1StopFrame[FRAME_LEN];
uint8_t vfd2RunFrame[FRAME_LEN];
uint8_t vfd2StopFrame[FRAME_LEN];

void rs485Transmit(const uint8_t *frame, size_t len) {
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
size_t rs485Receive(uint8_t *buf, size_t bufLen, unsigned long timeoutMs) {
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

void clearRxBuffer() {
    while (vfdSerial.available()) vfdSerial.read();
}

ModbusResult sendModbusCommand(const uint8_t *frame, size_t len, const char *label) {
    ModbusResult result = ModbusResult::TIMEOUT;

    for (uint8_t attempt = 1; attempt <= MAX_RETRIES + 1; attempt++) {
        clearRxBuffer();
        rs485Transmit(frame, len);

        uint8_t resp[FRAME_LEN];
        size_t n = rs485Receive(resp, sizeof(resp), RESPONSE_TIMEOUT_MS);
        result = evaluateResponse(frame, len, resp, n);

        if (result == ModbusResult::OK) {
            Serial.print("[Modbus] "); Serial.print(label); Serial.println(" OK");
            return result;
        }

        Serial.print("[Modbus] "); Serial.print(label);
        Serial.print(" attempt "); Serial.print(attempt); Serial.print("/"); Serial.print(MAX_RETRIES + 1);
        Serial.print(" failed: "); Serial.println(resultToString(result));
    }

    Serial.print("[Modbus] "); Serial.print(label); Serial.println(" FAILED after all retries");
    return result;
}

bool attemptPumpStart() {
    return sendModbusCommand(vfd2RunFrame, FRAME_LEN, "VFD2 (pump) RUN") == ModbusResult::OK;
}

bool attemptCutterStart() {
    bool ok = sendModbusCommand(vfd1RunFrame, FRAME_LEN, "VFD1 (cutter) RUN") == ModbusResult::OK;
    if (ok) {
        digitalWrite(VFD1_X1_PIN, HIGH);
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
    return ok;
}

void attemptCutterStop() {
    digitalWrite(VFD1_X1_PIN, LOW);     // unconditional: doesn't wait on Modbus ack
    digitalWrite(STATUS_LED_PIN, LOW);
    sendModbusCommand(vfd1StopFrame, FRAME_LEN, "VFD1 (cutter) STOP");
}

void attemptPumpStop() {
    sendModbusCommand(vfd2StopFrame, FRAME_LEN, "VFD2 (pump) STOP");
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
