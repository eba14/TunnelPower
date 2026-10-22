// ESP32 firmware (PlatformIO esp32dev). Reads MCR aux contact, sequences VFD1 (cutter) and
// VFD2 (pump) over Modbus RTU/RS-485, with an X1 hardwire backup for VFD1.
// Start: pump RUN -> wait PUMP_LEAD_MS -> cutter RUN. Stop: cutter STOP -> wait -> pump STOP.
// Every write is ACK-verified against the VFD's echoed response and retried on failure.
// Speed control: send "FC <hz>" (cutter) or "FP <hz>" (pump) over USB Serial, e.g. "FC 45.5"
// -- a bring-up/test input, not the final remote-operator interface.
//
// The state-transition logic itself lives in fsm.h (Arduino-independent, unit-tested in
// test/test_fsm.cpp against a fake VFD). This file wires that logic to real hardware.

#include <Arduino.h>
#include "config.h"
#include "modbus_protocol.h"
#include "modbus_command.h"
#include "rs485_transport.h"
#include "fsm.h"

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

// A failed read is treated as "no fault" rather than triggering FAULT itself -- a transient
// comm glitch on a status poll shouldn't trip the machine; only a confirmed nonzero fault
// code should.
bool vfdHasFault(uint8_t addr, const char *label) {
    ModbusReadOutcome outcome = readModbusRegister(vfdTransport, addr, REG_FAULT_TRIPINFO, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
    if (outcome.result != ModbusResult::OK) {
        Serial.print("[Modbus] "); Serial.print(label); Serial.println(" fault check FAILED (treated as no fault)");
        return false;
    }
    if (outcome.value != 0) {
        Serial.print("[Modbus] "); Serial.print(label); Serial.print(" reports fault code 0x");
        Serial.println(outcome.value, HEX);
        return true;
    }
    return false;
}

// Concrete FsmActions: performs real Modbus commands and the hardware side effects (X1
// backup pin, status LED) that go with them.
class Esp32Actions : public FsmActions {
public:
    bool pumpStart() override {
        return sendVfdCommand(vfd2RunFrame, FRAME_LEN, "VFD2 (pump) RUN") == ModbusResult::OK;
    }

    bool cutterStart() override {
        bool ok = sendVfdCommand(vfd1RunFrame, FRAME_LEN, "VFD1 (cutter) RUN") == ModbusResult::OK;
        if (ok) {
            digitalWrite(VFD1_X1_PIN, HIGH);
            digitalWrite(STATUS_LED_PIN, HIGH);
        }
        return ok;
    }

    void cutterStop() override {
        digitalWrite(VFD1_X1_PIN, LOW);     // unconditional: doesn't wait on Modbus ack
        digitalWrite(STATUS_LED_PIN, LOW);
        sendVfdCommand(vfd1StopFrame, FRAME_LEN, "VFD1 (cutter) STOP");
    }

    void pumpStop() override {
        sendVfdCommand(vfd2StopFrame, FRAME_LEN, "VFD2 (pump) STOP");
    }

    bool anyFault() override {
        return vfdHasFault(VFD1_ADDR, "VFD1 (cutter)") || vfdHasFault(VFD2_ADDR, "VFD2 (pump)");
    }
};

Esp32Actions esp32Actions;
Fsm fsm;

// Writes a VFD's frequency setpoint. Can be called whether that VFD is running or not --
// it applies once running. Clamped here (not just at the Serial parser) so no caller can
// push an out-of-range value onto the register.
bool setFrequency(uint8_t vfdAddr, int centiHz, int minCentiHz, int maxCentiHz, const char *label) {
    centiHz = clampInt(centiHz, minCentiHz, maxCentiHz);

    uint8_t frame[FRAME_LEN];
    buildModbusFrame(frame, vfdAddr, MODBUS_FC_WRITE_SINGLE, REG_COMMAND_FREQUENCY, (uint16_t)centiHz);
    return sendVfdCommand(frame, FRAME_LEN, label) == ModbusResult::OK;
}

bool setCutterFrequency(int centiHz) {
    return setFrequency(VFD1_ADDR, centiHz, CUTTER_MIN_CENTIHERTZ, CUTTER_MAX_CENTIHERTZ, "VFD1 (cutter) frequency setpoint");
}

bool setPumpFrequency(int centiHz) {
    return setFrequency(VFD2_ADDR, centiHz, PUMP_MIN_CENTIHERTZ, PUMP_MAX_CENTIHERTZ, "VFD2 (pump) frequency setpoint");
}

// Shared by both Serial frequency commands below: parses "<number>" from after the prefix,
// clamps/sends via setFn, and reports the result -- the only difference between FC/FP is
// which VFD's setter and bounds get used.
void handleFrequencyCommand(const String &arg, bool (*setFn)(int), const char *label) {
    float hz = arg.toFloat();
    if (hz < 0.0f || hz > 60.0f) {
        Serial.print("[CMD] "); Serial.print(label); Serial.println(" frequency must be 0-60 Hz");
        return;
    }
    int centiHz = (int)(hz * 100.0f + 0.5f);
    if (setFn(centiHz)) {
        Serial.print("[CMD] "); Serial.print(label); Serial.print(" frequency set to "); Serial.print(hz); Serial.println(" Hz");
    } else {
        Serial.print("[CMD] "); Serial.print(label); Serial.println(" frequency setpoint FAILED");
    }
}

// Bring-up input over USB Serial: "FC <hz>" for the cutter, "FP <hz>" for the pump
// (e.g. "FC 45.5"). Not the final remote interface.
void processSerialLine(String line) {
    line.trim();
    if (line.length() == 0) return;
    line.toUpperCase();

    if (line.startsWith("FC ")) {
        handleFrequencyCommand(line.substring(3), setCutterFrequency, "Cutter");
    } else if (line.startsWith("FP ")) {
        handleFrequencyCommand(line.substring(3), setPumpFrequency, "Pump");
    } else {
        Serial.print("[CMD] Unknown command: "); Serial.println(line);
    }
}

// Accumulates bytes across loop() calls and dispatches each complete line. Deliberately not
// Serial.readStringUntil(): that blocks (up to Serial's timeout, 1000ms by default) waiting for
// a newline, which would stall MCR polling and VFD sequencing if one never arrives -- exactly
// the kind of blocking call the rest of this firmware was written to avoid.
void handleSerialCommand() {
    static String lineBuffer;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processSerialLine(lineBuffer);
            lineBuffer = "";
        } else if (c != '\r') {
            lineBuffer += c;
        }
    }
}

const char *stateName(State s) {
    switch (s) {
        case State::IDLE:               return "IDLE";
        case State::PUMP_LEAD_WAIT:     return "PUMP_LEAD_WAIT";
        case State::RUNNING:            return "RUNNING";
        case State::CUTTER_SETTLE_WAIT: return "CUTTER_SETTLE_WAIT";
        case State::FAULT:              return "FAULT";
        default:                        return "?";
    }
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

    buildModbusFrame(vfd1RunFrame,  VFD1_ADDR, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);
    buildModbusFrame(vfd1StopFrame, VFD1_ADDR, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_STOP);
    buildModbusFrame(vfd2RunFrame,  VFD2_ADDR, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);
    buildModbusFrame(vfd2StopFrame, VFD2_ADDR, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_STOP);

    Serial.println("[BOOT] POC ready - waiting for MCR signal");
}

void loop() {
    handleSerialCommand();

    bool mcrActive = digitalRead(MCR_INPUT_PIN) == HIGH;

    State before = fsm.state();
    fsm.tick(mcrActive, millis(), esp32Actions);
    State after = fsm.state();

    if (before != after) {
        Serial.print("[FSM] "); Serial.print(stateName(before));
        Serial.print(" -> "); Serial.println(stateName(after));
    }

    // FAULT indicator: blinks while in FAULT. Status LED during RUNNING is driven by
    // Esp32Actions::cutterStart()/cutterStop() instead; explicitly clear it on the way
    // out of FAULT since no cutter action runs on that transition to do it for us.
    if (after == State::FAULT) {
        digitalWrite(STATUS_LED_PIN, ((millis() / 250) % 2) ? HIGH : LOW);
    } else if (before == State::FAULT) {
        digitalWrite(STATUS_LED_PIN, LOW);
    }

    delay(100);
}
