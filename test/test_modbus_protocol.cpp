// Native (PC) test for the Arduino-independent logic in src/modbus_protocol.h.
// Build: g++ -std=c++17 -I src test/test_modbus_protocol.cpp -o test_modbus_protocol

#include "../src/config.h"
#include "../src/modbus_protocol.h"
#include <cstdio>

int failures = 0;

void check(bool cond, const char *label) {
    if (!cond) {
        std::printf("FAIL: %s\n", label);
        failures++;
    } else {
        std::printf("ok:   %s\n", label);
    }
}

int main() {
    // CRC16 against the known-good VFD1 STOP frame from main.cpp/simulate.py field testing.
    uint8_t stopFrame[8];
    buildModbusFrame(stopFrame, 0x01, 0x0006, 0x0001);
    uint16_t crc = (stopFrame[7] << 8) | stopFrame[6];
    check(crc == modbusCRC16(stopFrame, 6), "CRC16 matches the CRC embedded by buildModbusFrame");

    // Exact echo -> OK.
    uint8_t echo[8];
    buildModbusFrame(echo, 0x01, 0x0006, 0x0002);
    check(evaluateResponse(echo, 8, echo, 8) == ModbusResult::OK, "exact echo evaluates OK");

    // No bytes received -> TIMEOUT.
    check(evaluateResponse(echo, 8, echo, 0) == ModbusResult::TIMEOUT, "zero-length response is TIMEOUT");

    // One byte flipped -> MISMATCH.
    uint8_t corrupted[8];
    buildModbusFrame(corrupted, 0x01, 0x0006, 0x0002);
    corrupted[4] ^= 0xFF;
    check(evaluateResponse(echo, 8, corrupted, 8) == ModbusResult::MISMATCH, "corrupted echo is MISMATCH");

    // Modbus exception reply (addr, func|0x80, exceptionCode, CRC lo, CRC hi) -> EXCEPTION.
    uint8_t exc[5] = {0x01, 0x86, 0x02, 0, 0};  // 0x02 = Illegal Data Address
    uint16_t excCrc = modbusCRC16(exc, 3);
    exc[3] = excCrc & 0xFF;
    exc[4] = excCrc >> 8;
    check(evaluateResponse(echo, 8, exc, 5) == ModbusResult::EXCEPTION, "well-formed exception reply is EXCEPTION");

    // Exception reply with a bad CRC -> MISMATCH, not silently trusted as EXCEPTION.
    uint8_t badExc[5] = {0x01, 0x86, 0x02, 0x00, 0x00};
    check(evaluateResponse(echo, 8, badExc, 5) == ModbusResult::MISMATCH, "exception reply with bad CRC is MISMATCH");

    // Corrected register map sanity check: operation command is 0x0006, not the old 0x2000.
    uint8_t vfd1Run[8];
    buildModbusFrame(vfd1Run, 0x01, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);
    check(vfd1Run[0] == 0x01 && vfd1Run[2] == 0x00 && vfd1Run[3] == 0x06 && vfd1Run[5] == 0x02,
          "VFD1 RUN frame targets register 0x0006 with CMD_RUN_FORWARD (0x0002)");

    uint8_t vfd1Stop[8];
    buildModbusFrame(vfd1Stop, 0x01, REG_OPERATION_COMMAND, CMD_STOP);
    check(vfd1Stop[5] == 0x01, "VFD1 STOP frame writes CMD_STOP (0x0001), i.e. bit0");

    std::printf("\n%s\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
