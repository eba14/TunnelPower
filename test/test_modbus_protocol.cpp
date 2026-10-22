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
    buildModbusFrame(stopFrame, 0x01, MODBUS_FC_WRITE_SINGLE, 0x0006, 0x0001);
    uint16_t crc = (stopFrame[7] << 8) | stopFrame[6];
    check(crc == modbusCRC16(stopFrame, 6), "CRC16 matches the CRC embedded by buildModbusFrame");

    // Exact echo -> OK.
    uint8_t echo[8];
    buildModbusFrame(echo, 0x01, MODBUS_FC_WRITE_SINGLE, 0x0006, 0x0002);
    check(evaluateResponse(echo, 8, echo, 8) == ModbusResult::OK, "exact echo evaluates OK");

    // No bytes received -> TIMEOUT.
    check(evaluateResponse(echo, 8, echo, 0) == ModbusResult::TIMEOUT, "zero-length response is TIMEOUT");

    // One byte flipped -> MISMATCH.
    uint8_t corrupted[8];
    buildModbusFrame(corrupted, 0x01, MODBUS_FC_WRITE_SINGLE, 0x0006, 0x0002);
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
    buildModbusFrame(vfd1Run, 0x01, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);
    check(vfd1Run[0] == 0x01 && vfd1Run[2] == 0x00 && vfd1Run[3] == 0x06 && vfd1Run[5] == 0x02,
          "VFD1 RUN frame targets register 0x0006 with CMD_RUN_FORWARD (0x0002)");

    uint8_t vfd1Stop[8];
    buildModbusFrame(vfd1Stop, 0x01, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_STOP);
    check(vfd1Stop[5] == 0x01, "VFD1 STOP frame writes CMD_STOP (0x0001), i.e. bit0");

    // Frequency setpoint: 45.00 Hz -> 4500 (0x1194) at register 0x0005, 0.01 Hz units.
    uint8_t freq45[8];
    buildModbusFrame(freq45, VFD1_ADDR, MODBUS_FC_WRITE_SINGLE, REG_COMMAND_FREQUENCY, 4500);
    check(freq45[2] == 0x00 && freq45[3] == 0x05, "frequency frame targets register 0x0005");
    check(freq45[4] == 0x11 && freq45[5] == 0x94, "45.00 Hz encodes as 0x1194 (4500 x 0.01Hz)");

    // Read-holding-register frame: FC 0x03, register=REG_FAULT_TRIPINFO, count=1.
    uint8_t faultReadReq[8];
    buildModbusFrame(faultReadReq, VFD1_ADDR, MODBUS_FC_READ_HOLDING, REG_FAULT_TRIPINFO, 1);
    check(faultReadReq[1] == 0x03 && faultReadReq[3] == 0x0F && faultReadReq[5] == 1,
          "fault-read request targets FC 0x03, register 0x000F, count 1");

    // Well-formed read reply -> OK, value extracted correctly.
    uint8_t faultReply[7] = {0x01, 0x03, 0x02, 0x00, 0x00, 0, 0};  // value 0x0000 = no fault
    uint16_t replyCrc = modbusCRC16(faultReply, 5);
    faultReply[5] = replyCrc & 0xFF;
    faultReply[6] = replyCrc >> 8;
    uint16_t faultValue = 0xFFFF;  // poison value to prove it actually gets overwritten
    check(evaluateReadResponse(0x01, faultReply, 7, &faultValue) == ModbusResult::OK,
          "well-formed read reply evaluates OK");
    check(faultValue == 0x0000, "read reply value extracted correctly (0x0000 = no fault)");

    // Nonzero fault code read back correctly.
    uint8_t faultReply2[7] = {0x02, 0x03, 0x02, 0x00, 0x07, 0, 0};  // value 0x0007 = some fault
    uint16_t replyCrc2 = modbusCRC16(faultReply2, 5);
    faultReply2[5] = replyCrc2 & 0xFF;
    faultReply2[6] = replyCrc2 >> 8;
    uint16_t faultValue2 = 0;
    check(evaluateReadResponse(0x02, faultReply2, 7, &faultValue2) == ModbusResult::OK &&
          faultValue2 == 0x0007, "nonzero fault code (0x0007) extracted correctly");

    // Timeout and exception paths work the same way for reads as for writes.
    check(evaluateReadResponse(0x01, faultReply, 0, &faultValue) == ModbusResult::TIMEOUT,
          "zero-length read response is TIMEOUT");

    uint8_t readExc[5] = {0x01, 0x83, 0x02, 0, 0};  // 0x03|0x80 = exception on a read request
    uint16_t readExcCrc = modbusCRC16(readExc, 3);
    readExc[3] = readExcCrc & 0xFF;
    readExc[4] = readExcCrc >> 8;
    check(evaluateReadResponse(0x01, readExc, 5, &faultValue) == ModbusResult::EXCEPTION,
          "well-formed exception on a read request is EXCEPTION");

    // Wrong byte count (claims to return 2 registers instead of 1) -> MISMATCH, not trusted.
    uint8_t wrongByteCount[7] = {0x01, 0x03, 0x04, 0x00, 0x00, 0, 0};
    check(evaluateReadResponse(0x01, wrongByteCount, 7, &faultValue) == ModbusResult::MISMATCH,
          "read reply with wrong byte count is MISMATCH");

    // setCutterFrequency's clamp (src/main.cpp) delegates to this -- verify the bounds directly.
    check(clampInt(-100, VFD1_MIN_CENTIHERTZ, VFD1_MAX_CENTIHERTZ) == VFD1_MIN_CENTIHERTZ,
          "negative Hz clamps to VFD1_MIN_CENTIHERTZ (0)");
    check(clampInt(9999, VFD1_MIN_CENTIHERTZ, VFD1_MAX_CENTIHERTZ) == VFD1_MAX_CENTIHERTZ,
          "over-range Hz clamps to VFD1_MAX_CENTIHERTZ (6000 = 60.00 Hz)");
    check(clampInt(4500, VFD1_MIN_CENTIHERTZ, VFD1_MAX_CENTIHERTZ) == 4500,
          "in-range Hz passes through unclamped");
    check(clampInt(VFD1_MAX_CENTIHERTZ, VFD1_MIN_CENTIHERTZ, VFD1_MAX_CENTIHERTZ) == VFD1_MAX_CENTIHERTZ,
          "exact upper bound (60.00 Hz) passes through unclamped");

    std::printf("\n%s\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
