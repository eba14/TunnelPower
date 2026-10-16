#pragma once
#include <cstdint>
#include <cstddef>

// Modbus RTU CRC16 (poly 0xA001, init 0xFFFF)
inline uint16_t modbusCRC16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

// Builds a Write Single Register (FC 0x06) frame, CRC included.
inline void buildModbusFrame(uint8_t *out, uint8_t addr, uint16_t reg, uint16_t value) {
    out[0] = addr;
    out[1] = 0x06;
    out[2] = reg >> 8;
    out[3] = reg & 0xFF;
    out[4] = value >> 8;
    out[5] = value & 0xFF;
    uint16_t crc = modbusCRC16(out, 6);
    out[6] = crc & 0xFF;
    out[7] = crc >> 8;
}

inline int clampInt(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

enum class ModbusResult { OK, TIMEOUT, MISMATCH, EXCEPTION };

inline const char *resultToString(ModbusResult result) {
    switch (result) {
        case ModbusResult::OK:        return "OK";
        case ModbusResult::TIMEOUT:   return "no response (timeout)";
        case ModbusResult::MISMATCH:  return "response mismatch";
        case ModbusResult::EXCEPTION: return "VFD returned exception";
        default:                      return "unknown";
    }
}

// A successful FC 0x06 reply is an exact echo of the request; else check for a Modbus exception.
inline ModbusResult evaluateResponse(const uint8_t *sentFrame, size_t sentLen, const uint8_t *resp, size_t n) {
    if (n == 0) return ModbusResult::TIMEOUT;

    if (n >= 5 && (resp[1] & 0x80)) {  // addr, func|0x80, exceptionCode, CRC lo, CRC hi
        uint16_t crc = modbusCRC16(resp, 3);
        bool crcOk = (resp[3] == (crc & 0xFF)) && (resp[4] == (crc >> 8));
        return crcOk ? ModbusResult::EXCEPTION : ModbusResult::MISMATCH;
    }

    if (n != sentLen) return ModbusResult::MISMATCH;
    for (size_t i = 0; i < sentLen; i++) {
        if (resp[i] != sentFrame[i]) return ModbusResult::MISMATCH;
    }
    return ModbusResult::OK;
}
