#pragma once
#include "../src/rs485_transport.h"
#include "../src/modbus_protocol.h"
#include <vector>
#include <cstring>
#include <algorithm>

enum class FakeMode { ALWAYS_ECHO, NEVER_RESPOND, ALWAYS_CORRUPT, ALWAYS_EXCEPTION, ECHO_AFTER_N_FAILURES };

// A fake VFD for testing sendModbusCommand's retry logic without real hardware. transmit()
// just records the frame; receive() replies according to `mode`, simulating a healthy VFD,
// a dead one, line noise, a rejected register, or a transient glitch that later recovers.
class FakeRs485Transport : public Rs485Transport {
public:
    FakeMode mode = FakeMode::ALWAYS_ECHO;
    uint8_t failuresBeforeSuccess = 0;  // ECHO_AFTER_N_FAILURES only
    int transmitCount = 0;

    void transmit(const uint8_t *frame, size_t len) override {
        transmitCount++;
        lastFrame.assign(frame, frame + len);
    }

    size_t receive(uint8_t *buf, size_t bufLen, unsigned long /*timeoutMs*/) override {
        if (mode == FakeMode::NEVER_RESPOND) return 0;
        if (mode == FakeMode::ECHO_AFTER_N_FAILURES && transmitCount <= failuresBeforeSuccess) return 0;

        if (mode == FakeMode::ALWAYS_CORRUPT) {
            size_t n = echoLastFrame(buf, bufLen);
            if (n > 0) buf[0] ^= 0xFF;
            return n;
        }

        if (mode == FakeMode::ALWAYS_EXCEPTION) {
            uint8_t exc[5] = {lastFrame[0], uint8_t(0x80 | lastFrame[1]), 0x02, 0, 0};  // 0x02 = Illegal Data Address
            uint16_t crc = modbusCRC16(exc, 3);
            exc[3] = crc & 0xFF;
            exc[4] = crc >> 8;
            size_t n = std::min(sizeof(exc), bufLen);
            std::memcpy(buf, exc, n);
            return n;
        }

        return echoLastFrame(buf, bufLen);  // ALWAYS_ECHO, or past-the-failure-count ECHO_AFTER_N_FAILURES
    }

private:
    std::vector<uint8_t> lastFrame;

    size_t echoLastFrame(uint8_t *buf, size_t bufLen) {
        size_t n = std::min(lastFrame.size(), bufLen);
        std::memcpy(buf, lastFrame.data(), n);
        return n;
    }
};
