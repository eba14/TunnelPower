#pragma once
#include <cstdint>
#include <cstddef>

// Abstracts the RS-485 link so Modbus command/retry logic (modbus_command.h) can run
// against a real UART on the ESP32 or a fake VFD in a PC test, unchanged either way.
class Rs485Transport {
public:
    virtual ~Rs485Transport() = default;
    virtual void transmit(const uint8_t *frame, size_t len) = 0;
    virtual size_t receive(uint8_t *buf, size_t bufLen, unsigned long timeoutMs) = 0;
};
