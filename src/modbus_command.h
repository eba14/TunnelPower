#pragma once
#include "modbus_protocol.h"
#include "rs485_transport.h"

struct ModbusCommandOutcome {
    ModbusResult result;
    uint8_t attempts;  // how many transmit attempts were actually made
};

// Retries on TIMEOUT/MISMATCH (plausible transient bus issues). Stops immediately on
// EXCEPTION -- the VFD understood the request and explicitly rejected it (e.g. bad
// register address), so retrying the identical frame can't produce a different result.
inline ModbusCommandOutcome sendModbusCommand(Rs485Transport &transport, const uint8_t *frame, size_t len,
                                               uint8_t maxRetries, unsigned long responseTimeoutMs) {
    ModbusResult result = ModbusResult::TIMEOUT;
    uint8_t attemptsUsed = 0;

    for (uint8_t attempt = 1; attempt <= maxRetries + 1; attempt++) {
        attemptsUsed = attempt;
        transport.transmit(frame, len);

        uint8_t resp[16];  // largest expected reply (8-byte echo) fits with room to spare
        size_t n = transport.receive(resp, sizeof(resp), responseTimeoutMs);
        result = evaluateResponse(frame, len, resp, n);

        if (result == ModbusResult::OK || result == ModbusResult::EXCEPTION) break;
    }

    return ModbusCommandOutcome{result, attemptsUsed};
}
