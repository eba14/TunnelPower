// Native (PC) test of sendModbusCommand's retry/timeout orchestration against a fake VFD
// that simulates realistic bus/drive fault conditions -- no real hardware involved.
// Build: g++ -std=c++17 -I src test/test_modbus_command.cpp -o test_modbus_command

#include "../src/config.h"
#include "../src/modbus_command.h"
#include "fake_rs485_transport.h"
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
    uint8_t frame[FRAME_LEN];
    buildModbusFrame(frame, VFD1_ADDR, MODBUS_FC_WRITE_SINGLE, REG_OPERATION_COMMAND, CMD_RUN_FORWARD);

    // Healthy VFD: succeeds on the first attempt, no retries burned.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::ALWAYS_ECHO;
        auto outcome = sendModbusCommand(t, frame, FRAME_LEN, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::OK, "healthy VFD succeeds");
        check(outcome.attempts == 1, "healthy VFD takes exactly 1 attempt");
        check(t.transmitCount == 1, "healthy VFD is transmitted to exactly once");
    }

    // Dead VFD: exhausts every retry, never succeeds.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::NEVER_RESPOND;
        auto outcome = sendModbusCommand(t, frame, FRAME_LEN, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::TIMEOUT, "dead VFD reports TIMEOUT");
        check(outcome.attempts == MAX_RETRIES + 1, "dead VFD burns all retries");
        check(t.transmitCount == MAX_RETRIES + 1, "dead VFD is transmitted to on every attempt");
    }

    // Noisy line: every reply is corrupted, never recovers.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::ALWAYS_CORRUPT;
        auto outcome = sendModbusCommand(t, frame, FRAME_LEN, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::MISMATCH, "corrupted replies report MISMATCH");
        check(outcome.attempts == MAX_RETRIES + 1, "corrupted replies burn all retries");
    }

    // Rejected register: VFD understands the frame and explicitly refuses it. Must NOT
    // waste retries on something retrying can't fix.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::ALWAYS_EXCEPTION;
        auto outcome = sendModbusCommand(t, frame, FRAME_LEN, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::EXCEPTION, "rejected register reports EXCEPTION");
        check(outcome.attempts == 1, "rejected register fails fast -- no retries wasted");
        check(t.transmitCount == 1, "rejected register is transmitted to exactly once");
    }

    // Transient glitch that clears within the retry budget: must recover.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::ECHO_AFTER_N_FAILURES;
        t.failuresBeforeSuccess = 1;  // 1st attempt silently fails, 2nd succeeds
        auto outcome = sendModbusCommand(t, frame, FRAME_LEN, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::OK, "transient glitch within budget recovers");
        check(outcome.attempts == 2, "transient glitch succeeds on the 2nd attempt");
    }

    // Transient glitch that outlasts the retry budget: must still fail cleanly.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::ECHO_AFTER_N_FAILURES;
        t.failuresBeforeSuccess = MAX_RETRIES + 5;  // never clears within the budget
        auto outcome = sendModbusCommand(t, frame, FRAME_LEN, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::TIMEOUT, "glitch outlasting the budget reports TIMEOUT");
        check(outcome.attempts == MAX_RETRIES + 1, "glitch outlasting the budget burns all retries");
    }

    // readModbusRegister: healthy VFD returns the register's actual value.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::REPLY_READ_VALUE;
        t.readReplyValue = 0x0000;
        auto outcome = readModbusRegister(t, VFD1_ADDR, REG_FAULT_TRIPINFO, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::OK, "fault read succeeds against a healthy VFD");
        check(outcome.value == 0x0000, "no-fault value (0x0000) read back correctly");
    }

    // readModbusRegister: a real fault code comes back intact.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::REPLY_READ_VALUE;
        t.readReplyValue = 0x0007;
        auto outcome = readModbusRegister(t, VFD2_ADDR, REG_FAULT_TRIPINFO, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::OK && outcome.value == 0x0007,
              "nonzero fault code read back correctly");
    }

    // readModbusRegister: dead VFD still exhausts retries and reports TIMEOUT, same as writes.
    {
        FakeRs485Transport t;
        t.mode = FakeMode::NEVER_RESPOND;
        auto outcome = readModbusRegister(t, VFD1_ADDR, REG_FAULT_TRIPINFO, MAX_RETRIES, RESPONSE_TIMEOUT_MS);
        check(outcome.result == ModbusResult::TIMEOUT, "dead VFD read reports TIMEOUT");
        check(outcome.attempts == MAX_RETRIES + 1, "dead VFD read burns all retries");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
