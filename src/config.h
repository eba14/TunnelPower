#pragma once

// --- Pin assignments ---
#define MCR_INPUT_PIN   34  // MCR aux contact, 24V -> 6.8k/1k divider -> ~3.1V (ESP32 abs max is 3.6V)
#define RS485_DE_PIN     4  // MAX485 DE+RE tied together (HIGH=transmit, LOW=receive)
#define VFD_TX_PIN      17  // UART2 TX -> MAX485 DI, shared bus (VFD1 + VFD2 multi-drop)
#define VFD_RX_PIN      16  // UART2 RX <- MAX485 RO
#define VFD1_X1_PIN     26  // VFD1 (cutter) X1 terminal, hardwire RUN/STOP backup
#define STATUS_LED_PIN   2  // HIGH = cutter running, blinking = FAULT

// --- Modbus RTU station addresses (set via each VFD's keypad, CM.01 "Int485 St ID") ---
#define VFD1_ADDR 0x01  // Cutter (LSLV0220S100-4EOFNS)
#define VFD2_ADDR 0x02  // Pump (LSLV0004S100-4EOFNS)

// --- S100-series common-area registers (manual sec. 7.4, p.289) ---
#define REG_OPERATION_COMMAND 0x0006  // bitfield; see CMD_* below for the writable low bits
#define REG_COMMAND_FREQUENCY 0x0005  // 0.01 Hz units; written by setCutterFrequency()/setPumpFrequency()
#define REG_FAULT_TRIPINFO    0x000F  // nonzero = a fault is present on that VFD right now

#define CMD_STOP           0x0001  // B0
#define CMD_RUN_FORWARD    0x0002  // B1
#define CMD_RUN_REVERSE    0x0004  // B2 (unused)
#define CMD_TRIP_RESET     0x0008  // B3 (unused)
#define CMD_EMERGENCY_STOP 0x0010  // B4 (unused)
// B5-B14 report command/frequency source (keypad/terminal/485) and are read-only on this
// register -- source is set once via keypad, not re-written per command.

// Frequency setpoint bounds, in 0.01 Hz units (register 0x0005 scale), one pair per VFD.
// Each requires that VFD's "Frq" (Frequency reference source, Operation group) keypad
// parameter set to 6 (Int 485) -- otherwise it ignores this register regardless of what's
// written. Pump frequency control is here in case the fluids team ends up keeping a VFD
// for the pump instead of a fixed-speed relay + valve.
#define CUTTER_MIN_CENTIHERTZ 0     // 0.00 Hz
#define CUTTER_MAX_CENTIHERTZ 6000  // 60.00 Hz
#define PUMP_MIN_CENTIHERTZ   0     // 0.00 Hz
#define PUMP_MAX_CENTIHERTZ   6000  // 60.00 Hz

#define FRAME_LEN 8  // 6 data bytes + 2 CRC bytes

// --- Sequencing timing (non-blocking, tracked via millis()) ---
#define PUMP_LEAD_MS          500
#define CUTTER_STOP_SETTLE_MS 500

// --- Modbus transaction timing (tune once real VFDs are on the bench) ---
#define RESPONSE_TIMEOUT_MS   50
#define INTERBYTE_SILENCE_MS   4  // ~3.5 char times at 9600 baud = end of reply frame
#define MAX_RETRIES            2
