# ------------------------------------------------------------
# simulate.py — PC (Personal Computer) simulation of ESP32 MCR (Motor Control Relay)/VFD (Variable Frequency Drive) FSM (Finite State Machine)
#
# Simulates real VFD behavior for BOTH drives on the shared RS-485 bus:
#   - VFD1 (addr 1): cutter motor — accel/decel ramp from 0Hz to target frequency, S-curve shaped
#   - VFD2 (addr 2): pump — fixed-speed RUN/STOP, started before the cutter and stopped after it
#   - Modbus RTU (Remote Terminal Unit) frames for RUN, STOP, and frequency setpoint (CRC computed, not hardcoded)
#   - Simulated VFD acknowledgment (Modbus RX (Receive))
#   - RPM (Revolutions Per Minute) output derived from output frequency
#
# Start order: pump RUN, then (after a lead delay) cutter RUN — mirrors the panel's hardware
# interlock (VFD2 relay-out -> VFD1 program pin) as a software-side check of the same logic.
# Stop order is reversed: cutter STOP first (removes the load), then pump STOP.
#
# FSM states: IDLE → ACCEL → RUNNING → DECEL → IDLE
#
# Run this on PC to verify logic before flashing to ESP32.
# ------------------------------------------------------------

import time
import math

POLES = 4  # assumed 4-pole motor

VFD1_ADDR = 0x01  # cutter motor VFD (LSLV0220S100-4EXNNS)
VFD2_ADDR = 0x02  # pump VFD (LSLV0004S100-4EOFNS)

PUMP_LEAD_S          = 0.5  # matches PUMP_LEAD_MS in src/config.h
CUTTER_STOP_SETTLE_S = 0.5  # matches CUTTER_STOP_SETTLE_MS in src/config.h
PUMP_HZ              = 60.0  # pump VFD runs at a fixed setpoint, no ramp modeled here

def crc16(data):
    # Modbus error-check: 2-byte checksum appended to every frame
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return [crc & 0xFF, crc >> 8]

def build_frame(addr, register, value):
    # FC 0x06 = Write Single Register
    frame = [addr, 0x06, register >> 8, register & 0xFF, value >> 8, value & 0xFF]  # addr, FC, reg hi, reg lo, val hi, val lo
    return frame + crc16(frame)

def fmt(frame):
    return " ".join(f"{b:02X}" for b in frame)

def hz_to_rpm(hz):
    # Synchronous RPM with ~3% slip for a standard induction motor
    return int((120 * hz / POLES) * 0.97)

# S100-series common-area registers (manual sec. 7.4): 0x0006 = operation command (bitfield,
# B0=Stop, B1=Forward run), 0x0005 = command frequency (0.01 Hz units).
REG_OPERATION_COMMAND = 0x0006
REG_COMMAND_FREQUENCY = 0x0005
CMD_STOP        = 0x0001
CMD_RUN_FORWARD = 0x0002

VFD1_RUN  = build_frame(VFD1_ADDR, REG_OPERATION_COMMAND, CMD_RUN_FORWARD)
VFD1_STOP = build_frame(VFD1_ADDR, REG_OPERATION_COMMAND, CMD_STOP)
VFD2_RUN  = build_frame(VFD2_ADDR, REG_OPERATION_COMMAND, CMD_RUN_FORWARD)
VFD2_STOP = build_frame(VFD2_ADDR, REG_OPERATION_COMMAND, CMD_STOP)

# VFD limits
MAX_HZ     = 60.0
MIN_HZ     = 0.0
RAMP_STEP  = 1.0   # Hz per ramp tick
RAMP_DELAY = 0.15  # seconds per tick (simulates accel/decel time)

def s_curve(progress):
    # S-curve profile using sine: slow start, fast middle, slow end — mirrors real VFD behavior
    return (math.sin(math.pi * progress - math.pi / 2) + 1) / 2  # maps 0→1 input to 0→1 output on an S-shaped curve

def ramp(current_hz, target_hz, state_label):
    # S-curve ramp: non-linear acceleration/deceleration matching real VFD torque profiles
    steps = max(1, int(abs(target_hz - current_hz) / RAMP_STEP))
    for i in range(1, steps + 1):
        hz = current_hz + (target_hz - current_hz) * s_curve(i / steps)
        hz = round(max(MIN_HZ, min(MAX_HZ, hz)), 1)
        print(f"  [{state_label}] {hz:.1f} Hz  |  {hz_to_rpm(hz)} RPM")
        time.sleep(RAMP_DELAY)
    return target_hz

def start_pump():
    print("[SIM] VFD2 (pump) RUN")
    print(f"[Modbus TX] {fmt(VFD2_RUN)}  (VFD2 addr {VFD2_ADDR} RUN)")
    print(f"[Modbus RX] {fmt(VFD2_RUN)}  <- VFD2 acknowledged")
    print(f"  [PUMP] running at {PUMP_HZ:.1f} Hz (fixed)")
    time.sleep(PUMP_LEAD_S)

def stop_pump():
    time.sleep(CUTTER_STOP_SETTLE_S)
    print("[SIM] VFD2 (pump) STOP")
    print(f"[Modbus TX] {fmt(VFD2_STOP)}  (VFD2 addr {VFD2_ADDR} STOP)")
    print(f"[Modbus RX] {fmt(VFD2_STOP)}  <- VFD2 acknowledged")

def run_fsm():
    state      = "IDLE"
    current_hz = 0.0
    target_hz  = 40.0   # default setpoint

    print("[BOOT] POC (Proof of Concept) ready (SIMULATION MODE)")
    print(f"  Default cutter target frequency: {target_hz} Hz")
    print("  Commands:")
    print("    '1'      = MCR ON  (start pump, then cutter)")
    print("    '0'      = MCR OFF (stop cutter, then pump)")
    print("    'f <hz>' = set cutter target frequency (e.g. 'f 45')")
    print("    'q'      = quit\n")

    while True:
        cmd = input("Command: ").strip()

        if cmd == "q":
            break

        elif cmd.startswith("f "):
            try:
                new_hz = float(cmd.split()[1])
                if new_hz < MIN_HZ or new_hz > MAX_HZ:
                    print(f"[ERR] Frequency must be between {MIN_HZ} and {MAX_HZ} Hz\n")
                else:
                    target_hz = new_hz
                    frame     = build_frame(VFD1_ADDR, REG_COMMAND_FREQUENCY, int(target_hz * 100))
                    print(f"[Modbus TX] {fmt(frame)}  (VFD1 reg 0x0005 = {int(target_hz*100)} x 0.01Hz)")
                    print(f"[Modbus RX] {fmt(frame)}  <- VFD1 acknowledged")
                    if state == "RUNNING":
                        direction = "ACCEL" if target_hz > current_hz else "DECEL"
                        state = direction
                        current_hz = ramp(current_hz, target_hz, direction)
                        state = "RUNNING"
                        print(f"[FSM] -> RUNNING at {current_hz} Hz  |  {hz_to_rpm(current_hz)} RPM\n")
            except (IndexError, ValueError):
                print("[ERR] Usage: f <hz>  (e.g. f 45)\n")

        elif cmd == "1":
            if state != "IDLE":
                print(f"[FSM] No transition (state={state}, cmd='{cmd}')\n")
                continue
            print("[SIM] MCR energized")
            start_pump()
            print(f"[Modbus TX] {fmt(VFD1_RUN)}  (VFD1 addr {VFD1_ADDR} RUN)")
            print(f"[Modbus RX] {fmt(VFD1_RUN)}  <- VFD1 acknowledged")
            state      = "ACCEL"
            current_hz = ramp(0.0, target_hz, "ACCEL")
            state      = "RUNNING"
            print(f"[FSM] -> RUNNING at {current_hz} Hz  |  {hz_to_rpm(current_hz)} RPM\n")

        elif cmd == "0":
            if state != "RUNNING":
                print(f"[FSM] No transition (state={state}, cmd='{cmd}')\n")
                continue
            print("[SIM] MCR de-energized")
            print(f"[Modbus TX] {fmt(VFD1_STOP)}  (VFD1 addr {VFD1_ADDR} STOP)")
            print(f"[Modbus RX] {fmt(VFD1_STOP)}  <- VFD1 acknowledged")
            state      = "DECEL"
            current_hz = ramp(current_hz, 0.0, "DECEL")
            state      = "IDLE"
            stop_pump()
            print(f"[FSM] -> IDLE\n")

        else:
            print(f"[FSM] Unknown command '{cmd}'\n")

if __name__ == "__main__":
    run_fsm()
