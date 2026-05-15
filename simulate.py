# ------------------------------------------------------------
# simulate.py — PC simulation of ESP32 MCR/VFD FSM
#
# Simulates real VFD behavior:
#   - Accel ramp from 0Hz to target frequency
#   - Decel ramp from current frequency back to 0Hz
#   - Modbus RTU frames for RUN, STOP, and frequency setpoint
#   - Simulated VFD acknowledgment (Modbus RX)
#   - RPM output derived from output frequency
#
# FSM states: IDLE → ACCEL → RUNNING → DECEL → IDLE
#
# Run this on PC to verify logic before flashing to ESP32.
# ------------------------------------------------------------

import time
import math

POLES = 4  # assumed 4-pole motor

def crc16(data):
    # Modbus error-check: 2-byte checksum appended to every frame
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return [crc & 0xFF, crc >> 8]

def build_frame(register, value):
    # FC 0x06 = Write Single Register, slave address 0x01
    frame = [0x01, 0x06, register >> 8, register & 0xFF, value >> 8, value & 0xFF]  # addr, FC, reg hi, reg lo, val hi, val lo
    return frame + crc16(frame)

def fmt(frame):
    return " ".join(f"{b:02X}" for b in frame)

def hz_to_rpm(hz):
    # Synchronous RPM with ~3% slip for a standard induction motor
    return int((120 * hz / POLES) * 0.97)

# --- GK3000 Modbus RTU frames (slave addr 0x01) ---
# Register 0x2000 = command word
# Register 0x2001 = frequency setpoint (unit: 0.01 Hz, so 6000 = 60.00 Hz)
MODBUS_RUN  = build_frame(0x2000, 0x0001)
MODBUS_STOP = build_frame(0x2000, 0x0005)

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

def run_fsm():
    state      = "IDLE"
    current_hz = 0.0
    target_hz  = 40.0   # default setpoint

    print("[BOOT] POC ready (SIMULATION MODE)")
    print(f"  Default target frequency: {target_hz} Hz")
    print("  Commands:")
    print("    '1'      = MCR ON  (start motor)")
    print("    '0'      = MCR OFF (stop motor)")
    print("    'f <hz>' = set target frequency (e.g. 'f 45')")
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
                    frame     = build_frame(0x2001, int(target_hz * 100))
                    print(f"[Modbus TX] {fmt(frame)}  (reg 0x2001 = {int(target_hz*100)} x 0.01Hz)")
                    print(f"[Modbus RX] {fmt(frame)}  <- VFD acknowledged")
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
            print(f"[Modbus TX] {fmt(MODBUS_RUN)}  (RUN command)")
            print(f"[Modbus RX] {fmt(MODBUS_RUN)}  <- VFD acknowledged")
            state      = "ACCEL"
            current_hz = ramp(0.0, target_hz, "ACCEL")
            state      = "RUNNING"
            print(f"[FSM] -> RUNNING at {current_hz} Hz  |  {hz_to_rpm(current_hz)} RPM\n")

        elif cmd == "0":
            if state != "RUNNING":
                print(f"[FSM] No transition (state={state}, cmd='{cmd}')\n")
                continue
            print("[SIM] MCR de-energized")
            print(f"[Modbus TX] {fmt(MODBUS_STOP)}  (STOP command)")
            print(f"[Modbus RX] {fmt(MODBUS_STOP)}  <- VFD acknowledged")
            state      = "DECEL"
            current_hz = ramp(current_hz, 0.0, "DECEL")
            state      = "IDLE"
            print(f"[FSM] -> IDLE\n")

        else:
            print(f"[FSM] Unknown command '{cmd}'\n")

if __name__ == "__main__":
    run_fsm()
