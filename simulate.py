# Simulate ESP32 FSM for LS Electric GK3000 VFD (currently without the hardware)
import time

POLES = 4  # assumed 4-pole motor

def crc16(data):
    # Modbus error-check: 2-byte checksum appended to every frame
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return [crc & 0xFF, crc >> 8]

def build_write(register, value):
    # FC 0x06 = Write Single Register, slave address 0x01
    frame = [0x01, 0x06, register >> 8, register & 0xFF, value >> 8, value & 0xFF]
    return frame + crc16(frame)

def build_read(register, count=1):
    # FC 0x03 = Read Holding Registers
    frame = [0x01, 0x03, register >> 8, register & 0xFF, 0x00, count]
    return frame + crc16(frame)

def build_read_response(value):
    # GK3000 read response format: slave + FC + byte_count + data + CRC
    frame = [0x01, 0x03, 0x02, value >> 8, value & 0xFF]
    return frame + crc16(frame)

def fmt(frame):
    return " ".join(f"{b:02X}" for b in frame)

def hz_to_rpm(hz):
    # Synchronous RPM with ~3% slip for a standard induction motor
    return int((120 * hz / POLES) * 0.97)

CMD_REGISTER  = 0x2000  # GK3000 command register
FREQ_REGISTER = 0x2001  # GK3000 frequency setpoint (units: 0.01 Hz)
OUT_FREQ_REG  = 0x2100  # GK3000 output frequency readback register
CMD_RUN       = 0x0001
CMD_STOP      = 0x0005
ACCEL_RATE    = 5       # Hz per ramp step

def simulate_ramp(current_hz, target_hz):
    # Simulate VFD accelerating or decelerating to target
    step = ACCEL_RATE if target_hz > current_hz else -ACCEL_RATE
    hz = current_hz
    while (step > 0 and hz < target_hz) or (step < 0 and hz > target_hz):
        hz = min(hz + step, target_hz) if step > 0 else max(hz + step, target_hz)
        print(f"  [VFD] {hz:.1f} Hz  |  {hz_to_rpm(hz)} RPM")
        time.sleep(0.3)
    return target_hz

def run_fsm():
    state      = "IDLE"
    target_hz  = 50.0
    current_hz = 0.0

    print("[BOOT] Simulation ready")
    print("  '1'=RUN  '0'=STOP  'f <hz>'=set frequency  'r'=read VFD status  'q'=quit\n")

    while True:
        cmd = input("Command: ").strip()

        if cmd == "q":
            break

        elif cmd == "r":
            # Read current output frequency from VFD
            req  = build_read(OUT_FREQ_REG)
            resp = build_read_response(int(current_hz * 100))
            print(f"[Modbus TX] {fmt(req)}   <- read output frequency")
            print(f"[Modbus RX] {fmt(resp)}  <- VFD response: {current_hz:.1f} Hz  |  {hz_to_rpm(current_hz)} RPM\n")

        elif cmd.startswith("f "):
            try:
                target_hz = float(cmd[2:])
                frame = build_write(FREQ_REGISTER, int(target_hz * 100))
                print(f"[Modbus TX] {fmt(frame)}  <- frequency setpoint: {target_hz} Hz")
                print(f"[Modbus RX] {fmt(frame)}  <- VFD acknowledged")  # FC 0x06: VFD echoes same frame
                if state == "RUNNING":
                    current_hz = simulate_ramp(current_hz, target_hz)
                print()
            except ValueError:
                print("[ERR] Usage: f <hz>  e.g. f 45\n")

        elif cmd == "1" and state == "IDLE":
            state = "RUNNING"
            frame = build_write(CMD_REGISTER, CMD_RUN)
            print(f"[Modbus TX] {fmt(frame)}  <- RUN command")
            print(f"[Modbus RX] {fmt(frame)}  <- VFD acknowledged")
            print(f"[FSM] -> RUNNING at {target_hz} Hz")
            current_hz = simulate_ramp(0.0, target_hz)
            print()

        elif cmd == "0" and state == "RUNNING":
            state = "IDLE"
            frame = build_write(CMD_REGISTER, CMD_STOP)
            print(f"[Modbus TX] {fmt(frame)}  <- STOP command")
            print(f"[Modbus RX] {fmt(frame)}  <- VFD acknowledged")
            print("[FSM] -> IDLE (ramping down)")
            current_hz = simulate_ramp(current_hz, 0.0)
            print()

        else:
            print(f"[FSM] No transition (state={state})\n")

if __name__ == "__main__":
    run_fsm()
