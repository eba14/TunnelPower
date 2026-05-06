# Simulate ESP32 FSM for LS Electric GK3000 VFD (currently without the hardware)

def crc16(data):
    # Modbus error-check: 2-byte checksum appended to every frame
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return [crc & 0xFF, crc >> 8]  # low byte first, then high byte

def build_frame(register, value):
    # FC 0x06 = Write Single Register, slave address 0x01 (GK3000 default)
    frame = [0x01, 0x06, register >> 8, register & 0xFF, value >> 8, value & 0xFF]
    return frame + crc16(frame)

RUN_FRAME  = build_frame(0x2000, 0x0001)  # 0x2000 = GK3000 command register, 0x0001 = RUN
STOP_FRAME = build_frame(0x2000, 0x0005)  # 0x0005 = STOP

def fmt(frame):
    return " ".join(f"{b:02X}" for b in frame)

def run_fsm():
    state = "IDLE"
    print("[BOOT] Simulation ready — '1'=MCR ON, '0'=MCR OFF, 'q'=quit\n")
    while True:
        cmd = input("Command: ").strip()
        if cmd == "q":
            break
        elif cmd == "1" and state == "IDLE":
            state = "RUNNING"
            print(f"[Modbus TX] {fmt(RUN_FRAME)}")
            print("[FSM] -> RUNNING\n")
        elif cmd == "0" and state == "RUNNING":
            state = "IDLE"
            print(f"[Modbus TX] {fmt(STOP_FRAME)}")
            print("[FSM] -> IDLE\n")
        else:
            print(f"[FSM] No transition (state={state})\n")

if __name__ == "__main__":
    run_fsm()
