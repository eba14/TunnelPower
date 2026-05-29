# TunnelPower (Washington Tunneling) — MCR/VFD Communication POC

Proof of Concept (POC) for ESP32 ↔ LS Electric GK3000 VFD (Variable Frequency Drive) communication over Modbus RTU (Remote Terminal Unit) via RS-485.
The ESP32 reads an MCR (Motor Control Relay) auxiliary contact and drives the VFD RUN/STOP
via Modbus, with a hardwire backup signal on the VFD X1 terminal.

---

## Files

| File | Purpose |
|---|---|
| `src/main.cpp` | ESP32 firmware — reads MCR input, sends Modbus RUN/STOP to GK3000 over RS-485 |
| `simulate.py` | PC simulator — run without hardware to verify FSM (Finite State Machine) logic, Modbus frames, S-curve ramp, and RPM (Revolutions Per Minute) output |
| `platformio.ini` | PlatformIO build config for ESP32 (`esp32dev`) and native PC (`native`) targets |

---

## Wiring

| Signal | ESP32 Pin | Notes |
|---|---|---|
| MCR aux contact | GPIO 34 | 24V → voltage divider → 3.3V |
| RS-485 DE/RE (Direction Enable/Receiver Enable) | GPIO 4 | MAX485 direction control |
| VFD TX (UART2 — Universal Asynchronous Receiver Transmitter) | GPIO 17 | → MAX485 DI (Data Input) |
| VFD RX (UART2) | GPIO 16 | ← MAX485 RO (Receiver Output) |
| VFD X1 (digital RUN) | GPIO 26 | Hardwire backup RUN/STOP |
| Status LED | GPIO 2 | HIGH = running |

### 24V → 3.3V Voltage Divider (MCR input)
```
24V ──┬── 6.8kΩ ──┬── GPIO 34
      │           │
     GND        3.3kΩ
                  │
                 GND
```

### MAX485 Connections
```
ESP32 GPIO 17 (TX) → MAX485 DI
ESP32 GPIO 16 (RX) → MAX485 RO
ESP32 GPIO 4       → MAX485 DE + RE (tied together)
MAX485 A/B         → VFD RS-485 terminals
```

---

## FSM Logic

| State | MCR Input | Action |
|---|---|---|
| IDLE | Goes HIGH | → RUNNING: Modbus RUN + X1 HIGH |
| RUNNING | Goes LOW | → IDLE: Modbus STOP + X1 LOW |

---

## Modbus Frames (GK3000 device address 0x01)

| Command | Register | Value | Frame |
|---|---|---|---|
| RUN | 0x2000 | 0x0001 | `01 06 20 00 00 01 43 CA` |
| STOP | 0x2000 | 0x0005 | `01 06 20 00 00 05 42 09` |
| Freq setpoint | 0x2001 | 0.01 Hz units | computed via CRC16 (Cyclic Redundancy Check) |

---

## GK3000 VFD Parameters
Set these on the VFD keypad before testing:

| Parameter | Value | Description |
|---|---|---|
| P0.01 | 2 | Command source = RS-485 |
| P0.03 | 2 | Frequency source = RS-485 |
| P14.00 | 1 | Device address = 1 |
| P14.01 | 9600 | Baud rate — must match `vfdSerial.begin()` |
| P14.02 | 0 | 8N1 format (8 data bits, No parity, 1 stop bit) |

---

## Simulator (`simulate.py`)

Runs entirely on PC — no ESP32, VFD, or wiring needed.
Demonstrates FSM logic, Modbus frames, S-curve speed ramp acceleration/deceleration, and motor RPM.

```
python simulate.py
```

| Command | Action |
|---|---|
| `1` | Simulate MCR energizing → ACCEL ramp → RUNNING |
| `0` | Simulate MCR dropping → DECEL ramp → IDLE |
| `f <hz>` | Set target frequency (e.g. `f 45`) — range 0–60 Hz |
| `q` | Quit |

**Example output:**
```
Command: 1
[Modbus TX] 01 06 20 00 00 01 43 CA  (RUN command)
[Modbus RX] 01 06 20 00 00 01 43 CA  <- VFD acknowledged
[FSM] -> RUNNING at 40.0 Hz
  [ACCEL] 1.2 Hz  |  35 RPM
  [ACCEL] 4.9 Hz  |  142 RPM
  ...
  [ACCEL] 40.0 Hz  |  1164 RPM
```

The ramp uses an **S-curve profile** (sine-based) — slow start, fast middle, slow end —
matching real VFD acceleration behavior to protect the motor from torque spikes.

RPM is derived from the synchronous speed formula with 3% slip:
```
RPM = (120 × Hz / poles) × 0.97
```

---

## Flashing the ESP32

1. Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode)
2. Install CP2102 USB driver (Silicon Labs)
3. Plug in ESP32 via micro-USB
4. Click **→ Upload** in PlatformIO toolbar
5. Click **Serial Monitor** — set baud to **115200**
6. Press reset button on ESP32

Expected boot output:
```
[BOOT] POC ready — waiting for MCR signal
```

---

## Hardware Required (full test)

| Component | Purpose |
|---|---|
| ESP32 DevKit V1 | MCU (Microcontroller Unit) — runs the firmware |
| Micro-USB cable | Flash firmware from PC |
| MAX485 module | Converts UART to RS-485 for VFD communication |
| GK3000 VFD (1hp, 3-phase) | Variable frequency drive |
| 3-phase induction motor | Load |
| MCR relay | Input signal source |
| 6.8kΩ + 3.3kΩ resistors | Voltage divider for MCR input |
