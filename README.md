# TunnelPower (Washington Tunneling) — MCR/VFD Communication POC

Proof of Concept (POC) for ESP32 ↔ LS Electric VFD (Variable Frequency Drive) communication over Modbus RTU (Remote Terminal Unit) via RS-485, controlling both drives on the panel:
- **VFD1** (addr 1, `LSLV0220S100-4EOFNS`) — cutter motor
- **VFD2** (addr 2, `LSLV0004S100-4EOFNS`) — water pump

The ESP32 reads an MCR (Motor Control Relay) auxiliary contact and drives both VFDs' RUN/STOP
via Modbus, with a hardwire backup signal on VFD1's X1 terminal. On start, the pump (VFD2) is
commanded RUN first, then the cutter (VFD1) after a short lead delay — matching, in software,
the panel's hardware interlock where VFD2's relay output feeds VFD1's program/start-permit pin.
On stop, the cutter is commanded STOP first (removing the load) and the pump second.

Speed is set over USB Serial at any time — `FC <hz>` for the cutter, `FP <hz>` for the pump
(e.g. `FC 45.5`) — a bring-up/test input, not the final remote-operator interface. Pump speed
control exists in case the fluids team ends up keeping a VFD for the pump instead of a
fixed-speed relay + controllable valve.

---

## Files

| File | Purpose |
|---|---|
| `src/main.cpp` | ESP32 firmware — RS-485 transport, Serial speed command; wires `Fsm` to real hardware |
| `src/config.h` | Pin assignments, Modbus addresses/registers, timing constants |
| `src/fsm.h` | The state-transition logic itself — no Arduino dependency |
| `src/modbus_protocol.h` | CRC16, frame building, response validation — no Arduino dependency |
| `src/modbus_command.h` | Retry/timeout orchestration over an `Rs485Transport` — no Arduino dependency |
| `src/rs485_transport.h` | `Rs485Transport` interface — real UART on ESP32, fake VFD in tests |
| `test/test_fsm.cpp` | Native (PC) test of `Fsm` against a fake VFD pair — see Testing section |
| `test/fake_fsm_actions.h` | Fake VFD pair used by `test_fsm.cpp` (configurable pump/cutter start success) |
| `test/test_modbus_protocol.cpp` | Native (PC) test of the protocol logic — see Testing section |
| `test/test_modbus_command.cpp` | Native (PC) test of retry/timeout logic against a fake VFD — see Testing section |
| `test/fake_rs485_transport.h` | Fake VFD used by `test_modbus_command.cpp` (healthy / dead / noisy / rejecting / intermittent) |
| `simulate.py` | PC simulator — run without hardware to verify FSM logic, dual-VFD sequencing, Modbus frames, S-curve ramp, and RPM (Revolutions Per Minute) output |
| `platformio.ini` | PlatformIO build config for the ESP32 (`esp32dev`) target |

---

## Wiring

| Signal | ESP32 Pin | Notes |
|---|---|---|
| MCR aux contact | GPIO 34 | 24V → voltage divider → ~3.1V |
| RS-485 DE/RE (Direction Enable/Receiver Enable) | GPIO 4 | MAX485 direction control |
| VFD TX (UART2 — Universal Asynchronous Receiver Transmitter) | GPIO 17 | → MAX485 DI (Data Input) |
| VFD RX (UART2) | GPIO 16 | ← MAX485 RO (Receiver Output) |
| VFD1 X1 (digital RUN, cutter only) | GPIO 26 | Hardwire backup RUN/STOP for VFD1. VFD2 (pump) has no direct ESP32 hardwire connection — it's Modbus-only, with its own relay output feeding VFD1's program/interlock pin on the panel. |
| Status LED | GPIO 2 | HIGH = cutter motor running |

### 24V → ~3.1V Voltage Divider (MCR input)
```
24V ──┬── 6.8kΩ ──┬── GPIO 34
      │           │
     GND          1kΩ
                   │
                  GND
```
6.8kΩ/1kΩ gives 24V × 1k/7.8k ≈ **3.08V** at GPIO 34 — safe under the ESP32's 3.6V absolute
maximum GPIO input rating. Confirm the physical resistors on the board match 6.8k/1k before
connecting a live 24V MCR contact.

### MAX485 Connections (shared bus — both VFDs multi-drop off one transceiver)
```
ESP32 GPIO 17 (TX) → MAX485 DI
ESP32 GPIO 16 (RX) → MAX485 RO
ESP32 GPIO 4       → MAX485 DE + RE (tied together)
MAX485 A/B         → VFD1 RS-485 terminals → VFD2 RS-485 terminals (daisy-chained)
```

---

## FSM Logic

Five states, non-blocking (timed via `millis()`, not `delay()`), with every Modbus command
verified against the VFD's echoed response (retried up to `MAX_RETRIES` times before the
command is considered failed):

| State | Trigger | Action |
|---|---|---|
| IDLE | MCR goes HIGH | Send VFD2 (pump) RUN. Confirmed → `PUMP_LEAD_WAIT`. Failed after retries → `FAULT`. |
| PUMP_LEAD_WAIT | MCR drops | Abort: send VFD2 STOP → back to `IDLE`. |
| PUMP_LEAD_WAIT | `PUMP_LEAD_MS` elapses (MCR still HIGH) | Send VFD1 (cutter) RUN + X1 HIGH. Confirmed → `RUNNING`. Failed after retries → stop VFD2, go to `FAULT`. |
| RUNNING | MCR goes LOW | Send VFD1 STOP + X1 LOW (unconditional — doesn't wait on Modbus ack) → `CUTTER_SETTLE_WAIT`. |
| RUNNING | Either VFD reports a fault (`REG_FAULT_TRIPINFO` nonzero) | Stop both VFDs → `FAULT`. Catches a fault occurring *during* operation, not just at start — the one window a simple command ACK can't see. |
| CUTTER_SETTLE_WAIT | `CUTTER_STOP_SETTLE_MS` elapses | Send VFD2 (pump) STOP → `IDLE`. |
| FAULT | — | Status LED blinks. Cleared only when MCR cycles back to LOW → `IDLE`. |

Waits are non-blocking (`millis()`, not `delay()`), so a dropped MCR is noticed within 100ms
even mid-sequence. Each Modbus transaction is still a short, bounded blocking call (a few ms;
~150ms worst case) — confirming a command landed is worth a brief stall.

---

## Modbus Frames

Register addresses are from the official LS Electric S100 manual, section 7.4 "Compatible
Common Area Parameter" (p.289) — not assumed. `0x0006` is a bitfield (only B0=Stop, B1=Forward
run matter here). `0x0005` (frequency, 0.01 Hz units) is written by `setCutterFrequency()`/
`setPumpFrequency()`, each clamped to its own bounds regardless of caller input.

Frames are built at runtime (`buildModbusFrame()`/`modbusCRC16()`), never hand-typed — a
transcription error can't silently ship the way the original register addresses once did.

| VFD | Addr | Command | Register | Value | Frame |
|---|---|---|---|---|---|
| VFD1 (cutter) | 0x01 | RUN (forward) | 0x0006 | 0x0002 | `01 06 00 06 00 02 E8 0A` |
| VFD1 (cutter) | 0x01 | STOP | 0x0006 | 0x0001 | `01 06 00 06 00 01 A8 0B` |
| VFD1 (cutter) | 0x01 | Freq setpoint example (45.00 Hz) | 0x0005 | 0x1194 | `01 06 00 05 11 94 94 34` |
| VFD2 (pump) | 0x02 | RUN (forward) | 0x0006 | 0x0002 | `02 06 00 06 00 02 E8 39` |
| VFD2 (pump) | 0x02 | STOP | 0x0006 | 0x0001 | `02 06 00 06 00 01 A8 38` |
| VFD1 (cutter) | 0x01 | Read fault status (FC 0x03) | 0x000F | — | `01 03 00 0F 00 01 B4 09` |
| VFD2 (pump) | 0x02 | Read fault status (FC 0x03) | 0x000F | — | `02 03 00 0F 00 01 B4 3A` |

The fault-status read (`0x000F`, nonzero = fault present) is polled once per tick while
`RUNNING`, catching a fault mid-operation that a RUN/STOP command's ACK alone wouldn't see. A
failed read is treated as "no fault," not a trip — a comm glitch on a status poll shouldn't
itself stop the machine.

---

## VFD Parameters
Set on each VFD's keypad before testing (codes verified against the S100 manual, sections 4.5
and 7.2):

| Group.Code | LCD Display | VFD1 (cutter) | VFD2 (pump) | Description |
|---|---|---|---|---|
| DRV-06 | Cmd Source | 3 (Int 485) | 3 (Int 485) | **Required** — without this the VFD ignores Modbus RUN/STOP entirely and listens to the keypad/terminal instead. Not the factory default. |
| Frq (Operation group) | Freq Ref Src | 6 (Int 485) | 6 (Int 485) | **Required on both** for `FC`/`FP` to have any effect — without it a VFD ignores the frequency register and keeps using its keypad/analog setpoint. Not the factory default. If the fluids team drops the pump VFD in favor of a fixed-speed relay + valve, VFD2's row here becomes moot. |
| CM.01 | Int485 St ID | 1 | 2 | Station address — must match `VFD1_ADDR`/`VFD2_ADDR` in `config.h` |
| CM.02 | Int485 Proto | 0 (Modbus RTU) | 0 (Modbus RTU) | Factory default already matches |
| CM.03 | Int485 BaudR | 3 (9600 bps) | 3 (9600 bps) | Must match `vfdSerial.begin(9600, ...)` |
| CM.04 | Int485 Mode | 0 (D8/PN/S1) | 0 (D8/PN/S1) | 8 data bits, no parity, 1 stop — matches `SERIAL_8N1` |
| CM.05 | Resp Delay | 5 ms (default) | 5 ms (default) | VFD's reply delay — well within `RESPONSE_TIMEOUT_MS` (50ms) |

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
[SIM] MCR energized
[SIM] VFD2 (pump) RUN
[Modbus TX] 02 06 00 06 00 02 E8 39  (VFD2 addr 2 RUN)
[Modbus RX] 02 06 00 06 00 02 E8 39  <- VFD2 acknowledged
  [PUMP] running at 60.0 Hz (fixed)
[Modbus TX] 01 06 00 06 00 02 E8 0A  (VFD1 addr 1 RUN)
[Modbus RX] 01 06 00 06 00 02 E8 0A  <- VFD1 acknowledged
  [ACCEL] 1.2 Hz  |  35 RPM
  [ACCEL] 4.9 Hz  |  142 RPM
  ...
  [ACCEL] 40.0 Hz  |  1164 RPM
[FSM] -> RUNNING at 40.0 Hz  |  1164 RPM
```

The ramp uses an **S-curve profile** (sine-based) — slow start, fast middle, slow end —
matching real VFD acceleration behavior to protect the motor from torque spikes.

RPM is derived from the synchronous speed formula with 3% slip:
```
RPM = (120 × Hz / poles) × 0.97
```

---

## Testing

Three native (PC) test suites run against the actual source the ESP32 build compiles — not
reimplementations. Requires a PC C++ compiler (e.g. MSYS2's `g++`, not PlatformIO's ESP32
toolchain):
```
g++ -std=c++17 -Wall -Wextra test/test_modbus_protocol.cpp -o test/test_modbus_protocol.exe
g++ -std=c++17 -Wall -Wextra test/test_modbus_command.cpp -o test/test_modbus_command.exe
g++ -std=c++17 -Wall -Wextra test/test_fsm.cpp -o test/test_fsm.exe
./test/test_modbus_protocol.exe
./test/test_modbus_command.exe
./test/test_fsm.exe
```

**`test_modbus_protocol.cpp`** — CRC correctness, response validation (echo, timeout,
corruption, exceptions — with and without a bad CRC), frame contents for RUN/STOP/frequency/
fault-read, and the frequency clamp at both bounds.

**`test_modbus_command.cpp`** — retry/timeout orchestration against `FakeRs485Transport`
(healthy / dead / noisy / rejecting / intermittent), covering both writes and reads. Caught a
real bug: the retry loop used to retry on Modbus exceptions too, which can't ever succeed on
retry (the VFD already explicitly rejected the frame) — now fails fast on `EXCEPTION`, only
retries `TIMEOUT`/`MISMATCH`.

**`test_fsm.cpp`** — every `Fsm` state transition against `FakeFsmActions`: full start/stop
cycle, mid-sequence abort, `FAULT` entry on either VFD failing to start, a fault detected
mid-`RUNNING`, `FAULT` clearing only on an MCR cycle, and exact timing thresholds. Previously
the FSM had zero automated coverage — only a manual trace and one physical-pin-touch test.

**Not tested by any of this:** real UART timing or RS-485 electrical behavior —
`Esp32Rs485Transport` is untested by these harnesses, and `simulate.py` always takes the
happy path. Only a real VFD can confirm those.

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

To set speed, type into the Serial Monitor's send line (Enter-terminated) at any time:
```
FC 45.5
FP 30.0
```
`FC` targets the cutter (VFD1), `FP` targets the pump (VFD2). Each requires that VFD's `Frq`
keypad parameter set to `6` (Int 485) — see VFD Parameters above — or the write succeeds on
the bus but the VFD ignores the value.

---

## Hardware Required (full test)

| Component | Purpose |
|---|---|
| ESP32 DevKit V1 | Runs the firmware |
| Micro-USB cable | Flash firmware + power the ESP32 |
| MAX485 module | UART ↔ RS-485, shared bus for both VFDs |
| Female-Female + Female-Male jumper wires | ESP32↔MAX485, MAX485↔VFD `S+`/`S-`/`SG` terminals |
| Small (M2-size) precision screwdriver | VFD control terminal screws are M2 — a standard driver may slip |
| VFD1: LS Electric `LSLV0220S100-4EOFNS` (IP20) | Cutter motor drive |
| VFD2: LS Electric `LSLV0004S100-4EOFNS` (IP20) | Pump drive |
| Circuit breakers (`ABS103c` 100A, `ABS33c` 5A) | Branch protection, one per VFD |
| **Real 480V 3-phase power source** (generator or equivalent) | **Hard dependency** — VFDs have no control power without it; nothing above this line does anything until it's connected |
| 3-phase induction motor(s) | Only needed to actually spin something — not required to test Modbus communication itself |
| MCR relay + 6.8kΩ/1kΩ divider resistors | Real MCR signal input — not needed for the communication test, only the full auto-sequencing panel |
