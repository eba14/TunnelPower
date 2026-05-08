# TunnelPower (Washington Tunneling) — MCR/VFD Communication POC

Proof of concept for ESP32 ↔ LS Electric GK3000 VFD communication over Modbus RTU (RS-485),
with MCR aux contact reading and a simple FSM to drive RUN/STOP.

## Setup
1. Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode)
2. `PlatformIO: Upload`
3. Open Terminal
4. Energize MCR using `simulate.py` script → watch Serial output + VFD response
