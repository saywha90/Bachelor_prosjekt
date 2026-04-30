# Hardware

> **Important:** The OpenRB-150 is flashed using the Arduino IDE, but the project does **not** use a separate Arduino microcontroller.

## Bill of Materials

| # | Component | Model / Spec | Qty | Role |
|---|-----------|-------------|-----|------|
| 1 | Host computer | Raspberry Pi 5 (8 GB) | 1 | Runs Python pipeline (`main.py`), vision, IK |
| 2 | Camera | Luxonis OAK-D S2 (wrist-mounted) | 1 | RGB + depth, mounted on arm wrist, USB-C |
| 3 | Motor controller | ROBOTIS OpenRB-150 | 1 | Drives Dynamixel bus, USB-C serial to Pi |
| 4 | Servo – Base (ID 1) | Dynamixel XM430-W350 | 1 | Joint 1 rotation |
| 5 | Servo – Shoulder (ID 2) | Dynamixel XM540-W270 | 1 | Joint 2 lift |
| 6 | Servo – Elbow (ID 3) | Dynamixel XM430-W350 | 1 | Joint 3 reach |
| 7 | Servo – Wrist (ID 4) | Dynamixel XL430-W250 | 1 | Joint 4 tilt |
| 8 | Servo – Claw (ID 5) | Dynamixel XM430-W350 | 1 | Gripper open/close |
| 9 | Power supply | Dual-output PSU (12 V + 5 V) | 1 | Powers all components |
| 10 | Fuse | 15 A blade fuse + inline holder | 1 | Protects 12 V rail |
| 11 | USB-C cable (data) | USB-C to USB-C | 2 | Pi ↔ OpenRB-150, Pi ↔ OAK-D S2 |
| 12 | USB-C cable (power) | USB-C power cable | 1 | PSU 5 V → Pi |
| 13 | TTL servo cable | 3-pin JST (Dynamixel) | 5 | Daisy-chain between servos |

### Camera mounting

The OAK-D S2 is mounted on the arm wrist, attached to the
3D-printed bracket above the claw assembly. The camera optical
axis is angled approximately 30° below the forearm axis so that
when the arm is at SCAN_POSE, the camera looks down at the
workspace from a height of ~30–40 cm.

This mounting choice was made because:
- It allows the camera to follow the arm during approach (future
  visual-servoing capability)
- It avoids the need for a separate camera pillar
- It keeps all sensing/actuation in a single moveable unit

Note that this mounting requires a calibrated SCAN_POSE; see
docs/calibration.md → Step 02c.

## Wiring Diagram

### Circuit Schematic (ASCII)

```
                    ┌─────────────────────────┐
                    │    Dual-output PSU       │
                    │                          │
                    │  Output 1: 12 V / 10 A   │
                    │  Output 2:  5 V /  3 A   │
                    │  GND (felles minus)       │
                    └──┬──────────┬──────┬─────┘
                       │          │      │
                  12 V │     5 V  │      │ GND
                       │          │      │
              ┌────────┘     ┌────┘      └──── common GND bus
              │              │                  (felles minus)
              ▼              ▼
     ┌──── 15 A ────┐  ┌──────────┐
     │   sikring     │  │          │
     └──────┬────────┘  │          │
            │           │          ▼
            ▼           │   ┌──────────────┐
     ┌──────────────┐   │   │ Raspberry    │
     │ OpenRB-150   │   │   │ Pi 5         │
     │ (12 V power  │   │   │ (USB-C 5 V)  │
     │  jack)       │   │   └──────┬───────┘
     └──────┬───────┘   │          │
            │           │     USB-C│data
       TTL  │           │          │
       bus  │           │   ┌──────┴───────┐
     ┌──┬──┬┤──┬──┐     │   │ OAK-D S2     │
     │  │  │   │  │     │   │ (USB-C)      │
     m1 m2 m3 m4 m5     │   └──────────────┘
     XM  XM  XM  XL XL  │          ▲
     430 540 430 430 430 └──────────┘
                              5 V power
```

### Connections Summary

| From | To | Cable / Connection | Purpose |
|------|----|--------------------|---------|
| PSU 12 V output | 15 A fuse | Wire + klemme (terminal) | Over-current protection |
| 15 A fuse | OpenRB-150 power jack | Barrel connector | 12 V power to motors |
| PSU 5 V output | Raspberry Pi 5 | USB-C power cable | 5 V host power |
| PSU 5 V output | OAK-D S2 | USB-C (via Pi USB port or direct) | 5 V camera power |
| PSU GND | All components | Common bus (felles minus) | Ground reference |
| Raspberry Pi 5 | OpenRB-150 | USB-C data cable | Serial (115200 baud, `/dev/ttyACM0`) |
| Raspberry Pi 5 | OAK-D S2 | USB-C data cable | Video + depth stream |
| OpenRB-150 | Dynamixel servos | 3-pin TTL daisy-chain | Motor commands + telemetry |

## Motor Wiring (TTL Daisy-Chain)

The five Dynamixel servos are connected in a TTL daisy-chain from the OpenRB-150's Dynamixel port:

```
OpenRB-150 ──► [ID 1] XM430 ──► [ID 2] XM540 ──► [ID 3] XM430 ──► [ID 4] XL430 ──► [ID 5] XM430
               Base          Shoulder         Elbow           Wrist          Claw
```

Each servo has two JST connectors — one input, one output to the next servo. The last servo (ID 5) has no output connected.

**Pin mapping (TTL 3-pin connector):**

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | GND | Ground |
| 2 | VCC | 12 V supply (passed through from OpenRB-150) |
| 3 | DATA | Half-duplex TTL serial |

## Power Notes

- The PSU provides **two independent regulated outputs** — the 5 V rail comes directly from a dedicated PSU output, not from a voltage conversion of the 12 V rail.
- The **15 A fuse** on the 12 V rail protects against short circuits in the motor chain.
- The Raspberry Pi 5 requires a stable 5 V / 3 A supply via USB-C for reliable operation (especially with USB peripherals).
- The OAK-D S2 can draw up to 2.5 W; ensure the 5 V rail has sufficient current capacity.

## Firmware

The OpenRB-150 runs custom firmware located at [`firmware/openrb_bridge/openrb_bridge.ino`](../firmware/openrb_bridge/openrb_bridge.ino).

- **Language:** C++ (Arduino framework)
- **Flashed via:** Arduino IDE with ROBOTIS OpenRB-150 board support
- **Baud rate:** 115200 (USB serial to Pi)
- **Libraries:** Dynamixel2Arduino, ArduinoJson v7

> The OpenRB-150 is flashed using the **Arduino IDE**, but the project does **not** use a separate Arduino microcontroller. The OpenRB-150 is the only embedded board.

See [docs/calibration.md](calibration.md) Step 01 for flashing instructions.
