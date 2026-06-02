# HYDRA Octorotor — Wiring Guide

> ⚠️ **No-solder build constraint:** All power connections use pre-terminated XT60/XT90 plugs. Signal wiring uses JST-GH or Dupont. A soldering iron is needed only for the battery harness (or buy pre-made).

---

## Power Architecture

```
[2× 6S LiPo 22000mAh]
       |
  [XT90 Parallel Harness]
       |
  [AttoPilot 180A Current Sensor]
       |
  [8-output XT60 PDB]
   /  /  /  /  |  \  \  \
 ESC ESC ESC ESC ESC ESC ESC ESC
  |   |   |   |   |   |   |   |
 M1  M2  M3  M4  M5  M6  M7  M8

PDB → [UBEC 5V/5A] → FC (Cube Orange+ power rail)
                   → Receiver
                   → Gimbal Servos
```

---

## Motor Order (ArduCopter Flat Octa X)

Looking from above, **clockwise from front-right**:

| Motor # | Position | Spin | Wire Color Tip |
|---|---|---|---|
| M1 | Front-Right | CW  | Red hub |
| M2 | Front-Left  | CCW | Black hub |
| M3 | Right       | CCW | Black hub |
| M4 | Left        | CW  | Red hub |
| M5 | Rear-Right  | CW  | Red hub |
| M6 | Rear-Left   | CCW | Black hub |
| M7 | Right-Rear  | CCW | Black hub |
| M8 | Left-Rear   | CW  | Red hub |

> If a motor spins the wrong way: swap any two of its three phase wires.

---

## ESC → FC Signal Wiring (Cube Orange+ Main Out)

| FC Output Pin | ESC | Motor |
|---|---|---|
| MAIN 1 | ESC 1 | M1 Front-Right |
| MAIN 2 | ESC 2 | M2 Front-Left |
| MAIN 3 | ESC 3 | M3 Right |
| MAIN 4 | ESC 4 | M4 Left |
| MAIN 5 | ESC 5 | M5 Rear-Right |
| MAIN 6 | ESC 6 | M6 Rear-Left |
| MAIN 7 | ESC 7 | M7 Right-Rear |
| MAIN 8 | ESC 8 | M8 Left-Rear |

---

## GPS / Compass (Here3+ CAN)

```
Here3+ → CAN1 port on Cube Orange+ carrier board
       → CAN Splitter (if daisy-chaining other CAN devices)

ArduCopter params:
  GPS_TYPE = 9  (UAVCAN)
  CAN_P1_DRIVER = 1
  CAN_D1_PROTOCOL = 1
```

---

## Receiver (ELRS EP2 Nano)

```
ELRS Receiver → UART (TELEM2 or RCIN on carrier)
  TX → FC RX
  RX → FC TX
  5V → UBEC 5V
  GND → Common GND

ArduCopter params:
  SERIAL2_PROTOCOL = 23  (ELRS / CRSF)
  SERIAL2_BAUD = 115
  RC_PROTOCOLS = 512     (CRSF bitmask)
```

---

## Telemetry (SiK 915MHz)

```
SiK Air Module → TELEM1 on carrier board
  TX → FC RX
  RX → FC TX  
  5V → UBEC
  GND → GND

ArduCopter:
  SERIAL1_PROTOCOL = 1  (MAVLink 1)
  SERIAL1_BAUD = 57     (57600)
```

---

## Payload Gimbal Servos

```
DS3225 Servo (Roll axis)  → AUX 1
DS3225 Servo (Pitch axis) → AUX 2
Servo Power → Separate 6V BEC (NOT from FC rail — too much current)

ArduCopter:
  SERVO9_FUNCTION  = 7   (Mount Tilt)
  SERVO10_FUNCTION = 6   (Mount Roll)
  MNT_TYPE = 1           (Servo gimbal)
```

---

## Wire Gauges

| Connection | Wire Gauge | Max Current |
|---|---|---|
| Battery → PDB (main) | 10 AWG | 120A |
| PDB → ESC | 12 AWG | 60A |
| ESC → Motor | 14 AWG | 40A |
| UBEC → FC/Rx | 22 AWG | 3A |
| Signal wires | 26–28 AWG | <1A |

---

## Pre-Flight Checklist

- [ ] Battery polarity verified before first connection
- [ ] All ESC connectors locked / heat-shrunk
- [ ] Motor rotation directions confirmed
- [ ] ESC calibration completed
- [ ] Props torqued correctly (CW props on CCW motors, CCW props on CW motors)
- [ ] Props clear of wiring harness
- [ ] Vibration dampers on FC mount
- [ ] GPS clear of motor magnetic interference (>10cm away)
