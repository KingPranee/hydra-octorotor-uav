# Project HYDRA — Adaptive Octorotor UAV

> **Self-Levelling Adaptive Octorotor | 10–12 kg Payload Capacity**  
> SSN College of Engineering | Spartan Aero Club

[![ArduCopter](https://img.shields.io/badge/FC-ArduCopter-red)](https://ardupilot.org/copter/)
[![Frame](https://img.shields.io/badge/Frame-Octorotor-blue)]()
[![Payload](https://img.shields.io/badge/Payload-10--12kg-green)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Overview

**Project HYDRA** is a heavy-lift adaptive octorotor UAV designed for 10–12 kg payload capacity. The platform features:

- **Dual motor modes:** 8-motor full power and 4-motor efficient cruise
- **Self-levelling payload mount:** Active gimbal platform using IMU feedback
- **Modular arm design:** Tool-free arm swaps for transport and repair
- **Dual FC redundancy:** Primary + backup flight controller failover

The project has two build specifications:

| Spec | Description | Est. Cost |
|---|---|---|
| **Rev 1.0 — Premium** | Carbon frame, Cube Orange+, T-Motor, Here3+ GPS | ~₹3,60,000 |
| **Rev 2.0 — Budget** | Aluminium frame, Pixhawk 2.4.8, local motors, BN-220 GPS | ~₹67,500 |

---

## Frame Configuration

```
Octorotor — X8 or Flat Octa

         M1(CW)   M2(CCW)
           \       /
M8(CCW)----[BODY]----M3(CW)
           /       \
M7(CW)   M6(CCW)   M4(CCW)   M5(CW)

Flat octa: all 8 motors in single plane
Arm length: 500mm (Rev 1.0) / 450mm (Rev 2.0)
```

---

## Specifications

### Rev 1.0 — Premium Build

| Parameter | Value |
|---|---|
| Frame | Custom carbon fibre flat octa |
| Arm length | 500 mm |
| Motors | T-Motor MN5212 340KV (×8) |
| ESC | T-Motor Flame 60A (×8) |
| Props | 15×5 CF (×8) |
| Flight Controller | Cube Orange+ |
| GPS | Here3+ CAN |
| Battery | 6S 22000mAh (×2 parallel) |
| AUW (no payload) | ~8.5 kg |
| Max payload | 12 kg |
| Flight time (hover, no payload) | ~22 min |
| Flight time (10 kg payload) | ~11 min |
| **Total Cost** | **~₹3,60,000** |

### Rev 2.0 — Budget Build

| Parameter | Value |
|---|---|
| Frame | 30×30mm aluminium square tube |
| Arm length | 450 mm |
| Motors | SunnySky X4108S 380KV (×8) |
| ESC | 40A BLHeli_S (×8) |
| Props | 13×4.5 CF (×8) |
| Flight Controller | Pixhawk 2.4.8 |
| GPS | BN-220 UART |
| Battery | 6S 16000mAh (×2) |
| AUW (no payload) | ~7.2 kg |
| Max payload | 10 kg |
| Flight time (hover, no payload) | ~18 min |
| **Total Cost** | **~₹67,500** |

---

## Repository Structure

```
hydra-octorotor-uav/
├── hardware/
│   ├── BOM/
│   │   ├── BOM_rev1_premium.csv
│   │   └── BOM_rev2_budget.csv
│   └── CAD/               # Frame dimension drawings
├── firmware/
│   ├── ardupilot_params/  # .parm files for Rev 1 and Rev 2
│   └── companion/         # Payload gimbal controller
├── simulation/
│   └── octa_sitl.md       # SITL setup for octorotor
├── parameters/
│   ├── hydra_rev1.parm
│   └── hydra_rev2.parm
└── docs/
    ├── build_guide_rev1.md
    ├── build_guide_rev2.md
    └── wiring_guide.md
```

---

## Wiring Overview (No-Solder Design)

> ⚠️ **Constraint:** All connections use XT60/XT30 plug-and-play connectors. No soldering required for assembly.

| Connection | Method |
|---|---|
| Battery → PDB | XT60 male plug (pre-soldered on cable) |
| PDB → ESC | XT30 or bullet connectors |
| ESC → Motor | 3.5mm banana connector (screw-lock) |
| FC → ESC | JST-SH signal wires (pre-terminated) |
| GPS → FC | JST-GH (Here3+) or Dupont (BN-220) |

---

## ArduCopter Frame Type

```
# Set in Mission Planner or via MAVProxy:
FRAME_TYPE = 12      # Octo Flat (Rev 1.0 / 2.0)
# Alternative:
FRAME_TYPE = 14      # X8 coaxial

FRAME_CLASS = 3      # Octorotor
```

---

## Quick Start

```bash
# 1. Flash ArduCopter to your FC using Mission Planner
# 2. Load parameter file
#    Mission Planner → Config → Full Parameter List → Load from file
#    Select: parameters/hydra_rev1.parm  OR  parameters/hydra_rev2.parm

# 3. Calibrate:
#    - Compass
#    - Accelerometer
#    - Radio (ELRS/Crossfire)
#    - ESC (all at once via FC)

# 4. Pre-arm checks:
#    All checks should pass except GPS (indoor test)
```

---

## License

MIT License — see [LICENSE](LICENSE)

## Acknowledgements

Spartan Aero Club, SSN College of Engineering  
ArduPilot community documentation
