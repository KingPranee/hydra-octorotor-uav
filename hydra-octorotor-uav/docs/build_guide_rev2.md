# Build Guide — HYDRA Rev 2.0 (Budget)

## Key Differences from Rev 1.0

| Aspect | Rev 1.0 | Rev 2.0 |
|---|---|---|
| Frame | CF tubes | 30×30mm aluminium square tube |
| FC | Cube Orange+ | Pixhawk 2.4.8 |
| GPS | Here3+ CAN | BN-220 UART |
| Motors | T-Motor MN5212 | SunnySky X4108S |
| ESC | BLHeli_32 60A | BLHeli_S 40A |
| Signal protocol | DSHOT1200 | Standard PWM |
| Total cost | ~₹3,60,000 | ~₹50,000 |

---

## Tools Required

Same as Rev 1.0, plus:
- Hacksaw or angle grinder (for aluminium tube cutting)
- Metal file (deburring cut edges)
- Drill + M5 drill bit (arm attachment holes)
- Tap set M3 (optional — for threaded inserts)

---

## Frame Construction (Aluminium)

The Rev 2.0 uses **30×30mm square aluminium tube** instead of carbon fibre. This is heavier but far cheaper and available at any hardware store.

### Cutting

```
Cut 8 arm pieces from 30×30×2mm tube:
  Each arm: 450mm long
  Hacksaw or circular saw with aluminium blade
  Deburr all edges with a file
```

### Centre Plates

```
Cut 2× pieces of 3mm AL sheet: 250×250mm
Drill 8 arm-attachment holes (M5) at 45° spacing
Drill 4 FC-mount holes (M3) in centre
```

### Assembly

1. Drill M5 hole through each arm end (for centre plate bolt)
2. Bolt arms to bottom plate at correct angles — same 45° spacing as Rev 1
3. Install top plate with 40mm M5 standoffs
4. No arm clamps needed — bolts pass through arm and plate

---

## ESC Configuration (PWM, not DSHOT)

Pixhawk 2.4.8 clone boards often have PWM-only main outputs. Verify:

```
BLHeli_S 40A ESC → set to PWM mode (not DSHOT)
  MOT_PWM_TYPE = 0   (Normal PWM)
  MOT_PWM_MIN  = 1000
  MOT_PWM_MAX  = 2000

ESC calibration: Mission Planner → ESC Calibration → All at once
```

> ⚠️ **Pixhawk 2.4.8 clones** vary in quality. Before trusting the clone, verify:
> - All 8 MAIN OUT pins output correct PWM
> - USB does not share 5V with servo rail (check with multimeter)
> - Compass is functional (some clones have bad onboard compass — use external)

---

## GPS (BN-220) Wiring

```
BN-220 TX → Pixhawk SERIAL3 RX (GPS port)
BN-220 RX → Pixhawk SERIAL3 TX
BN-220 VCC → 5V
BN-220 GND → GND

ArduCopter params:
  GPS_TYPE = 1    (Auto — detects u-blox protocol)
  SERIAL3_BAUD = 38    (38400 default for BN-220)
  SERIAL3_PROTOCOL = 5 (GPS)
```

The BN-220 outputs standard NMEA and u-blox binary. ArduCopter auto-detects it.

---

## Motor Order (Same as Rev 1)

Same flat octa X layout as Rev 1.0 — see `docs/wiring_guide.md` for motor numbering and spin directions.

---

## Weight Budget

| Component | Mass (g) |
|---|---|
| Aluminium frame (8 arms + 2 plates) | 1850 |
| SunnySky X4108S (×8) | 960 |
| BLHeli_S 40A ESC (×8) | 320 |
| CF props 13" (×8) | 320 |
| Pixhawk 2.4.8 + GPS + Rx | 180 |
| 6S 16000mAh (×2) | 3400 |
| PDB + wiring | 200 |
| 3D printed parts | 150 |
| **AUW (no payload)** | **~7380g** |

---

## Estimated Performance

| Metric | Value |
|---|---|
| Total static thrust | ~12,000g (8× ~1500g per motor at 6S) |
| T/W (no payload) | 12000 / 7380 ≈ 1.63 |
| Hover throttle | ~40% |
| Max payload | ~8kg |
| Flight time (no payload, hover) | ~18 min |
| Flight time (5kg payload) | ~10 min |

---

## Upgrading from Rev 2.0 to Rev 1.0

The Rev 2.0 is designed as a stepping stone. Upgrade path:
1. Replace aluminium arms with CF tubes (same hole pattern)
2. Swap Pixhawk for Cube Orange+ (same ArduCopter config)
3. Upgrade ESCs to DSHOT1200-capable BLHeli_32 60A
4. Upgrade GPS to Here3+ CAN (`GPS_TYPE = 9`)
5. Upgrade battery to 22000mAh pair

Estimated upgrade cost: ~₹3,10,000 in parts.
