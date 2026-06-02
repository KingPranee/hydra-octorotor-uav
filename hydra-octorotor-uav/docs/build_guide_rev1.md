# Build Guide — HYDRA Rev 1.0 (Premium)

## Tools Required

- M3 hex driver set
- M5 hex driver
- Measuring tape / calipers
- Threadlocker (Loctite Blue 243) — for all flight-critical fasteners
- Velcro (hook+loop) strips
- Cable ties (3mm width)
- Wire stripper (if any pre-made cables need trimming)

> ⚠️ No soldering required for this build. All connections are pre-terminated XT60/XT90 plug-and-play.

---

## Build Sequence

### Phase 1 — Frame Assembly

1. Lay out both CF centre plates (300×300mm)
2. Install brass M5 standoffs (40mm) at the 8 arm attachment points on the bottom plate
3. Slide carbon fibre arm tubes (500mm, 25mm OD) into the 3D-printed arm clamps
4. Bolt arm clamps to bottom centre plate — **do not fully tighten yet**
5. Arrange arms at equal 45° intervals (flat octa X):
   - Arm angles: 22.5°, 67.5°, 112.5°, 157.5°, 202.5°, 247.5°, 292.5°, 337.5° from forward
6. Verify equal arm lengths (measure tip to centre) → tighten clamp bolts with Loctite

### Phase 2 — Motor & ESC Mounting

7. Mount T-Motor MN5212 motors to arm tips using the CNC aluminium motor mounts
   - 4× M3×10 screws per motor — apply Loctite
   - Verify motor spin direction label faces up
8. Attach T-Motor Flame 60A ESCs under each arm with double-sided foam tape + cable tie
   - Position ESC ~15cm from motor (short motor wire run)
   - Route ESC signal wire along arm toward centre

### Phase 3 — Power Distribution

9. Mount XT90 PDB at centre of bottom plate
10. Connect 8× ESC XT30 power cables to PDB outputs
    - Note polarity: red = positive, black = negative
11. Install AttoPilot 180A current sensor in series with main positive lead
12. Install 5V/5A UBEC — output to FC power rail

### Phase 4 — Flight Controller

13. Install Cube Orange+ on carrier board with 4× anti-vibration standoffs
14. Mount carrier board on top centre plate using nylon standoffs
15. Connect:
    - GPS (Here3+ CAN) → CAN1 port
    - ELRS receiver → TELEM2 (CRSF)
    - Telemetry radio → TELEM1
    - ESC signal wires → MAIN OUT 1–8 (see wiring_guide.md for motor order)
    - Power module → PM1
    - UBEC 5V → FC servo rail
16. Route all signal wires along arms with cable ties — keep away from motor wires

### Phase 5 — Payload Mount

17. Mount gimbal plate below bottom CF plate on vibration dampers
18. Install DS3225 servo (roll axis) — connect to AUX 1
19. Install DS3225 servo (pitch axis) — connect to AUX 2
20. Route servo power from dedicated 6V BEC (NOT from FC rail)

### Phase 6 — First Power-On Checks

- [ ] Battery polarity verified with multimeter before first connection
- [ ] All ESC connectors fully seated
- [ ] No loose wires near propeller arcs
- [ ] Connect battery — no smoke/heat → proceed
- [ ] FC LEDs illuminate → connect to Mission Planner via USB
- [ ] Load `parameters/hydra_rev1.parm`
- [ ] Run accelerometer calibration
- [ ] Run compass calibration (rotate in figure-8)
- [ ] Verify all 8 motors spin in correct direction (without props!)
- [ ] Run ESC calibration via Mission Planner

### Phase 7 — Prop Installation

- Install CW propellers on CW motors, CCW propellers on CCW motors
- Tighten prop nuts firmly — self-tightening direction (CCW nut on CW motor, CW nut on CCW motor)
- All prop bolts with Loctite Blue

### Phase 8 — First Hover Test

1. Choose open outdoor area, no obstacles
2. Arm in Stabilize mode
3. Gradually increase throttle to ~40% — should hover ~35% by parameter
4. Check for oscillations → reduce `ATC_RAT_RLL_P/I` if oscillating
5. Land, review logs in Mission Planner DataFlash viewer

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| FC won't arm | Pre-arm check failure | Check Mission Planner Messages tab |
| Motor spins wrong way | Motor phase wiring | Swap any 2 of 3 motor wires |
| Oscillations on roll/pitch | PIDs too high | Reduce `ATC_RAT_RLL_P` by 20% |
| Toilet-bowl drift in Loiter | Compass interference | Move GPS further from motors |
| EKF variance errors | Vibration | Add foam dampers under FC |
| Battery voltage reading wrong | BATT_VOLT_MULT wrong | Calibrate in Mission Planner |
