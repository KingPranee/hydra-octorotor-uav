# HYDRA v4 — Motor Shutdown System
### PixHawk 2.4.8 (primary nav) + SpeedyBee F405 Wing (payload FC) + Raspberry Pi

---

## What is HYDRA

HYDRA is a payload-triggered adaptive frame reconfiguration system for an ArduPilot octocopter. When HX711 load-cell readings confirm payload release, motors 5–8 are gracefully disabled and ArduPilot redistributes thrust across the remaining quad (M1–M4). Mixer compensation automatically scales yaw PIDs to preserve authority. When payload weight recovers, all eight motors re-enable.

v4 replaces the single-hypothesis PDD and simple Kalman filter from v3 with a full Unscented Kalman Filter, a three-hypothesis Bayesian drop detector, a PID-controlled transition gate, ESC health regression, vibration-adaptive loop rate, motor mixing matrix compensation, cryptographic black-box logging, a Prometheus metrics endpoint, a live gRPC streaming server, and a fault injection framework for bench testing.

---

## Architecture

```
PixHawk 2.4.8 (primary FC)          SpeedyBee F405 Wing (payload FC)
  GPS, RC, Mission, Attitude  ──────►  MAVLink passthrough M1-M4 PWM
  TELEM1 → RPi GPIO17 (RX)            UART3 ↔ RPi GPIO14/15 (MAVLink RW)
                                        UART2 ← HX711 ASCII (9600)
                                        SD card: motor_controller_v4.lua
                                        SERVO5-8 → ESC M5-M8

Raspberry Pi 3B+/4
  GPIO5/6   ← HX711 primary  (DOUT/SCK)
  GPIO23/24 ← HX711 secondary (DOUT/SCK, optional)
  GPIO14/15 ↔ SpeedyBee UART3 (MAVLink 57600)
  GPIO17    ← PixHawk TELEM1 TX (monitoring 57600)
  :9090     → Prometheus /metrics scrape endpoint
  :50051    → gRPC HydraControl service (optional)
```

Two completely independent control paths run simultaneously:

| Path | Where | Input | Motor control |
|------|--------|-------|--------------|
| **Lua** | SpeedyBee SD card | HX711 → UART2 | `param:set_and_save()` |
| **Python** | Raspberry Pi | HX711 → GPIO | DroneKit `PARAM_SET` |

---

## File Index

```
HYDRA_v4/
├── companion/
│   ├── hydra_companion_v4.py     Main Python companion (2 400+ lines)
│   ├── hydra_grpc_server.py      gRPC service + ground client stub
│   └── hydra_v4.service          systemd unit file
│
├── lua/
│   └── motor_controller_v4.lua   Onboard Lua (SpeedyBee, 1 150+ lines)
│
├── config/
│   ├── speedybee_f405_v4.param   ArduPilot params — SpeedyBee
│   └── pixhawk_248_v4.param      ArduPilot params — PixHawk 2.4.8
│
├── ground_tools/
│   ├── blackbox_analyze_v4.py    Post-flight JSONL analyzer + matplotlib report
│   └── blackbox_verify.py        HMAC-SHA256 chain verifier
│
├── fault_schedules/
│   └── standard_bench_test.json  Fault injection test schedule
│
├── requirements.txt
└── README.md
```

---

## v4 Feature Summary

### 1. Unscented Kalman Filter (2-state)
State `[mass_g, mass_dot_g_per_s]` fuses the HX711 reading with vertical acceleration (baro altitude rate proxy). Aerodynamic load on the payload mount is decoupled from true mass. Both Python (NumPy sigma points) and Lua (pure-Lua scalar UKF) implementations run simultaneously.

### 2. Multi-Hypothesis Drop Detector (MHDD)
Three parallel detectors feed a Bayesian combiner:
- **H0** — OLS linear regression (slope + R²)
- **H1** — CUSUM change-point detector on UKF mass estimate
- **H2** — Load-cell variance collapse (payload slack / sudden unload)

Posterior `P(drop) = Σ wᵢ·Pᵢ`. Transition fires when posterior ≥ 0.75.

### 3. PID Transition Gate
Error integral must accumulate past `pid_commit_threshold` before `PRE_DROP → T_QUAD` fires. Eliminates false triggers from brief mechanical disturbances. Hard dwell cap (`pid_dwell`) provides a worst-case latency bound.

### 4. Motor Mixing Matrix Compensation
Reads `ATC_RAT_YAW_P/I` before disabling M5-M8, scales by 1.85×, writes back. Restores originals on re-enable. Yaw authority is preserved across the 8→4 transition without requiring ArduPilot reboot.

### 5. Vibration-Adaptive Loop Rate
EMA of IMU vibration magnitude continuously adjusts the main loop from 2 Hz (high vibration) to 10 Hz (smooth hover). UKF measurement noise `R` is also scaled proportionally, preventing vibration from causing spurious MHDD triggers.

### 6. ESC Health Regression
Per-motor RPM time series (60 s window) is fitted with OLS. A motor with declining RPM trend (slope < −5 RPM/s, p < 0.05) is flagged as `degrading` before it fails. FSM enters `DEGRADED` state and commands RTL if any primary motor (M1–M4) is degrading.

### 7. Dual HX711 Sensor Arbitration
Two independent load cells on separate GPIO pairs. Inverse-variance weighted fusion when sensors agree. Mahalanobis distance gate detects divergence and falls back to the healthy sensor. UKF treats sensor failure as increased `R`, not data loss.

### 8. Aerodynamic Trim Correction
During ATE baseline collection, vertical acceleration bias is measured and subtracted. Threshold is set on true mass, not apparent weight (which varies with throttle).

### 9. Cryptographic Black-Box Logging
Every JSONL record is appended with `"_mac": HMAC-SHA256(key, prev_mac || payload)`. Key is derived from the FC UID read at boot. `blackbox_verify.py` can detect any tampered or truncated records post-flight.

### 10. Prometheus Metrics Endpoint
`http://<rpi-ip>:9090/metrics` exposes Prometheus-format gauges for weight, UKF mass, MHDD posterior, FSM state, ESC RPM × 8, ESC health × 8, vibration, loop rate. Scrapeable by Grafana in real time.

### 11. gRPC Bidirectional Server
`HydraControl` service on port 50051:
- `StreamStatus` — server-streaming 5 Hz `MotorSystemStatus` proto
- `SendCommand` — unary: `EMERGENCY_SHUTDOWN / FORCE_ENABLE_ALL / RTL / …`
- `UpdateConfig` — unary: change threshold/PDD/PID params in-flight
- `BidirectionalControl` — full bidi stream for advanced GCS integration

### 12. Fault Injection Framework
JSON schedule drives synthetic faults: `weight_drop`, `jerk_spike`, `hb_drop`, `sensor_fail`. Each fault is time-sequenced from arm. Use with `--fault-inject` for bench testing without hardware.

### 13. Seven-State FSM
`OCTOCOPTER → PRE_DROP → T_QUAD → QUAD → T_OCTO → OCTOCOPTER`  
Plus `DEGRADED` (primary motor declining, RTL commanded) and `EMERGENCY` (immediate all-motor shutdown, reboot required).

### 14. Boot Self-Test
On first arm, each secondary motor (M5–M8) is briefly spun and RPM is verified via Bidirectional DSHOT. Blocks if any motor fails to spin.

---

## Installation

### 1. Load ArduPilot parameters

**PixHawk 2.4.8** — Mission Planner → Full Parameter List → Load from file:
```
config/pixhawk_248_v4.param
```

**SpeedyBee F405 Wing** — Mission Planner → Full Parameter List → Load from file:
```
config/speedybee_f405_v4.param
```

Key params to verify after load:

| FC | Param | Value | Why |
|----|-------|-------|-----|
| SpeedyBee | `SCR_ENABLE` | 1 | Lua engine |
| SpeedyBee | `SCR_HEAP_SIZE` | 200000 | UKF + ring buffers |
| SpeedyBee | `SERIAL2_PROTOCOL` | 28 | HX711 scripting UART |
| SpeedyBee | `SERVO_BLH_BDSHOT` | 1 | Bidirectional DSHOT → RPM |
| Both | `FRAME_CLASS` | 2 | Octocopter |
| Both | `FRAME_TYPE` | 0 | X layout |

### 2. Install Lua script on SpeedyBee

Format an SD card FAT32 and copy:
```
SD:/APM/scripts/motor_controller_v4.lua
```
Power-cycle SpeedyBee. Within 5 s, GCS Messages should show:
```
[HYDRA-v4] motor_controller_v4.lua loaded
[HYDRA-v4][XXXXXXXX] boot_sync: all motors active → OCTO
[HYDRA-v4][XXXXXXXX] HYDRA v4.0 READY | thr=500g hyst=75g UKF=on MHDD=on MIXER=true
```

### 3. Set up Raspberry Pi

```bash
# On fresh Raspberry Pi OS Lite (64-bit), disable Bluetooth for serial
echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt
echo "dtoverlay=uart5"      | sudo tee -a /boot/config.txt
sudo sed -i 's/console=serial0,[0-9]* //' /boot/cmdline.txt
sudo systemctl disable hciuart bluetooth

# Install Python deps
mkdir ~/hydra_v4 && cd ~/hydra_v4
pip3 install -r requirements.txt --break-system-packages

# Copy companion files
cp companion/hydra_companion_v4.py .
cp companion/hydra_grpc_server.py .
cp -r fault_schedules ground_tools .

# Install service
sudo cp companion/hydra_v4.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable hydra_v4
sudo reboot
```

### 4. Calibrate load cell(s)

```bash
# After reboot
python3 ground_tools/hx711_calibrate.py --dout 5 --sck 6
# Follow interactive prompts → get --scale value
# Edit hydra_v4.service to set --scale <value>
# If using second sensor:
python3 ground_tools/hx711_calibrate.py --dout 23 --sck 24
# Edit --scale2 <value>

sudo systemctl daemon-reload && sudo systemctl restart hydra_v4
```

### 5. Verify

```bash
sudo systemctl status hydra_v4
journalctl -u hydra_v4 -f
# Expect: "SpeedyBee OK" and "PixHawk OK" within 10 s
```

---

## Wiring

### HX711 Primary → RPi

| HX711 | RPi GPIO (BCM) |
|-------|---------------|
| DOUT  | GPIO 5 |
| SCK   | GPIO 6 |
| VCC   | 3.3 V |
| GND   | GND |

### HX711 Secondary → RPi (optional)

| HX711 | RPi GPIO (BCM) |
|-------|---------------|
| DOUT  | GPIO 23 |
| SCK   | GPIO 24 |
| VCC   | 3.3 V |
| GND   | GND |

### RPi → SpeedyBee UART3

| RPi | SpeedyBee |
|-----|-----------|
| GPIO 14 (TX) | UART3 RX |
| GPIO 15 (RX) | UART3 TX |
| GND | GND |

### RPi → PixHawk TELEM1 (monitoring only)

| RPi | PixHawk |
|-----|---------|
| GPIO 17 (RX) | TELEM1 TX |
| GND | GND |

### HX711 Adapter → SpeedyBee UART2 (Lua path)

| Adapter | SpeedyBee |
|---------|-----------|
| TX (ASCII) | UART2 RX |
| GND | GND |

---

## Ground Tools

### Post-Flight Analysis

```bash
# Full report
python3 ground_tools/blackbox_analyze_v4.py blackbox.jsonl

# With matplotlib 6-panel report
python3 ground_tools/blackbox_analyze_v4.py blackbox.jsonl --plot

# With HTML wrapper
python3 ground_tools/blackbox_analyze_v4.py blackbox.jsonl --plot --html

# With CSV event export
python3 ground_tools/blackbox_analyze_v4.py blackbox.jsonl --csv
```

### Cryptographic Log Verification

```bash
# Verify HMAC chain
python3 ground_tools/blackbox_verify.py blackbox.jsonl

# With detailed stats
python3 ground_tools/blackbox_verify.py blackbox.jsonl --stats

# Export event timeline CSV
python3 ground_tools/blackbox_verify.py blackbox.jsonl --export

# Remove tampered records
python3 ground_tools/blackbox_verify.py blackbox.jsonl --fix
```

### Live gRPC Monitor

```bash
# On ground station (requires grpcio installed):
python3 companion/hydra_grpc_server.py --client --host <rpi-ip> --port 50051
```

### Prometheus + Grafana

Scrape `http://<rpi-ip>:9090/metrics` from your Prometheus instance.
Metrics: `hydra_weight_g`, `hydra_ukf_mass_g`, `hydra_mhdd_posterior`,
`hydra_fsm_state_code`, `hydra_esc_rpm{motor="N"}`, `hydra_esc_health{motor="N"}`.

### Fault Injection Testing

```bash
# Bench test (no FCs needed — sim mode)
python3 companion/hydra_companion_v4.py \
    --no-px \
    --fault-inject fault_schedules/standard_bench_test.json \
    --sim-drop-time 50 \
    --blackbox /tmp/bench_test.jsonl

# Then analyze results
python3 ground_tools/blackbox_analyze_v4.py /tmp/bench_test.jsonl --plot
```

---

## Sentinel Parameters

The Lua script writes these ArduPilot parameters every 2 seconds. Read them from Mission Planner or MAVProxy to get live FSM state without parsing GCS text:

| Param | Units | Description |
|-------|-------|-------------|
| `HYDRA_STATE` | 0–6 | FSM state code (0=OCTO … 6=EMER) |
| `HYDRA_MASS` | grams | UKF mass estimate (integer) |
| `HYDRA_POST` | ×1000 | MHDD posterior (750 = 0.750) |
| `HYDRA_PID_I` | ×1000 | PID integral output |
| `HYDRA_VIBE` | ×100 | Vibration EMA |
| `HYDRA_PX_HB` | seconds | PixHawk heartbeat timestamp (written by Python companion) |
| `HYDRA_M_DEGRADE` | 1–4 or 0 | Degrading primary motor index (0 = none) |

---

## Key Tunable Parameters

### Python companion (CLI flags in `hydra_v4.service`)

| Flag | Default | Effect |
|------|---------|--------|
| `--threshold` | 500 g | Base weight threshold (ATE overwrites) |
| `--hysteresis` | 75 g | Re-enable dead-band |
| `--mhdd-post` | 0.75 | Bayesian posterior trigger level |
| `--pid-kp/ki/kd` | 0.05/0.02/0.005 | Transition PID gains |
| `--pid-thr` | 1.0 | PID output commit threshold |
| `--pid-dwell` | 2.0 s | Hard cap on PRE_DROP wait |
| `--jerk-max` | 3.0 rad/s² | Angular jerk gate |
| `--att-limit` | 10° | Attitude gate |
| `--att-diverge` | 5° | FC attitude coherence alert |

### Lua script (top of `motor_controller_v4.lua`)

| Variable | Default | Effect |
|----------|---------|--------|
| `POSTERIOR_THRESHOLD` | 0.75 | MHDD commit threshold |
| `PID_COMMIT_THOLD` | 1.0 | PID commit threshold |
| `DWELL_MAX_S` | 2.0 s | PRE_DROP hard dwell cap |
| `QUAD_YAW_P_SCALE` | 1.85 | Yaw P gain scale for quad mode |
| `VIBE_HI_THRESHOLD` | 30 m/s² | Vibration → slow loop |
| `CUSUM_H` | 200 g | CUSUM decision threshold |

---

## Pre-Flight Checklist

- [ ] Props **OFF** for all bench tests
- [ ] `HYDRA_STATE` visible in Mission Planner and reads 0 (OCTO)
- [ ] GCS Messages shows `HYDRA v4.0 READY` on SpeedyBee boot
- [ ] `journalctl -u hydra_v4 -f` shows `SpeedyBee OK` and `PixHawk OK`
- [ ] ATE locks within 6 s of arming with payload — `HYDRA_STATE` still 0
- [ ] MHDD posterior visible in Prometheus at `hydra_mhdd_posterior`
- [ ] Simulate slow weight drop on bench — observe PRE_DROP → QUAD with lat < 2 s
- [ ] Jerk spike (tilt FC >30°/s) during PRE_DROP — transition must defer
- [ ] Mixer compensation: `ATC_RAT_YAW_P` reads 1.85× original in QUAD state
- [ ] Verify: YAW_P restored to original on OCTO re-enable
- [ ] Boot self-test passes — all M5–M8 spin briefly on first arm
- [ ] Blackbox log writing: `tail -f ~/hydra_v4/blackbox.jsonl`
- [ ] Log verification: `python3 ground_tools/blackbox_verify.py blackbox.jsonl` — chain intact
- [ ] Common ground: RPi ↔ SpeedyBee ↔ PixHawk ↔ HX711 all share ground
- [ ] Load cell is rigidly mounted — vibration causes false readings
- [ ] Bidirectional DSHOT active: `SERVO_BLH_BDSHOT=1`, ESC RPM visible in logs

---

*Project HYDRA · HALO Aero Club · SSN College of Engineering, Kalavakkam*
*Authors: Praneeth, Devibala S · Advisor: R. Ramaprabha*
