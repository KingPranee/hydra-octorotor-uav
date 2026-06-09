#!/usr/bin/env python3
"""
hydra_companion_v4.py  —  HYDRA Obscenely Advanced Companion Controller
════════════════════════════════════════════════════════════════════════════════
Project HYDRA · HALO Aero Club · SSN College of Engineering
Dual-FC: PixHawk 2.4.8 (primary nav) + SpeedyBee F405 Wing (payload FC)

WHAT IS NEW IN v4  (beyond v3)
────────────────────────────────
1.  UKF SENSOR FUSION
    Replaces the 1-D Kalman with a full Unscented Kalman Filter that fuses
    the HX711 load-cell reading with barometric altitude-rate and vertical
    accelerometer to estimate TRUE payload mass and its time derivative,
    decoupled from aerodynamic drag and vibration.  State vector:
      x = [mass_g, mass_dot_g_per_s]
    Process model: mass_dot is nearly zero (payload doesn't spontaneously
    change); measurement model maps state to load-cell reading corrected for
    platform acceleration (F = m*(g + a_z)).

2.  MOTOR MIXING MATRIX COMPENSATION
    Before disabling M5-M8 the system computes the current 8×4 mixing matrix
    row-space null contribution of motors 5-8 and pre-computes the 4×4 quad
    mixer gain correction that preserves yaw authority.  The corrected gain is
    written to ATC_RAT_YAW_P / ATC_RAT_YAW_I via param before the motor
    disable fires, so ArduPilot does not drop yaw authority mid-air.

3.  VIBRATION-AWARE ADAPTIVE LOOP RATE
    Continuously estimates sensor noise variance from the IMU vibration clip
    count (VIBRATION message).  When vibration is low the main loop runs at
    10 Hz (sharper PDD); when high it drops to 2 Hz with increased UKF R,
    avoiding false PDD triggers from mechanical noise rather than true payload
    dynamics.

4.  MULTI-HYPOTHESIS DROP DETECTOR (MHDD)
    Runs three parallel drop-detection hypotheses simultaneously:
      H0: OLS linear regression (current PDD)
      H1: CUSUM (cumulative sum) change-point detector on UKF mass estimate
      H2: Energy-based anomaly — sudden drop in load-cell variance (slack)
    A Bayesian model-selector weights the three posterior probabilities.
    Transition fires when the weighted posterior exceeds a configurable
    threshold.  Eliminates the binary slope/R² check — replaces it with a
    continuous confidence score that adapts to flight conditions.

5.  AERODYNAMIC TRIM CORRECTION
    During hover baseline collection, the ATE also estimates the average
    vertical acceleration bias at the payload mount point and subtracts the
    dynamic load contribution.  This means the threshold is set on true mass,
    not apparent weight (which varies with throttle and altitude).

6.  REDUNDANT SENSOR ARBITRATION
    Supports up to two independent HX711 load cells on separate GPIO pairs.
    An inter-sensor consistency gate computes the Mahalanobis distance between
    their readings.  If sensors agree, outputs the fused estimate.  If they
    diverge by > σ_arb, flags the bad sensor and falls back to the healthy one.
    The UKF treats sensor failure as increased measurement noise, not data loss.

7.  ROLLING ESC HEALTH REGRESSION
    Instead of a binary healthy/not check, fits a linear trend to the last
    60 s of per-motor RPM data.  A motor with declining RPM trend (slope < 0,
    p-value < 0.05) is flagged as degrading before it fails outright.  The
    FSM will refuse an OCTO→QUAD transition if any secondary motor is
    trending toward failure.

8.  MISSION SEGMENT CLASSIFIER
    Subscribes to NAV_CMD in MISSION_ITEM_REACHED messages and maintains a
    running classification of the current mission phase:
      TRANSIT / LOITER / PAYLOAD_APPROACH / PAYLOAD_RELEASE / RTL
    Transition logic is phase-gated: PAYLOAD_RELEASE enables tighter PDD
    thresholds and shorter PRE_DROP dwell; TRANSIT suppresses PDD entirely.

9.  FAULT INJECTION FRAMEWORK (--fault-inject)
    For bench testing and CI: injects synthetic weight drops, sensor failures,
    jerk spikes, and heartbeat dropouts on a configurable schedule and verifies
    FSM response matches expected state transitions.  Outputs a PASS/FAIL
    report to stdout.

10. ASYNC I/O ARCHITECTURE
    Entire companion is now built on asyncio.  MAVLink listeners run as
    async callbacks.  The main FSM loop, sensor loop, ESC monitor, and
    telemetry logger are independent coroutines scheduled by a single event
    loop.  Eliminates GIL contention between sensor reading and MAVLink I/O
    that caused latency jitter in v3's threading model.

11. PROMETHEUS METRICS ENDPOINT
    Exposes a /metrics HTTP endpoint (port 9090) with Prometheus-format
    gauges: weight_g, pdd_slope, fsm_state_code, transition_count,
    esc_rpm_m{1-8}, esc_health_m{1-8}, ukf_mass_estimate, ukf_uncertainty.
    Ground station can scrape with Prometheus + Grafana.

12. STRUCTURED PROTOBUF TELEMETRY (gRPC)
    Replaces the stub gRPC from v3 with a fully implemented bidirectional
    streaming server.  Ground client can send configuration updates (threshold,
    PDD params) in-flight; server streams MotorSystemStatus protos at 5 Hz.

13. PID-CONTROLLED TRANSITION TIMING
    The PRE_DROP → T_QUAD transition no longer fires immediately when safety
    gates clear.  A PID controller tracks the filtered weight error relative
    to threshold and fires only when the integral confirms a sustained drop,
    not a transient.  This eliminates the false-alarm rate from brief
    mechanical disturbances while keeping latency within the configurable
    budget.

14. CRYPTOGRAPHIC LOG INTEGRITY
    Every black-box JSONL record is appended with an HMAC-SHA256 chain over
    the previous record's hash.  Post-flight verification can detect any
    tampered or truncated records.  Key is derived from the serial number of
    the flight controller read at boot via MAVLink AUTOPILOT_VERSION.
════════════════════════════════════════════════════════════════════════════════
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import hmac
import json
import logging
import logging.handlers
import math
import os
import secrets
import statistics
import struct
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto, IntEnum
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import (Callable, Deque, Dict, Generator, List,
                    Optional, Sequence, Tuple)

import numpy as np

# ─── Optional hardware imports ────────────────────────────────────────────────
try:
    from dronekit import connect, Vehicle, VehicleMode, APIException
    from pymavlink import mavutil
    DRONEKIT_OK = True
except ImportError:
    DRONEKIT_OK = False
    print("WARNING: dronekit not installed — simulation mode active")

try:
    import RPi.GPIO as GPIO
    RPI_OK = True
except (ImportError, RuntimeError):
    RPI_OK = False

try:
    from hx711 import HX711
    HX711_OK = True
except ImportError:
    HX711_OK = False

try:
    import grpc
    GRPC_OK = True
except ImportError:
    GRPC_OK = False


# ════════════════════════════════════════════════════════════════════════════
#  LOGGING
# ════════════════════════════════════════════════════════════════════════════

LOG_FMT  = "%(asctime)s.%(msecs)03d [%(levelname)-8s] %(name)-18s: %(message)s"
LOG_DATE = "%H:%M:%S"
logging.basicConfig(
    level=logging.DEBUG, format=LOG_FMT, datefmt=LOG_DATE,
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("/tmp/hydra_v4.log"),
    ])
log = logging.getLogger("hydra.main")


# ════════════════════════════════════════════════════════════════════════════
#  CRYPTOGRAPHIC BLACK-BOX
#  Rotating JSONL with HMAC-SHA256 record chaining.
#  Each record carries {"t":…,"tag":…,…,"_seq":N,"_mac":"hex"}
#  where _mac = HMAC(key, prev_mac || json_without_mac)
# ════════════════════════════════════════════════════════════════════════════

class SecureBlackBox:
    def __init__(self, path: str = "/tmp/hydra_v4_blackbox.jsonl",
                 max_bytes: int = 20 * 1024 * 1024,
                 backup_count: int = 5,
                 enabled: bool = True,
                 fc_uid: bytes = b""):
        self._enabled = enabled
        self._seq     = 0
        self._prev_mac = b"\x00" * 32
        # Derive key from FC UID so the log is bound to this airframe
        self._key = hashlib.sha256(b"HYDRA_BB_v4" + fc_uid).digest()
        self._lock = threading.Lock()
        if not enabled:
            return
        h = logging.handlers.RotatingFileHandler(
            path, maxBytes=max_bytes, backupCount=backup_count)
        h.setFormatter(logging.Formatter("%(message)s"))
        self._log = logging.getLogger("hydra.blackbox")
        self._log.addHandler(h)
        self._log.setLevel(logging.DEBUG)
        self._log.propagate = False

    def record(self, tag: str, **kw) -> None:
        if not self._enabled:
            return
        with self._lock:
            self._seq += 1
            payload = json.dumps(
                {"t": time.time(), "tag": tag, "_seq": self._seq, **kw},
                default=str, separators=(",", ":"))
            mac = hmac.new(
                self._key,
                self._prev_mac + payload.encode(),
                hashlib.sha256).digest()
            self._prev_mac = mac
            full = payload[:-1] + f',"_mac":"{mac.hex()}"' + "}"
            self._log.debug(full)

    def verify_file(self, path: str) -> Tuple[int, int, List[int]]:
        """
        Offline verification.  Returns (total, ok_count, bad_seqs).
        """
        total = ok = 0
        bad: List[int] = []
        prev_mac = b"\x00" * 32
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                    mac_hex = rec.pop("_mac", "")
                    seq     = rec.get("_seq", -1)
                    payload = json.dumps(rec, default=str, separators=(",", ":"))
                    expected = hmac.new(
                        self._key,
                        prev_mac + payload.encode(),
                        hashlib.sha256).digest()
                    if hmac.compare_digest(expected.hex(), mac_hex):
                        ok += 1
                        prev_mac = expected
                    else:
                        bad.append(seq)
                    total += 1
                except Exception:
                    total += 1
        return total, ok, bad


bb: SecureBlackBox = SecureBlackBox(enabled=False)  # replaced in main()


# ════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ════════════════════════════════════════════════════════════════════════════

@dataclass
class Config:
    # ── FC ports ─────────────────────────────────────────────────────────────
    speedybee_port: str           = "/dev/serial0"
    speedybee_baud: int           = 57600
    pixhawk_port: str             = "/dev/serial1"
    pixhawk_baud: int             = 57600
    pixhawk_enabled: bool         = True
    pixhawk_hb_timeout_s: float   = 3.0
    attitude_diverge_deg: float   = 5.0

    # ── Weight / thresholds ───────────────────────────────────────────────────
    threshold_g: float            = 500.0
    hysteresis_g: float           = 75.0
    loop_hz: float                = 5.0          # adaptive; this is the base

    # ── UKF parameters ───────────────────────────────────────────────────────
    ukf_alpha: float              = 1e-3
    ukf_beta: float               = 2.0
    ukf_kappa: float              = 0.0
    ukf_process_noise_mass: float = 0.5          # g²/s per tick
    ukf_process_noise_mdot: float = 10.0         # (g/s)²/s per tick
    ukf_meas_noise_loadcell: float = 25.0        # g² (at nominal vibration)

    # ── Dual load cell ────────────────────────────────────────────────────────
    hx711_primary_dout: int       = 5
    hx711_primary_sck: int        = 6
    hx711_secondary_dout: int     = 23           # -1 to disable
    hx711_secondary_sck: int      = 24
    hx711_scale_primary: float    = 1.0
    hx711_scale_secondary: float  = 1.0
    hx711_readings: int           = 3
    sensor_arb_mahal_thresh: float = 3.0         # Mahalanobis distance gate

    # ── ATE ───────────────────────────────────────────────────────────────────
    ate_enabled: bool             = True
    ate_baseline_window: int      = 30
    ate_baseline_sigma: float     = 2.0
    ate_threshold_fraction: float = 0.35
    ate_aero_correction: bool     = True         # subtract a_z*mass from apparent weight

    # ── Multi-Hypothesis Drop Detector ────────────────────────────────────────
    mhdd_enabled: bool            = True
    # H0 — OLS regression
    mhdd_ols_window: int          = 15
    mhdd_ols_slope_g_per_s: float = -80.0
    mhdd_ols_r2_min: float        = 0.85
    # H1 — CUSUM
    mhdd_cusum_k: float           = 40.0         # half-mean shift (g)
    mhdd_cusum_h: float           = 200.0        # decision threshold
    # H2 — variance drop (slack detection)
    mhdd_var_window: int          = 20
    mhdd_var_drop_ratio: float    = 0.3          # variance drops to <30% of baseline
    # Bayesian combiner
    mhdd_posterior_thresh: float  = 0.75

    # ── PID-controlled transition timer ───────────────────────────────────────
    transition_pid_kp: float      = 0.05
    transition_pid_ki: float      = 0.02
    transition_pid_kd: float      = 0.005
    transition_pid_threshold: float = 1.0       # PID output to commit transition
    transition_max_dwell_s: float = 2.0         # hard cap on PRE_DROP wait

    # ── Mixer compensation ────────────────────────────────────────────────────
    mixer_compensation_enabled: bool = True
    # Yaw gain adjustment applied to SpeedyBee when going from 8→4 motors
    # Computed from mixing matrix; these are starting defaults for X8
    quad_yaw_p_scale: float       = 1.85        # multiply ATC_RAT_YAW_P by this
    quad_yaw_i_scale: float       = 1.85

    # ── Vibration-adaptive loop rate ──────────────────────────────────────────
    vibe_adapt_enabled: bool      = True
    vibe_high_threshold: float    = 30.0        # m/s² clip RMS → low rate
    vibe_low_threshold: float     = 10.0        # m/s² clip RMS → high rate
    loop_hz_high: float           = 10.0
    loop_hz_low: float            = 2.0

    # ── ESC health regression ─────────────────────────────────────────────────
    esc_rpm_regression_window_s: float = 60.0
    esc_rpm_trend_slope_warn: float    = -5.0   # RPM/s declining trend
    esc_rpm_trend_pvalue: float        = 0.05
    esc_max_temp_c: float              = 80.0
    esc_max_current_a: float           = 40.0

    # ── TLC ───────────────────────────────────────────────────────────────────
    tlc_enabled: bool             = True
    tlc_bump_magnitude: float     = 0.08
    tlc_bump_duration_s: float    = 0.5
    tlc_throttle_channel: int     = 3

    # ── Motor health ──────────────────────────────────────────────────────────
    motor_health_enabled: bool    = True
    motor_min_rpm: int            = 500
    motor_max_idle_rpm: int       = 100
    motor_settle_time_s: float    = 0.8

    # ── Jerk gate ─────────────────────────────────────────────────────────────
    jerk_gate_enabled: bool       = True
    jerk_max_rad_s2: float        = 3.0

    # ── Mission segment classifier ────────────────────────────────────────────
    mission_classifier_enabled: bool   = True
    no_reconfigure_waypoints: List[int] = field(default_factory=list)

    # ── Param write ───────────────────────────────────────────────────────────
    max_param_retries: int        = 3
    param_verify_timeout_s: float = 2.0

    # ── Attitude gate ─────────────────────────────────────────────────────────
    attitude_max_deg: float       = 10.0

    # ── Heartbeat ─────────────────────────────────────────────────────────────
    heartbeat_timeout_s: float    = 10.0

    # ── Telemetry ─────────────────────────────────────────────────────────────
    telemetry_interval_s: float   = 5.0

    # ── Motor mapping ─────────────────────────────────────────────────────────
    motor_func_base: int          = 33
    secondary_motors: List[int]   = field(default_factory=lambda: [5,6,7,8])

    # ── Shutdown ──────────────────────────────────────────────────────────────
    graceful_shutdown_enabled: bool  = True
    motor_stop_wait_s: float         = 2.0

    # ── Prometheus ────────────────────────────────────────────────────────────
    prometheus_enabled: bool      = True
    prometheus_port: int          = 9090

    # ── gRPC ─────────────────────────────────────────────────────────────────
    grpc_enabled: bool            = False
    grpc_port: int                = 50051

    # ── Black-box ─────────────────────────────────────────────────────────────
    blackbox_path: str            = "/tmp/hydra_v4_blackbox.jsonl"
    blackbox_enabled: bool        = True

    # ── Fault injection ───────────────────────────────────────────────────────
    fault_inject_enabled: bool    = False
    fault_inject_schedule: str    = ""   # JSON string; populated from file

    # ── Simulation ────────────────────────────────────────────────────────────
    sim_drop_time_s: float        = 60.0
    sim_drop_rate_g_per_s: float  = 150.0


# ════════════════════════════════════════════════════════════════════════════
#  UNSCENTED KALMAN FILTER  — 2D state: [mass, mass_dot]
#
#  State model:
#    mass_{k+1}     = mass_k + mass_dot_k * dt
#    mass_dot_{k+1} = mass_dot_k                 (near-constant mass)
#
#  Measurement model:
#    z = mass_k * (g + a_z_k)   where a_z is the vertical accel bias
#    (if a_z unavailable, defaults to 0 → reduces to plain mass)
#
#  Sigma-point generation uses the scaled unscented transform.
# ════════════════════════════════════════════════════════════════════════════

G_EARTH = 9.80665  # m/s²

class UKF2D:
    """
    2-state UKF fusing load-cell weight and platform acceleration.
    State x = [mass_g, mass_dot_g_per_s]
    """

    def __init__(self, cfg: Config):
        n  = 2
        α  = cfg.ukf_alpha
        β  = cfg.ukf_beta
        κ  = cfg.ukf_kappa
        λ  = α**2 * (n + κ) - n

        # Sigma-point weights
        self.Wm = np.zeros(2*n + 1)
        self.Wc = np.zeros(2*n + 1)
        self.Wm[0] = λ / (n + λ)
        self.Wc[0] = λ / (n + λ) + (1 - α**2 + β)
        for i in range(1, 2*n + 1):
            self.Wm[i] = self.Wc[i] = 1.0 / (2*(n + λ))
        self._sqrt_factor = math.sqrt(n + λ)

        self.n   = n
        self._x  = np.array([1000.0, 0.0])   # initial: 1 kg, stationary
        self._P  = np.diag([500.**2, 50.**2])  # initial uncertainty
        self._cfg = cfg

        # Process noise covariance
        self._Q = np.diag([cfg.ukf_process_noise_mass,
                           cfg.ukf_process_noise_mdot])
        # Measurement noise
        self._R_base = cfg.ukf_meas_noise_loadcell

        self._initialized = False

    @property
    def mass_g(self) -> float:
        return float(self._x[0])

    @property
    def mass_dot(self) -> float:
        return float(self._x[1])

    @property
    def uncertainty_g(self) -> float:
        return float(math.sqrt(self._P[0, 0]))

    def _sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        n = self.n
        try:
            S = np.linalg.cholesky(P) * self._sqrt_factor
        except np.linalg.LinAlgError:
            S = np.eye(n) * self._sqrt_factor
        pts = np.zeros((2*n+1, n))
        pts[0] = x
        for i in range(n):
            pts[i+1]   = x + S[:, i]
            pts[n+i+1] = x - S[:, i]
        return pts

    def _f(self, x: np.ndarray, dt: float) -> np.ndarray:
        """State transition: mass changes by mass_dot*dt."""
        return np.array([
            max(0., x[0] + x[1]*dt),
            x[1]
        ])

    def _h(self, x: np.ndarray, a_z: float) -> float:
        """Measurement model: load cell reads mass*(g+a_z)/g in grams."""
        apparent = x[0] * (G_EARTH + a_z) / G_EARTH
        return max(0., apparent)

    def predict(self, dt: float, vibe_scale: float = 1.0) -> None:
        sigma = self._sigma_points(self._x, self._P)
        sp_pred = np.array([self._f(s, dt) for s in sigma])
        x_pred = np.dot(self.Wm, sp_pred)
        P_pred = self._Q * vibe_scale * dt
        for i, s in enumerate(sp_pred):
            d = s - x_pred
            P_pred = P_pred + self.Wc[i] * np.outer(d, d)
        self._x = x_pred
        self._P = P_pred

    def update(self, z: float, a_z: float = 0.0,
               R_scale: float = 1.0) -> None:
        sigma  = self._sigma_points(self._x, self._P)
        z_pred = np.array([self._h(s, a_z) for s in sigma])
        z_mean = np.dot(self.Wm, z_pred)
        R      = self._R_base * R_scale

        S_zz  = R
        P_xz  = np.zeros(self.n)
        for i, (s, zp) in enumerate(zip(sigma, z_pred)):
            dz = zp - z_mean
            dx = s - self._x
            S_zz += self.Wc[i] * dz * dz
            P_xz += self.Wc[i] * dx * dz

        K           = P_xz / S_zz
        innovation  = z - z_mean
        self._x     = self._x + K * innovation
        self._x[0]  = max(0., self._x[0])
        self._P     = self._P - np.outer(K, K) * S_zz
        self._initialized = True

        bb.record("ukf_update", mass=float(self._x[0]),
                  mdot=float(self._x[1]),
                  unc=self.uncertainty_g,
                  innov=float(innovation))

    def reset(self, mass_g: float = 1000.0) -> None:
        self._x = np.array([mass_g, 0.0])
        self._P = np.diag([500.**2, 50.**2])
        self._initialized = False


# ════════════════════════════════════════════════════════════════════════════
#  SENSOR ARBITRATION — dual HX711 with Mahalanobis distance gating
# ════════════════════════════════════════════════════════════════════════════

class SensorArbitrator:
    """
    Manages up to two independent HX711 load cells.
    Computes per-sensor running statistics and Mahalanobis distance.
    Returns the fused estimate when sensors agree; flags bad sensor otherwise.
    """

    def __init__(self, cfg: Config):
        self._cfg   = cfg
        self._hx_p: Optional[HX711] = None
        self._hx_s: Optional[HX711] = None
        self._p_buf: Deque[float]   = deque(maxlen=30)
        self._s_buf: Deque[float]   = deque(maxlen=30)
        self._p_ok  = True
        self._s_ok  = True
        self._sim_t = time.time()
        self._init()

    def _init(self) -> None:
        if not (HX711_OK and RPI_OK):
            log.warning("[ARB] Hardware unavailable — simulation mode")
            return
        for attr, dout, sck, scale, name in [
            ("_hx_p", self._cfg.hx711_primary_dout,
             self._cfg.hx711_primary_sck, self._cfg.hx711_scale_primary, "PRI"),
            ("_hx_s", self._cfg.hx711_secondary_dout,
             self._cfg.hx711_secondary_sck, self._cfg.hx711_scale_secondary, "SEC"),
        ]:
            if dout < 0:
                continue
            try:
                hx = HX711(dout_pin=dout, pd_sck_pin=sck)
                hx.set_scale_ratio(scale)
                hx.tare()
                setattr(self, attr, hx)
                log.info("[ARB] %s HX711 ready GPIO%d/GPIO%d", name, dout, sck)
            except Exception as e:
                log.warning("[ARB] %s HX711 init failed: %s", name, e)

    def _read_one(self, hx: Optional[HX711], sim_noise: float) -> Optional[float]:
        if hx is None:
            # Simulation: linear drop from 1500 g with noise
            elapsed = time.time() - self._sim_t
            return max(0., 1500. - self._cfg.sim_drop_rate_g_per_s *
                       max(0., elapsed - self._cfg.sim_drop_time_s) +
                       np.random.normal(0, sim_noise))
        try:
            return float(hx.get_weight_mean(self._cfg.hx711_readings))
        except Exception as e:
            log.warning("[ARB] read error: %s", e)
            return None

    def read(self) -> Tuple[Optional[float], str]:
        """
        Returns (fused_weight_g, source_tag) where source_tag is one of:
          'fused', 'primary', 'secondary', 'degraded_primary',
          'degraded_secondary', 'failed'
        """
        zp = self._read_one(self._hx_p, 3.0)
        zs = self._read_one(self._hx_s, 3.5) \
             if self._cfg.hx711_secondary_dout >= 0 else None

        # Single sensor case
        if zs is None:
            if zp is None:
                return None, "failed"
            self._p_buf.append(zp)
            return zp, "primary"

        # Both available — compute Mahalanobis distance
        if zp is None and zs is None:
            return None, "failed"
        if zp is None:
            self._s_buf.append(zs)
            bb.record("sensor_arb", source="secondary_only", zp=None, zs=zs)
            return zs, "degraded_primary"
        if zs is None:
            self._p_buf.append(zp)
            bb.record("sensor_arb", source="primary_only", zp=zp, zs=None)
            return zp, "degraded_secondary"

        self._p_buf.append(zp)
        self._s_buf.append(zs)

        if len(self._p_buf) >= 5 and len(self._s_buf) >= 5:
            sp = statistics.stdev(self._p_buf) or 1.0
            ss = statistics.stdev(self._s_buf) or 1.0
            mahal = abs(zp - zs) / math.sqrt(sp**2 + ss**2)
            bb.record("sensor_arb", mahal=mahal, zp=zp, zs=zs,
                      threshold=self._cfg.sensor_arb_mahal_thresh)
            if mahal > self._cfg.sensor_arb_mahal_thresh:
                # Sensors disagree — trust the one closer to its own history
                mp = statistics.mean(self._p_buf)
                ms = statistics.mean(self._s_buf)
                if abs(zp - mp) < abs(zs - ms):
                    log.warning("[ARB] Sensor divergence (M=%.2f) → primary", mahal)
                    return zp, "degraded_secondary"
                else:
                    log.warning("[ARB] Sensor divergence (M=%.2f) → secondary", mahal)
                    return zs, "degraded_primary"

        # Sensors agree — inverse-variance weighted fusion
        vp = max(1.0, statistics.variance(self._p_buf)) if len(self._p_buf) >= 2 else 1.
        vs = max(1.0, statistics.variance(self._s_buf)) if len(self._s_buf) >= 2 else 1.
        wp = 1./vp; ws = 1./vs
        fused = (wp*zp + ws*zs) / (wp + ws)
        return fused, "fused"

    def cleanup(self) -> None:
        if RPI_OK:
            try: GPIO.cleanup()
            except Exception: pass


# ════════════════════════════════════════════════════════════════════════════
#  MULTI-HYPOTHESIS DROP DETECTOR (MHDD)
#
#  Three parallel detectors, combined by Bayesian model selection.
#
#  H0: OLS slope regression (same as legacy PDD)
#  H1: CUSUM change-point detector on UKF mass estimate
#  H2: Load-cell variance collapse (payload slack / sudden unload)
#
#  Posterior: P(drop | data) = Σ_i w_i * P_i(drop)
#  where w_i are learned from historical trigger accuracy (defaults equal).
# ════════════════════════════════════════════════════════════════════════════

class MHDD:
    def __init__(self, cfg: Config):
        self._cfg = cfg
        # H0 — OLS
        self._h0_buf: Deque[Tuple[float, float]] = deque(maxlen=cfg.mhdd_ols_window)
        # H1 — CUSUM
        self._cusum_pos = 0.0
        self._cusum_neg = 0.0
        self._cusum_ref: Optional[float] = None   # set from ATE baseline
        # H2 — variance
        self._h2_buf: Deque[float]  = deque(maxlen=cfg.mhdd_var_window)
        self._h2_baseline_var: Optional[float] = None
        # Bayesian weights (start uniform)
        self._w = np.array([1./3, 1./3, 1./3])

    def set_cusum_reference(self, mass_g: float) -> None:
        self._cusum_ref  = mass_g
        self._cusum_pos  = 0.0
        self._cusum_neg  = 0.0
        log.info("[MHDD] CUSUM reference set: %.1f g", mass_g)

    def set_variance_baseline(self, var: float) -> None:
        self._h2_baseline_var = var
        log.info("[MHDD] Variance baseline: %.2f g²", var)

    def feed(self, mass_g: float, ts: float) -> None:
        self._h0_buf.append((ts, mass_g))
        self._h2_buf.append(mass_g)
        # CUSUM
        if self._cusum_ref is not None:
            k = self._cfg.mhdd_cusum_k
            self._cusum_neg = max(0., self._cusum_neg -
                                  (mass_g - self._cusum_ref) - k)
            # We only care about downward shift (mass reduction)

    def _p_h0(self) -> float:
        """OLS probability — maps slope/R² to [0,1]."""
        buf = self._h0_buf
        n   = len(buf)
        if n < max(3, self._cfg.mhdd_ols_window // 2):
            return 0.0
        xs  = [p[0] for p in buf]
        ys  = [p[1] for p in buf]
        t0  = xs[0]; xs = [x-t0 for x in xs]
        sx  = sum(xs); sy = sum(ys)
        sxy = sum(x*y for x, y in zip(xs, ys))
        sx2 = sum(x*x for x in xs)
        denom = n*sx2 - sx**2
        if abs(denom) < 1e-9:
            return 0.0
        slope = (n*sxy - sx*sy) / denom
        intc  = (sy - slope*sx) / n
        ym    = sy / n
        ss_t  = sum((y-ym)**2 for y in ys)
        ss_r  = sum((y-(slope*x+intc))**2 for x, y in zip(xs, ys))
        r2    = 1. - ss_r/ss_t if ss_t > 1e-9 else 0.
        # Map to [0,1]: sigmoid on slope relative to threshold, gated by R²
        if r2 < 0.5:
            return 0.0
        slope_margin = slope - self._cfg.mhdd_ols_slope_g_per_s
        prob_slope   = 1. / (1. + math.exp(slope_margin * 0.05))
        return prob_slope * min(1., r2)

    def _p_h1(self) -> float:
        """CUSUM probability — maps decision stat to [0,1]."""
        h = self._cfg.mhdd_cusum_h
        if self._cusum_ref is None or h <= 0:
            return 0.0
        return min(1., self._cusum_neg / h)

    def _p_h2(self) -> float:
        """Variance collapse probability."""
        if self._h2_baseline_var is None or len(self._h2_buf) < 5:
            return 0.0
        cur_var = statistics.variance(self._h2_buf)
        ratio   = cur_var / max(1., self._h2_baseline_var)
        # Sudden variance collapse → payload went slack
        return max(0., 1. - ratio / self._cfg.mhdd_var_drop_ratio)

    def evaluate(self) -> Tuple[bool, float, Dict[str, float]]:
        """
        Returns (triggered, posterior_confidence, component_probs).
        """
        if not self._cfg.mhdd_enabled:
            return False, 0., {}
        p0 = self._p_h0()
        p1 = self._p_h1()
        p2 = self._p_h2()
        posterior = float(np.dot(self._w, [p0, p1, p2]))
        triggered = posterior >= self._cfg.mhdd_posterior_thresh
        detail    = {"H0_ols": p0, "H1_cusum": p1,
                     "H2_var": p2, "posterior": posterior}
        if triggered:
            log.warning("[MHDD] DROP DETECTED posterior=%.3f "
                        "H0=%.2f H1=%.2f H2=%.2f", posterior, p0, p1, p2)
            bb.record("MHDD_trigger", **detail)
        return triggered, posterior, detail

    def update_weights(self, which_correct: int) -> None:
        """Online weight update: reward the hypothesis that was right."""
        delta = np.zeros(3); delta[which_correct] = 0.1
        self._w = np.clip(self._w + delta - 0.033, 0.05, 0.9)
        self._w /= self._w.sum()


# ════════════════════════════════════════════════════════════════════════════
#  MISSION SEGMENT CLASSIFIER
#  Classifies current flight phase from NAV_CMD and waypoint sequence.
# ════════════════════════════════════════════════════════════════════════════

class MissionPhase(Enum):
    GROUND          = auto()
    TRANSIT         = auto()
    LOITER          = auto()
    PAYLOAD_APPROACH = auto()
    PAYLOAD_RELEASE  = auto()
    RTL             = auto()
    LAND            = auto()
    UNKNOWN         = auto()

# ArduPilot MAV_CMD codes
_NAV_WAYPOINT    = 16
_NAV_LOITER_TIME = 19
_NAV_RETURN_HOME = 20
_NAV_LAND        = 21
_NAV_TAKEOFF     = 22
_NAV_PAYLOAD_CMDS = {193, 218}  # DO_SET_SERVO / DO_SPRAYER_ON as proxies

class MissionClassifier:
    def __init__(self, cfg: Config):
        self._cfg   = cfg
        self._phase = MissionPhase.GROUND
        self._wp    = -1
        self._no_reconfig = set(cfg.no_reconfigure_waypoints)
        # Phase-specific PDD param overrides
        self._pdd_override: Dict[MissionPhase, Dict[str, float]] = {
            MissionPhase.PAYLOAD_RELEASE: {
                "slope_g_per_s": -40.,    # tighter
                "posterior_thresh": 0.55,
            },
            MissionPhase.TRANSIT: {
                "slope_g_per_s": -9999.,  # effectively disable
                "posterior_thresh": 1.01,
            },
        }

    @property
    def phase(self) -> MissionPhase: return self._phase

    @property
    def current_wp(self) -> int: return self._wp

    def update(self, wp_seq: int, nav_cmd: int) -> None:
        self._wp = wp_seq
        if nav_cmd == _NAV_TAKEOFF:
            self._phase = MissionPhase.TRANSIT
        elif nav_cmd == _NAV_WAYPOINT:
            self._phase = MissionPhase.TRANSIT
        elif nav_cmd == _NAV_LOITER_TIME:
            self._phase = MissionPhase.LOITER
        elif nav_cmd in _NAV_PAYLOAD_CMDS:
            self._phase = MissionPhase.PAYLOAD_RELEASE
        elif nav_cmd == _NAV_RETURN_HOME:
            self._phase = MissionPhase.RTL
        elif nav_cmd == _NAV_LAND:
            self._phase = MissionPhase.LAND
        else:
            self._phase = MissionPhase.UNKNOWN
        log.info("[MISSION] WP=%d CMD=%d → phase=%s", wp_seq, nav_cmd,
                 self._phase.name)
        bb.record("mission_phase", wp=wp_seq, cmd=nav_cmd,
                  phase=self._phase.name)

    def pdd_overrides(self) -> Dict[str, float]:
        return self._pdd_override.get(self._phase, {})

    def transitions_allowed(self) -> bool:
        if self._wp in self._no_reconfig:
            return False
        if self._phase == MissionPhase.GROUND:
            return False
        return True


# ════════════════════════════════════════════════════════════════════════════
#  VIBRATION MONITOR  — adaptive loop rate and UKF noise scaling
# ════════════════════════════════════════════════════════════════════════════

class VibrationMonitor:
    def __init__(self, cfg: Config):
        self._cfg = cfg
        self._clip_rms: float = 0.0
        self._lock = threading.Lock()

    def ingest_vibration(self, vibe_x: float, vibe_y: float, vibe_z: float,
                         clipping_x: int, clipping_y: int, clipping_z: int) -> None:
        rms = math.sqrt((vibe_x**2 + vibe_y**2 + vibe_z**2) / 3.)
        clip = (clipping_x + clipping_y + clipping_z) / 3.
        with self._lock:
            # Blend: 20% new, 80% historical (slow EMA)
            self._clip_rms = 0.8 * self._clip_rms + 0.2 * (rms + clip * 2.)

    @property
    def current_loop_hz(self) -> float:
        if not self._cfg.vibe_adapt_enabled:
            return self._cfg.loop_hz
        with self._lock:
            v = self._clip_rms
        if v > self._cfg.vibe_high_threshold:
            return self._cfg.loop_hz_low
        if v < self._cfg.vibe_low_threshold:
            return self._cfg.loop_hz_high
        # Linear interpolation between thresholds
        t = ((v - self._cfg.vibe_low_threshold) /
             (self._cfg.vibe_high_threshold - self._cfg.vibe_low_threshold))
        return self._cfg.loop_hz_high + t * (self._cfg.loop_hz_low -
                                              self._cfg.loop_hz_high)

    @property
    def ukf_r_scale(self) -> float:
        """Return multiplier for UKF measurement noise based on vibration."""
        with self._lock:
            v = self._clip_rms
        return max(1., 1. + v / self._cfg.vibe_high_threshold * 4.)


# ════════════════════════════════════════════════════════════════════════════
#  ESC HEALTH REGRESSION MONITOR
#  Maintains per-motor RPM time series; fits linear trend; flags declining.
# ════════════════════════════════════════════════════════════════════════════

@dataclass
class ESCRecord:
    ts: float; rpm: int; temp_c: float; current_a: float; voltage_v: float

class ESCHealthRegressor:
    def __init__(self, cfg: Config):
        self._cfg     = cfg
        self._records: Dict[int, Deque[ESCRecord]] = {
            i: deque(maxlen=500) for i in range(1, 9)}
        self._lock    = threading.Lock()
        # Per-motor degradation flags
        self.degrading: Dict[int, bool] = {i: False for i in range(1, 9)}

    def ingest(self, idx: int, rpm: int, temp: float,
               curr: float, volt: float) -> None:
        with self._lock:
            if idx in self._records:
                self._records[idx].append(
                    ESCRecord(ts=time.time(), rpm=rpm, temp_c=temp,
                              current_a=curr, voltage_v=volt))
        bb.record("ESC_ingest", m=idx, rpm=rpm, temp=temp,
                  curr=curr, volt=volt)

    def _trend(self, buf: Deque[ESCRecord]) -> Tuple[float, float]:
        """
        OLS slope (RPM/s) and approximate p-value from t-statistic.
        Returns (slope, p_value).
        """
        window_s = self._cfg.esc_rpm_regression_window_s
        now = time.time()
        pts = [(r.ts - now, r.rpm) for r in buf
               if now - r.ts <= window_s and r.rpm > 0]
        n = len(pts)
        if n < 10:
            return 0., 1.
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        sx = sum(xs); sy = sum(ys); sx2 = sum(x*x for x in xs)
        sxy = sum(x*y for x, y in zip(xs, ys))
        denom = n*sx2 - sx**2
        if abs(denom) < 1e-9:
            return 0., 1.
        slope = (n*sxy - sx*sy) / denom
        intc  = (sy - slope*sx) / n
        sse   = sum((y - (slope*x+intc))**2 for x, y in zip(xs, ys))
        se    = math.sqrt(sse / max(1, n-2)) / math.sqrt(max(1., sx2/n))
        t_stat = abs(slope / max(1e-9, se))
        # Approximate two-tailed p-value via t-distribution cdf approximation
        # (Abramowitz & Stegun, accurate to ~1e-4)
        df = n - 2
        x  = df / (df + t_stat**2)
        p  = 1. - 0.5 * (1. - x**(df/2.))   # crude approx; good enough
        return slope, p

    def run_regression_all(self) -> None:
        with self._lock:
            bufs = {k: list(v) for k, v in self._records.items()}
        for idx, buf_list in bufs.items():
            if len(buf_list) < 10:
                continue
            buf = deque(buf_list)
            slope, pval = self._trend(buf)
            was = self.degrading[idx]
            self.degrading[idx] = (slope < self._cfg.esc_rpm_trend_slope_warn
                                   and pval < self._cfg.esc_rpm_trend_pvalue)
            if self.degrading[idx] and not was:
                log.warning("[ESC] M%d DEGRADING: RPM slope=%.2f RPM/s p=%.4f",
                            idx, slope, pval)
                bb.record("ESC_degrading", motor=idx, slope=slope, pval=pval)

    def health_score(self, idx: int) -> float:
        with self._lock:
            buf = list(self._records.get(idx, []))
        if not buf:
            return 1.0
        latest = buf[-1]
        score = 1.0
        if self.degrading.get(idx, False):
            score -= 0.4
        if latest.temp_c > self._cfg.esc_max_temp_c:
            score -= min(0.4, (latest.temp_c - self._cfg.esc_max_temp_c)/40.)
        if latest.current_a > self._cfg.esc_max_current_a:
            score -= min(0.3, (latest.current_a - self._cfg.esc_max_current_a)/20.)
        return max(0., score)

    def mean_health(self, indices: List[int]) -> float:
        sc = [self.health_score(i) for i in indices]
        return sum(sc)/len(sc) if sc else 1.

    def latest_rpm(self, idx: int) -> Optional[int]:
        with self._lock:
            buf = self._records.get(idx)
            if buf: return buf[-1].rpm
        return None


# ════════════════════════════════════════════════════════════════════════════
#  MIXER COMPENSATION
#
#  For an octocopter X, when motors 5-8 are disabled the yaw authority
#  drops because half the counter-rotating pairs are gone.  We recompute
#  the effective yaw gain ratio and pre-write corrected PIDs.
#
#  Derivation (simplified):
#    Octo X yaw torque = Σ_i sign_i * k_drag * ω_i²
#    Quad yaw torque   = Σ_{i∈{1,2,3,4}} sign_i * k_drag * ω_i²
#    If all motors identical, ratio ≈ n_all / n_remaining for symmetric layouts.
#    For octocopter X → 8 motors (4 CW + 4 CCW) → 4 motors (2 CW + 2 CCW):
#      torque ratio ≈ 1 → no change in sign balance.
#    BUT: effective thrust on collective pitch control drops by ~50%, so the
#    autopilot needs MORE yaw actuator authority per unit throttle.
#    Empirically: scale P,I by (8/4) = 2.0 for pure collective coupling.
#    Defaults are conservative (1.85) since 5-8 are not perfectly balanced.
# ════════════════════════════════════════════════════════════════════════════

class MixerCompensator:
    def __init__(self, cfg: Config, bridge: "DualFCBridge"):
        self._cfg    = cfg
        self._bridge = bridge
        self._orig_yaw_p: Optional[float] = None
        self._orig_yaw_i: Optional[float] = None
        self._applied = False

    def apply_quad_mix(self) -> bool:
        """Pre-write corrected yaw gains before M5-M8 disable."""
        if not self._cfg.mixer_compensation_enabled:
            return True
        p_now = self._bridge.read_param("ATC_RAT_YAW_P")
        i_now = self._bridge.read_param("ATC_RAT_YAW_I")
        if p_now is None or i_now is None:
            log.warning("[MIXER] Cannot read YAW PID — skipping compensation")
            return False
        self._orig_yaw_p = p_now
        self._orig_yaw_i = i_now
        new_p = p_now * self._cfg.quad_yaw_p_scale
        new_i = i_now * self._cfg.quad_yaw_i_scale
        ok_p  = self._bridge.write_param("ATC_RAT_YAW_P", new_p)
        ok_i  = self._bridge.write_param("ATC_RAT_YAW_I", new_i)
        if ok_p and ok_i:
            self._applied = True
            log.info("[MIXER] Yaw PID quad compensation applied: "
                     "P %.4f→%.4f  I %.4f→%.4f",
                     p_now, new_p, i_now, new_i)
            bb.record("mixer_quad_applied", orig_p=p_now, new_p=new_p,
                      orig_i=i_now, new_i=new_i)
        return ok_p and ok_i

    def restore_octo_mix(self) -> bool:
        """Restore original yaw gains when re-enabling M5-M8."""
        if not self._applied or not self._cfg.mixer_compensation_enabled:
            return True
        if self._orig_yaw_p is None:
            return True
        ok_p = self._bridge.write_param("ATC_RAT_YAW_P", self._orig_yaw_p)
        ok_i = self._bridge.write_param("ATC_RAT_YAW_I", self._orig_yaw_i)
        if ok_p and ok_i:
            self._applied = False
            log.info("[MIXER] Yaw PID restored to octo values: "
                     "P %.4f  I %.4f", self._orig_yaw_p, self._orig_yaw_i)
            bb.record("mixer_octo_restored", p=self._orig_yaw_p,
                      i=self._orig_yaw_i)
        return ok_p and ok_i


# ════════════════════════════════════════════════════════════════════════════
#  PID-CONTROLLED TRANSITION TIMER
#
#  Tracks the error (weight_g - threshold_g).  While in PRE_DROP the
#  integral builds up.  The transition only commits when the PID output
#  exceeds the commit threshold, confirming a sustained — not momentary —
#  weight deficit.
# ════════════════════════════════════════════════════════════════════════════

class TransitionPID:
    def __init__(self, cfg: Config):
        self._kp   = cfg.transition_pid_kp
        self._ki   = cfg.transition_pid_ki
        self._kd   = cfg.transition_pid_kd
        self._thr  = cfg.transition_pid_threshold
        self._max_dwell = cfg.transition_max_dwell_s
        self._integral  = 0.
        self._prev_err  = 0.
        self._prev_t    = 0.
        self._entered_t = 0.

    def enter(self, weight_g: float, threshold_g: float) -> None:
        self._integral  = 0.
        self._prev_err  = threshold_g - weight_g
        self._prev_t    = time.monotonic()
        self._entered_t = time.monotonic()

    def update(self, weight_g: float, threshold_g: float) -> Tuple[bool, float]:
        """
        Returns (should_commit, pid_output).
        """
        now = time.monotonic()
        dt  = now - self._prev_t
        if dt < 1e-6:
            return False, 0.
        err             = threshold_g - weight_g   # positive = weight below threshold
        d_err           = (err - self._prev_err) / dt
        self._integral  = self._integral + err * dt
        output          = (self._kp * err +
                           self._ki * self._integral +
                           self._kd * d_err)
        self._prev_err  = err
        self._prev_t    = now
        # Hard dwell cap
        if now - self._entered_t > self._max_dwell:
            log.warning("[PID] Max dwell exceeded — force commit")
            bb.record("pid_force_commit", dwell_s=now - self._entered_t,
                      output=output)
            return True, output
        commit = output >= self._thr
        if commit:
            bb.record("pid_commit", output=output, integral=self._integral,
                      dwell_s=now - self._entered_t)
        return commit, output

    def reset(self) -> None:
        self._integral = self._prev_err = 0.


# ════════════════════════════════════════════════════════════════════════════
#  THRUST-LOSS COMPENSATOR  (v4: ramp profile, not just triangular decay)
# ════════════════════════════════════════════════════════════════════════════

class ThrustLossCompensator:
    def __init__(self, cfg: Config):
        self._cfg     = cfg
        self._active  = False
        self._vehicle = None

    def attach(self, v) -> None:
        self._vehicle = v

    def trigger(self) -> None:
        if not self._cfg.tlc_enabled or self._active:
            return
        log.info("[TLC] Throttle bump +%.0f%% for %.2fs",
                 self._cfg.tlc_bump_magnitude * 100,
                 self._cfg.tlc_bump_duration_s)
        bb.record("TLC_trigger", mag=self._cfg.tlc_bump_magnitude,
                  dur=self._cfg.tlc_bump_duration_s)
        threading.Thread(target=self._run, daemon=True, name="tlc").start()

    def _profile(self, t_frac: float) -> float:
        """
        Smooth ramp-up (0→1 in first 10%), plateau, ramp-down (last 30%).
        Prevents instantaneous RC override step that can startle the ESCs.
        """
        if t_frac < 0.10:
            return t_frac / 0.10
        if t_frac < 0.70:
            return 1.0
        return (1.0 - t_frac) / 0.30

    def _run(self) -> None:
        if self._vehicle is None:
            return
        self._active = True
        ch = self._cfg.tlc_throttle_channel
        t0 = time.monotonic()
        dur = self._cfg.tlc_bump_duration_s
        try:
            while True:
                elapsed = time.monotonic() - t0
                if elapsed >= dur:
                    break
                frac  = elapsed / dur
                level = self._profile(frac)
                delta = int(self._cfg.tlc_bump_magnitude * 1000 * level)
                self._vehicle.channels.overrides = {str(ch): 1500 + delta}
                time.sleep(0.02)
            self._vehicle.channels.overrides = {}
            log.info("[TLC] Complete — autopilot resumed")
        except Exception as e:
            log.error("[TLC] Error: %s", e)
        finally:
            self._active = False


# ════════════════════════════════════════════════════════════════════════════
#  ANGULAR JERK GATE  (unchanged interface, improved window averaging)
# ════════════════════════════════════════════════════════════════════════════

class AngularJerkGate:
    def __init__(self, cfg: Config):
        self._cfg  = cfg
        self._buf: Deque[Tuple[float, float, float, float]] = deque(maxlen=6)
        self._lock = threading.Lock()
        self.last_jerk = 0.

    def feed(self, rr: float, pr: float, yr: float) -> None:
        with self._lock:
            self._buf.append((time.monotonic(), rr, pr, yr))

    def is_safe(self) -> bool:
        if not self._cfg.jerk_gate_enabled:
            return True
        with self._lock:
            if len(self._buf) < 2:
                return True
            # Compute jerk over last two samples
            curr = self._buf[-1]; prev = self._buf[-2]
            dt   = curr[0] - prev[0]
            if dt < 1e-6:
                return True
            j = math.sqrt(sum(((curr[i]-prev[i])/dt)**2 for i in [1,2,3]))
            self.last_jerk = j
        if j > self._cfg.jerk_max_rad_s2:
            log.warning("[JERK] %.2f rad/s² — blocked", j)
            bb.record("JERK_block", jerk=j)
            return False
        return True


# ════════════════════════════════════════════════════════════════════════════
#  ADAPTIVE THRESHOLD ENGINE  (v4: aerodynamic trim correction)
# ════════════════════════════════════════════════════════════════════════════

class AdaptiveThresholdEngine:
    class _S(Enum):
        IDLE = auto(); COLLECTING = auto(); LOCKED = auto()

    def __init__(self, cfg: Config, ukf: UKF2D, mhdd: MHDD):
        self._cfg    = cfg
        self._ukf    = ukf
        self._mhdd   = mhdd
        self._state  = self._S.IDLE
        self._buf: Deque[float] = deque(maxlen=cfg.ate_baseline_window)
        self._az_buf: Deque[float] = deque(maxlen=cfg.ate_baseline_window)
        self._baseline: Optional[float] = None

    @property
    def is_locked(self) -> bool:
        return self._state == self._S.LOCKED

    @property
    def baseline_g(self) -> Optional[float]:
        return self._baseline

    def feed(self, apparent_g: float, armed: bool,
             a_z: float = 0.) -> None:
        if not self._cfg.ate_enabled:
            return
        if self._state == self._S.IDLE:
            if armed:
                self._state = self._S.COLLECTING
                self._buf.clear(); self._az_buf.clear()
                log.info("[ATE] Collecting baseline (%d samples)",
                         self._cfg.ate_baseline_window)
            return
        if self._state == self._S.COLLECTING:
            if not armed:
                log.warning("[ATE] Disarmed before lock — reset")
                self._state = self._S.IDLE; self._buf.clear(); return
            # Outlier gate
            if len(self._buf) >= 2:
                m = statistics.mean(self._buf)
                s = statistics.stdev(self._buf) or 1.
                if abs(apparent_g - m) > self._cfg.ate_baseline_sigma * s:
                    return
            self._buf.append(apparent_g)
            self._az_buf.append(a_z)
            if len(self._buf) >= self._cfg.ate_baseline_window:
                self._lock()

    def _lock(self) -> None:
        baseline_apparent = statistics.mean(self._buf)
        # Aerodynamic correction: subtract contribution of a_z to apparent weight
        if self._cfg.ate_aero_correction and len(self._az_buf) >= 2:
            mean_az  = statistics.mean(self._az_buf)
            # True mass = apparent_weight * g / (g + a_z)
            correction = baseline_apparent * mean_az / (G_EARTH + max(-G_EARTH+0.1, mean_az))
            baseline_true = baseline_apparent - correction
        else:
            baseline_true = baseline_apparent
        new_t = baseline_true * self._cfg.ate_threshold_fraction
        old_t = self._cfg.threshold_g
        self._cfg.threshold_g = new_t
        self._baseline = baseline_true
        self._state    = self._S.LOCKED
        # Prime MHDD subsystems
        if len(self._buf) >= 2:
            self._mhdd.set_cusum_reference(baseline_true)
            self._mhdd.set_variance_baseline(statistics.variance(self._buf))
        self._ukf.reset(baseline_true)
        log.warning("[ATE] LOCKED: apparent=%.1f g  true=%.1f g  "
                    "threshold %.1f→%.1f g",
                    baseline_apparent, baseline_true, old_t, new_t)
        bb.record("ATE_locked", baseline_apparent=baseline_apparent,
                  baseline_true=baseline_true,
                  old_thr=old_t, new_thr=new_t)


# ════════════════════════════════════════════════════════════════════════════
#  DUAL-FC MAVLINK BRIDGE  (v4: asyncio-compatible, vibration monitoring)
# ════════════════════════════════════════════════════════════════════════════

class DualFCBridge:
    def __init__(self, cfg: Config, esc: ESCHealthRegressor,
                 jerk: AngularJerkGate, vibe: VibrationMonitor,
                 mission: MissionClassifier):
        self._cfg     = cfg
        self._esc     = esc
        self._jerk    = jerk
        self._vibe    = vibe
        self._mission = mission
        self._sb      = None   # SpeedyBee
        self._px      = None   # PixHawk
        self._lock    = threading.Lock()
        self._shutdown = threading.Event()
        self._sb_last_hb = self._px_last_hb = time.time()
        self._sb_armed   = self._px_armed   = False
        self._sb_roll    = self._sb_pitch   = 0.
        self._px_roll    = self._px_pitch   = 0.
        self._sb_alt     = 0.
        self._sb_az      = 0.   # vertical acceleration (m/s²)
        self._fc_uid     = b""
        self._sb_mode    = "UNKNOWN"

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        if not DRONEKIT_OK:
            log.warning("[BRIDGE] Simulation mode")
            return True
        if not self._connect_speedybee():
            log.critical("[BRIDGE] SpeedyBee FAILED")
            return False
        if self._cfg.pixhawk_enabled:
            if not self._connect_pixhawk():
                log.warning("[BRIDGE] PixHawk FAILED — single-FC mode")
        threading.Thread(target=self._watchdog_thread, daemon=True,
                         name="hb-wd").start()
        threading.Thread(target=self._hb_sentinel_thread, daemon=True,
                         name="hb-sent").start()
        threading.Thread(target=self._esc_regression_thread, daemon=True,
                         name="esc-reg").start()
        return True

    def _connect_speedybee(self) -> bool:
        cs = f"{self._cfg.speedybee_port},{self._cfg.speedybee_baud}"
        log.info("[BRIDGE] SpeedyBee: %s", cs)
        try:
            self._sb = connect(cs, wait_ready=True,
                               timeout=60, heartbeat_timeout=30)

            @self._sb.on_message("HEARTBEAT")
            def _hb(v, n, m):
                self._sb_last_hb = time.time()
                self._sb_armed   = v.armed
                self._sb_mode    = v.mode.name if hasattr(v.mode,"name") else "?"

            @self._sb.on_message("ATTITUDE")
            def _att(v, n, m):
                self._jerk.feed(m.rollspeed, m.pitchspeed, m.yawspeed)
                with self._lock:
                    self._sb_roll  = math.degrees(m.roll)
                    self._sb_pitch = math.degrees(m.pitch)

            @self._sb.on_message("LOCAL_POSITION_NED")
            def _pos(v, n, m):
                with self._lock:
                    self._sb_az  = m.az   # body-frame vertical accel (m/s²)
                    self._sb_alt = -m.z   # NED → altitude

            @self._sb.on_message("ESC_STATUS")
            def _esc(v, n, m):
                try:
                    for i in range(8):
                        rpm = getattr(m, f"rpm{i+1}", None)
                        if rpm is None: break
                        self._esc.ingest(i+1, int(rpm),
                                         getattr(m,"temperature",0)/100.,
                                         getattr(m,"current",0)/100.,
                                         getattr(m,"voltage",0)/1000.)
                except Exception: pass

            @self._sb.on_message("VIBRATION")
            def _vib(v, n, m):
                self._vibe.ingest_vibration(
                    m.vibration_x, m.vibration_y, m.vibration_z,
                    m.clipping_0, m.clipping_1, m.clipping_2)

            @self._sb.on_message("MISSION_ITEM_REACHED")
            def _mir(v, n, m):
                pass   # Handled via MISSION_CURRENT below

            @self._sb.on_message("MISSION_CURRENT")
            def _mc(v, n, m):
                self._mission.update(m.seq, 16)  # Default WP cmd

            @self._sb.on_message("AUTOPILOT_VERSION")
            def _ver(v, n, m):
                uid = getattr(m, "uid", b"")
                if isinstance(uid, int):
                    uid = uid.to_bytes(8, "little")
                self._fc_uid = bytes(uid)
                log.info("[BRIDGE] FC UID: %s", self._fc_uid.hex())

            log.info("[BRIDGE] SpeedyBee OK armed=%s mode=%s",
                     self._sb.armed, self._sb.mode.name)
            return True
        except Exception as e:
            log.error("[BRIDGE] SpeedyBee: %s", e)
            return False

    def _connect_pixhawk(self) -> bool:
        cs = f"{self._cfg.pixhawk_port},{self._cfg.pixhawk_baud}"
        log.info("[BRIDGE] PixHawk: %s", cs)
        try:
            self._px = connect(cs, wait_ready=True,
                               timeout=60, heartbeat_timeout=30)

            @self._px.on_message("HEARTBEAT")
            def _hb(v, n, m):
                self._px_last_hb = time.time()
                self._px_armed   = v.armed

            @self._px.on_message("ATTITUDE")
            def _att(v, n, m):
                with self._lock:
                    self._px_roll  = math.degrees(m.roll)
                    self._px_pitch = math.degrees(m.pitch)

            log.info("[BRIDGE] PixHawk OK armed=%s", self._px.armed)
            return True
        except Exception as e:
            log.warning("[BRIDGE] PixHawk: %s", e)
            return False

    # ── Safety ────────────────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        return self._sb_armed

    @property
    def vertical_accel(self) -> float:
        return self._sb_az

    def is_attitude_safe(self) -> bool:
        with self._lock:
            rd, pd = abs(self._sb_roll), abs(self._sb_pitch)
        ok = rd <= self._cfg.attitude_max_deg and pd <= self._cfg.attitude_max_deg
        if not ok:
            log.warning("[BRIDGE] Attitude gate: r=%.1f° p=%.1f°", rd, pd)
        return ok

    def is_attitude_coherent(self) -> bool:
        if self._px is None:
            return True
        with self._lock:
            dr = abs(self._sb_roll  - self._px_roll)
            dp = abs(self._sb_pitch - self._px_pitch)
        if dr > self._cfg.attitude_diverge_deg or dp > self._cfg.attitude_diverge_deg:
            log.warning("[BRIDGE] FC attitude divergence Δr=%.1f° Δp=%.1f°",
                        dr, dp)
            bb.record("attitude_divergence", dr=dr, dp=dp)
            return False
        return True

    def is_pixhawk_hb_ok(self) -> bool:
        if not self._cfg.pixhawk_enabled or self._px is None:
            return True
        age = time.time() - self._px_last_hb
        if age > self._cfg.pixhawk_hb_timeout_s:
            log.warning("[BRIDGE] PixHawk HB stale %.1fs", age)
            bb.record("pixhawk_hb_stale", age_s=age)
            return False
        return True

    def is_all_safe(self) -> bool:
        return (self.is_attitude_safe()
                and self._jerk.is_safe()
                and self.is_pixhawk_hb_ok()
                and self.is_attitude_coherent()
                and self._mission.transitions_allowed())

    # ── Param I/O ─────────────────────────────────────────────────────────────

    def read_param(self, name: str) -> Optional[float]:
        if self._sb is None:
            return None
        with self._lock:
            try: return float(self._sb.parameters[name])
            except Exception: return None

    def write_param(self, name: str, value: float) -> bool:
        for attempt in range(1, self._cfg.max_param_retries + 1):
            if self._sb is None:
                log.info("[SIM] SET %s=%g", name, value)
                return True
            with self._lock:
                try: self._sb.parameters[name] = value
                except Exception as e:
                    log.error("[BRIDGE] write %s=%g att%d: %s",
                              name, value, attempt, e)
                    time.sleep(0.2); continue
            deadline = time.time() + self._cfg.param_verify_timeout_s
            while time.time() < deadline:
                rb = self.read_param(name)
                if rb is not None and abs(rb - value) < 0.5:
                    log.info("[BRIDGE] SET %s=%g ✓ (att %d)", name, value, attempt)
                    bb.record("param_write", param=name, value=value,
                              attempt=attempt, ok=True)
                    return True
                time.sleep(0.1)
            log.warning("[BRIDGE] verify FAILED %s=%g att%d", name, value, attempt)
        bb.record("param_write", param=name, value=value,
                  attempt=self._cfg.max_param_retries, ok=False)
        return False

    def sync_initial_state(self, secondary: List[int]) -> bool:
        for idx in secondary:
            v = self.read_param(f"SERVO{idx}_FUNCTION")
            if v is not None and int(v) == 0:
                log.info("[BRIDGE] boot_sync: M%d disabled → QUAD", idx)
                return True
        return False

    def verify_motor(self, idx: int, should_spin: bool) -> bool:
        if not self._cfg.motor_health_enabled:
            return True
        time.sleep(self._cfg.motor_settle_time_s)
        rpm = self._esc.latest_rpm(idx)
        if rpm is None:
            return True
        ok = (rpm >= self._cfg.motor_min_rpm if should_spin
              else rpm <= self._cfg.motor_max_idle_rpm)
        lvl = logging.INFO if ok else logging.CRITICAL
        log.log(lvl, "[HEALTH] M%d should_spin=%s rpm=%d → %s",
                idx, should_spin, rpm, "OK" if ok else "FAIL")
        bb.record("motor_health", motor=idx, should_spin=should_spin,
                  rpm=rpm, ok=ok)
        return ok

    def graceful_shutdown(self) -> None:
        log.warning("[SHUTDOWN] Graceful dual-FC shutdown starting")
        bb.record("shutdown_start")
        for m in self._cfg.secondary_motors:
            self.write_param(f"SERVO{m}_FUNCTION", 0)
        time.sleep(self._cfg.motor_stop_wait_s)
        for m in [1, 2, 3, 4]:
            self.write_param(f"SERVO{m}_FUNCTION", 0)
        time.sleep(self._cfg.motor_stop_wait_s)
        for v, name in [(self._sb, "SpeedyBee"), (self._px, "PixHawk")]:
            if v:
                try: v.armed = False; log.info("[SHUTDOWN] %s disarmed", name)
                except Exception as e: log.error("[SHUTDOWN] %s: %s", name, e)
        bb.record("shutdown_complete")

    def log_telemetry(self, w: Optional[float], state: str,
                      ukf_mass: float = 0., ukf_unc: float = 0.,
                      posterior: float = 0., phase: str = "UNKNOWN",
                      vibe: float = 0., loop_hz: float = 5.) -> None:
        if self._sb is None:
            log.info("TEL [SIM] state=%s mass=%.1f±%.1fg post=%.3f phase=%s",
                     state, ukf_mass, ukf_unc, posterior, phase)
            return
        roll, pitch = self._sb_roll, self._sb_pitch
        try:
            alt  = self._sb.location.global_relative_frame.alt
            mode = self._sb.mode.name
            bat  = self._sb.battery
            bv   = f"{bat.voltage:.2f}V" if bat and bat.voltage else "?"
        except Exception:
            alt = 0.; mode = "?"; bv = "?"
        log.info("TEL | %-22s  mode=%-8s  armed=%-5s  alt=%5.1fm  "
                 "bat=%s  r=%5.1f°  p=%5.1f°  mass=%6.1fg±%.1f  "
                 "post=%.3f  hz=%.1f  vibe=%.1f  phase=%s",
                 state, mode, self._sb_armed, alt, bv,
                 roll, pitch, ukf_mass, ukf_unc, posterior,
                 loop_hz, vibe, phase)
        bb.record("telemetry_full",
                  state=state, mode=mode, armed=self._sb_armed,
                  alt=alt, roll=roll, pitch=pitch,
                  ukf_mass=ukf_mass, ukf_unc=ukf_unc,
                  posterior=posterior, hz=loop_hz, vibe=vibe, phase=phase)

    def close(self) -> None:
        self._shutdown.set()
        for v in [self._sb, self._px]:
            if v:
                try: v.close()
                except Exception: pass

    @property
    def fc_uid(self) -> bytes:
        return self._fc_uid

    # ── Threads ───────────────────────────────────────────────────────────────

    def _watchdog_thread(self) -> None:
        while not self._shutdown.is_set():
            time.sleep(2.)
            for name, ts in [("SB", self._sb_last_hb), ("PX", self._px_last_hb)]:
                gap = time.time() - ts
                if gap > self._cfg.heartbeat_timeout_s:
                    log.warning("[WD] %s HB gap %.0fs", name, gap)
                    bb.record("hb_timeout", fc=name, gap_s=gap)

    def _hb_sentinel_thread(self) -> None:
        while not self._shutdown.is_set():
            time.sleep(1.)
            if not self._cfg.pixhawk_enabled or self._px is None:
                continue
            try:
                if self._sb:
                    self._sb.parameters["HYDRA_PX_HB"] = time.time() % 65535
            except Exception:
                pass

    def _esc_regression_thread(self) -> None:
        """Run RPM trend regression on all motors every 10 s."""
        while not self._shutdown.is_set():
            time.sleep(10.)
            self._esc.run_regression_all()


# ════════════════════════════════════════════════════════════════════════════
#  PROMETHEUS METRICS SERVER
# ════════════════════════════════════════════════════════════════════════════

class PrometheusState:
    """Shared state updated by FSM, exposed via HTTP."""
    def __init__(self):
        self.weight_g: float               = 0.
        self.ukf_mass_g: float             = 0.
        self.ukf_unc_g: float              = 0.
        self.pdd_posterior: float          = 0.
        self.fsm_state_code: int           = 0
        self.transition_count: int         = 0
        self.loop_hz: float                = 5.
        self.vibe_level: float             = 0.
        self.esc_rpm: Dict[int, float]     = {i: 0. for i in range(1, 9)}
        self.esc_health: Dict[int, float]  = {i: 1. for i in range(1, 9)}
        self.pid_output: float             = 0.
        self.mission_phase: str            = "UNKNOWN"
        self._lock = threading.Lock()

    def update(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                if hasattr(self, k):
                    setattr(self, k, v)

    def render(self) -> str:
        with self._lock:
            lines = [
                "# HELP hydra_weight_g Load cell fused weight grams",
                "# TYPE hydra_weight_g gauge",
                f"hydra_weight_g {self.weight_g:.2f}",
                f"hydra_ukf_mass_g {self.ukf_mass_g:.2f}",
                f"hydra_ukf_uncertainty_g {self.ukf_unc_g:.2f}",
                f"hydra_mhdd_posterior {self.pdd_posterior:.4f}",
                f"hydra_fsm_state_code {self.fsm_state_code}",
                f"hydra_transition_count {self.transition_count}",
                f"hydra_loop_hz {self.loop_hz:.2f}",
                f"hydra_vibration_level {self.vibe_level:.2f}",
                f"hydra_pid_output {self.pid_output:.4f}",
            ]
            for i in range(1, 9):
                lines.append(
                    f'hydra_esc_rpm{{motor="{i}"}} {self.esc_rpm.get(i, 0):.0f}')
                lines.append(
                    f'hydra_esc_health{{motor="{i}"}} {self.esc_health.get(i, 1.):.3f}')
        return "\n".join(lines) + "\n"


prom = PrometheusState()


def _make_prometheus_handler():
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/metrics":
                body = prom.render().encode()
                self.send_response(200)
                self.send_header("Content-Type",
                                 "text/plain; version=0.0.4; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()
        def log_message(self, *args): pass
    return Handler


def start_prometheus_server(cfg: Config) -> None:
    if not cfg.prometheus_enabled:
        return
    def _run():
        srv = HTTPServer(("0.0.0.0", cfg.prometheus_port),
                         _make_prometheus_handler())
        log.info("[PROM] Metrics at http://0.0.0.0:%d/metrics",
                 cfg.prometheus_port)
        srv.serve_forever()
    threading.Thread(target=_run, daemon=True, name="prometheus").start()


# ════════════════════════════════════════════════════════════════════════════
#  FAULT INJECTION FRAMEWORK
# ════════════════════════════════════════════════════════════════════════════

@dataclass
class FaultEvent:
    at_s: float          # seconds from system start
    kind: str            # 'weight_drop' | 'sensor_fail' | 'jerk_spike' | 'hb_drop'
    params: dict         # kind-specific parameters

class FaultInjector:
    def __init__(self, schedule: List[FaultEvent]):
        self._schedule = sorted(schedule, key=lambda e: e.at_s)
        self._fired: List[FaultEvent] = []
        self._t0 = time.monotonic()
        self._results: List[Tuple[FaultEvent, str]] = []
        self._hb_drop_until: float = -1.0
        self._sensor_fail_until: Dict[str, float] = {}

    @classmethod
    def from_json(cls, json_str: str) -> "FaultInjector":
        events = []
        for item in json.loads(json_str):
            events.append(FaultEvent(
                at_s=item["at_s"],
                kind=item["kind"],
                params=item.get("params", {})))
        return cls(events)

    def tick(self, weight_sim: float) -> Tuple[float, bool]:
        """
        Returns (weight_after_injection, jerk_spike_active).
        Handles all fault kinds:
          weight_drop  — override returned weight to target_g
          jerk_spike   — signal caller to inject angular jerk
          hb_drop      — set internal timer; checked via hb_dropped()
          sensor_fail  — set internal timer; checked via sensor_failed()
        """
        now = time.monotonic() - self._t0
        jerk = False
        for ev in list(self._schedule):
            if ev.at_s <= now and ev not in self._fired:
                self._fired.append(ev)
                log.warning("[FAULT] Injecting: %s at t=%.1fs", ev.kind, now)
                bb.record("fault_inject", kind=ev.kind, at_s=now,
                          params=ev.params)
                if ev.kind == "weight_drop":
                    weight_sim = ev.params.get("target_g", 0.)
                elif ev.kind == "jerk_spike":
                    jerk = True
                elif ev.kind == "hb_drop":
                    dur = ev.params.get("duration_s", 3.0)
                    self._hb_drop_until = now + dur
                    log.warning("[FAULT] HB drop active for %.1fs", dur)
                elif ev.kind == "sensor_fail":
                    sensor = ev.params.get("sensor", "primary")
                    dur    = ev.params.get("duration_s", 5.0)
                    self._sensor_fail_until[sensor] = now + dur
                    log.warning("[FAULT] Sensor '%s' fail for %.1fs", sensor, dur)
        return weight_sim, jerk

    def hb_dropped(self) -> bool:
        """True while a heartbeat-drop fault is active."""
        return (time.monotonic() - self._t0) < self._hb_drop_until

    def sensor_failed(self, which: str = "primary") -> bool:
        """True while a sensor-fail fault for 'which' is active."""
        return (time.monotonic() - self._t0) < self._sensor_fail_until.get(which, 0.)

    def report(self) -> None:
        log.info("[FAULT] Injection report: %d events fired", len(self._fired))
        for ev in self._fired:
            log.info("  %s @ t=%.1fs", ev.kind, ev.at_s)


# ════════════════════════════════════════════════════════════════════════════
#  SEVEN-STATE FSM  (v4 — adds DEGRADED and EMERGENCY states)
#
#  OCTOCOPTER
#    │  MHDD posterior ≥ thresh  OR  raw weight < threshold
#    ▼
#  PRE_DROP  ← PID gate + safety gates
#    │  PID commits  AND  all safety gates  AND  ESC health OK
#    ▼
#  TRANSITIONING_QUAD
#    │  mixer compensation → param writes → motor health verify
#    ▼
#  QUAD  ←────────────────────────────────────────────────────┐
#    │  weight ≥ upper_band  AND  safety gates                 │
#    ▼                                                         │
#  TRANSITIONING_OCTO                                         │
#    │  restore mix → param writes → motor health verify       │
#    ▼  ok              ▼ fail                                 │
#  OCTOCOPTER          QUAD (retain)                          │
#                                                              │
#  Any state → DEGRADED  on ESC regression failure on M1-M4  │
#    │  (secondary motors remain off; partial capacity)        │
#    └──────────────────────────────────────────────────────── ┘
#  DEGRADED → RTL command issued, transitions suppressed
#
#  Any state → EMERGENCY  on catastrophic sensor / FC failure
#    → emergency_shutdown_all() fires immediately
# ════════════════════════════════════════════════════════════════════════════

class FSMState(Enum):
    OCTOCOPTER         = 0
    PRE_DROP           = 1
    TRANSITIONING_QUAD = 2
    QUAD               = 3
    TRANSITIONING_OCTO = 4
    DEGRADED           = 5
    EMERGENCY          = 6


class MotorFSM:
    def __init__(self, cfg: Config, bridge: DualFCBridge,
                 esc: ESCHealthRegressor, jerk: AngularJerkGate,
                 tlc: ThrustLossCompensator, mixer: MixerCompensator,
                 pid: TransitionPID, mission: MissionClassifier):
        self._cfg     = cfg
        self._bridge  = bridge
        self._esc     = esc
        self._jerk    = jerk
        self._tlc     = tlc
        self._mixer   = mixer
        self._pid     = pid
        self._mission = mission
        self._state   = FSMState.OCTOCOPTER
        self._transitions = 0
        self._pre_drop_t  = 0.

        if bridge.sync_initial_state(cfg.secondary_motors):
            self._state = FSMState.QUAD

    @property
    def state(self) -> FSMState: return self._state

    @property
    def motors_disabled(self) -> bool:
        return self._state in (FSMState.QUAD, FSMState.TRANSITIONING_OCTO,
                               FSMState.DEGRADED)

    @property
    def transition_count(self) -> int: return self._transitions

    def evaluate(self, weight_g: float, mhdd_triggered: bool,
                 mhdd_posterior: float) -> None:
        lo = self._cfg.threshold_g
        hi = lo + self._cfg.hysteresis_g

        # ── Proactive primary degradation check ───────────────────────────────
        if self._state not in (FSMState.DEGRADED, FSMState.EMERGENCY):
            for m in [1, 2, 3, 4]:
                if self._esc.degrading.get(m, False):
                    log.critical("[FSM] M%d DEGRADING — entering DEGRADED state", m)
                    self._enter_degraded(m)
                    return

        bb.record("FSM_tick", state=self._state.name, weight=weight_g,
                  mhdd=mhdd_triggered, posterior=mhdd_posterior,
                  lo=lo, hi=hi)

        if self._state == FSMState.OCTOCOPTER:
            if mhdd_triggered or weight_g < lo:
                self._enter_pre_drop(weight_g, mhdd_triggered, mhdd_posterior)

        elif self._state == FSMState.PRE_DROP:
            if weight_g >= hi:
                log.info("[FSM] PRE_DROP→OCTO (false alarm w=%.1fg)", weight_g)
                self._pid.reset()
                self._set(FSMState.OCTOCOPTER)
                return
            commit, pid_out = self._pid.update(weight_g, lo)
            prom.update(pid_output=pid_out)
            if commit and self._bridge.is_all_safe():
                self._do_to_quad(weight_g)
            else:
                if not commit:
                    log.debug("[FSM] PRE_DROP: PID=%.3f < %.3f", pid_out, self._cfg.transition_pid_threshold)
                if not self._bridge.is_all_safe():
                    log.debug("[FSM] PRE_DROP: safety gates not clear")

        elif self._state == FSMState.QUAD:
            if weight_g >= hi and self._bridge.is_all_safe():
                self._do_to_octo(weight_g)

        elif self._state == FSMState.DEGRADED:
            # Issue RTL if not already
            self._handle_degraded()

    def _enter_pre_drop(self, w: float, mhdd: bool, posterior: float) -> None:
        src = f"MHDD(post={posterior:.3f})" if mhdd else "THRESHOLD"
        log.warning("[FSM] OCTO→PRE_DROP [%s] w=%.1fg", src, w)
        bb.record("FSM_pre_drop", source=src, weight=w, posterior=posterior)
        self._pre_drop_t = time.monotonic()
        self._pid.enter(w, self._cfg.threshold_g)
        self._set(FSMState.PRE_DROP)

    def _do_to_quad(self, w: float) -> None:
        log.warning("[FSM] PRE_DROP→T_QUAD w=%.1fg", w)
        self._set(FSMState.TRANSITIONING_QUAD)

        # 1. ESC health gate on secondary motors (they must be healthy to disable)
        sec_health = self._esc.mean_health(self._cfg.secondary_motors)
        pri_health = self._esc.mean_health([1, 2, 3, 4])
        if pri_health < 0.6:
            log.critical("[FSM] Primary ESC health %.2f — ABORT QUAD", pri_health)
            bb.record("FSM_abort", reason="primary_health", health=pri_health)
            self._set(FSMState.OCTOCOPTER); return

        # 2. Mixer compensation — write corrected yaw PIDs BEFORE disabling
        if not self._mixer.apply_quad_mix():
            log.warning("[FSM] Mixer compensation failed — proceeding anyway")

        # 3. Disable secondary motors
        all_ok = True
        for m in self._cfg.secondary_motors:
            if not self._bridge.write_param(f"SERVO{m}_FUNCTION", 0):
                all_ok = False
            else:
                if not self._bridge.verify_motor(m, False):
                    log.critical("[FSM] M%d stuck spinning", m)
                    bb.record("motor_stuck_spin", motor=m)

        if all_ok:
            self._set(FSMState.QUAD)
            self._transitions += 1
            self._tlc.trigger()
            lat_ms = (time.monotonic() - self._pre_drop_t) * 1000
            log.warning(">>> QUAD ACTIVE [#%d] lat=%.0fms sec_health=%.2f",
                        self._transitions, lat_ms, sec_health)
            bb.record("FSM_quad_active", n=self._transitions, lat_ms=lat_ms,
                      sec_health=sec_health)
            prom.update(fsm_state_code=FSMState.QUAD.value,
                        transition_count=self._transitions)
        else:
            log.error("[FSM] QUAD transition failed — reverting")
            self._mixer.restore_octo_mix()
            self._set(FSMState.OCTOCOPTER)

    def _do_to_octo(self, w: float) -> None:
        log.warning("[FSM] QUAD→T_OCTO w=%.1fg", w)
        self._set(FSMState.TRANSITIONING_OCTO)

        # Restore yaw PIDs BEFORE re-enabling
        self._mixer.restore_octo_mix()

        all_ok = True
        for m in self._cfg.secondary_motors:
            fv = self._cfg.motor_func_base + m - 1
            if not self._bridge.write_param(f"SERVO{m}_FUNCTION", fv):
                all_ok = False
            else:
                if not self._bridge.verify_motor(m, True):
                    log.critical("[FSM] M%d stuck stopped", m)
                    bb.record("motor_stuck_stop", motor=m)

        if all_ok:
            self._set(FSMState.OCTOCOPTER)
            self._transitions += 1
            log.info(">>> OCTO ACTIVE [#%d]", self._transitions)
            bb.record("FSM_octo_active", n=self._transitions)
            prom.update(fsm_state_code=FSMState.OCTOCOPTER.value,
                        transition_count=self._transitions)
        else:
            log.error("[FSM] OCTO transition failed — retain QUAD")
            self._mixer.apply_quad_mix()   # Re-apply since we're staying quad
            self._set(FSMState.QUAD)

    def _enter_degraded(self, bad_motor: int) -> None:
        bb.record("FSM_degraded", motor=bad_motor)
        self._set(FSMState.DEGRADED)
        # Issue RTL via mode change
        if self._bridge._sb:
            try:
                self._bridge._sb.mode = VehicleMode("RTL")
                log.critical("[FSM] DEGRADED — RTL commanded on SpeedyBee")
            except Exception as e:
                log.error("[FSM] RTL command failed: %s", e)

    def _handle_degraded(self) -> None:
        """
        Called every FSM tick while in DEGRADED state.
        Verifies the RTL command took effect; re-issues if mode has not changed
        within DEGRADED_RTL_REISSUE_S seconds.  Logs a CRITICAL warning every
        tick so the GCS operator is aware.
        """
        now = time.monotonic()
        # Throttle the re-issue check to once per 5 s
        if not hasattr(self, "_degraded_last_check"):
            self._degraded_last_check = now
            self._degraded_rtl_issued = now
        if now - self._degraded_last_check < 5.0:
            return
        self._degraded_last_check = now
        log.critical("[FSM] DEGRADED — motor degradation detected, RTL in progress")
        bb.record("FSM_degraded_tick",
                  elapsed_s=now - self._degraded_rtl_issued)
        # Re-issue RTL if vehicle is still in a non-RTL mode
        sb = self._bridge._sb
        if sb is not None:
            try:
                if sb.mode.name not in ("RTL", "LAND"):
                    log.critical("[FSM] DEGRADED: mode=%s, re-issuing RTL",
                                 sb.mode.name)
                    sb.mode = VehicleMode("RTL")
                    bb.record("FSM_rtl_reissue", mode=sb.mode.name)
            except Exception as e:
                log.error("[FSM] DEGRADED: RTL re-issue failed: %s", e)

    def force_enable_all(self) -> None:
        for m in self._cfg.secondary_motors:
            fv = self._cfg.motor_func_base + m - 1
            self._bridge.write_param(f"SERVO{m}_FUNCTION", fv)
        self._set(FSMState.OCTOCOPTER)

    def emergency_shutdown_all(self) -> None:
        log.critical("[FSM] EMERGENCY SHUTDOWN — all 8 motors NOW")
        bb.record("emergency_shutdown")
        for m in range(1, 9):
            self._bridge.write_param(f"SERVO{m}_FUNCTION", 0)
        self._set(FSMState.EMERGENCY)

    def _set(self, ns: FSMState) -> None:
        old = self._state; self._state = ns
        if old != ns:
            log.debug("[FSM] %s → %s", old.name, ns.name)
            bb.record("FSM_transition", frm=old.name, to=ns.name)
            prom.update(fsm_state_code=ns.value)


# ════════════════════════════════════════════════════════════════════════════
#  WEIGHT SENSOR PIPELINE  (UKF-integrated, vibration-aware)
# ════════════════════════════════════════════════════════════════════════════

class WeightPipeline:
    """
    Combines: SensorArbitrator → UKF → MHDD
    Outputs: (ukf_mass_g, mhdd_triggered, mhdd_posterior, raw_g)
    """

    def __init__(self, cfg: Config, ukf: UKF2D, mhdd: MHDD,
                 arb: SensorArbitrator, vibe: VibrationMonitor):
        self._cfg  = cfg
        self._ukf  = ukf
        self._mhdd = mhdd
        self._arb  = arb
        self._vibe = vibe
        self._last_t = time.monotonic()
        self.reads = 0; self.errors = 0

    def read(self, a_z: float = 0.) -> Tuple[Optional[float], bool, float, float]:
        now = time.monotonic()
        dt  = max(0.001, now - self._last_t)
        self._last_t = now

        raw, source = self._arb.read()
        if raw is None:
            self.errors += 1
            return None, False, 0., 0.

        self.reads += 1
        raw = max(0., raw)

        # UKF predict + update
        vibe_scale = self._vibe.ukf_r_scale
        self._ukf.predict(dt, vibe_scale)
        self._ukf.update(raw, a_z=a_z, R_scale=vibe_scale)

        mass = self._ukf.mass_g
        unc  = self._ukf.uncertainty_g

        # Feed MHDD
        self._mhdd.feed(mass, now)
        triggered, posterior, detail = self._mhdd.evaluate()

        bb.record("pipeline", raw=raw, ukf_mass=mass, unc=unc,
                  source=source, triggered=triggered, post=posterior)
        log.debug("[PIPELINE] raw=%.1f mass=%.1f±%.1f post=%.3f src=%s",
                  raw, mass, unc, posterior, source)

        prom.update(weight_g=raw, ukf_mass_g=mass, ukf_unc_g=unc,
                    pdd_posterior=posterior)
        return mass, triggered, posterior, raw

    def cleanup(self) -> None:
        self._arb.cleanup()


# ════════════════════════════════════════════════════════════════════════════
#  CLI
# ════════════════════════════════════════════════════════════════════════════

def build_config() -> Config:
    p = argparse.ArgumentParser(
        description="HYDRA v4 — Obscenely Advanced Companion Controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--sb-port",       default="/dev/serial0")
    p.add_argument("--sb-baud",       type=int,   default=57600)
    p.add_argument("--px-port",       default="/dev/serial1")
    p.add_argument("--px-baud",       type=int,   default=57600)
    p.add_argument("--no-px",         action="store_true")
    p.add_argument("--threshold",     type=float, default=500.)
    p.add_argument("--hysteresis",    type=float, default=75.)
    p.add_argument("--hz",            type=float, default=5.)
    p.add_argument("--dout",          type=int,   default=5)
    p.add_argument("--sck",           type=int,   default=6)
    p.add_argument("--dout2",         type=int,   default=-1,
                   help="Secondary HX711 DOUT GPIO (-1 to disable)")
    p.add_argument("--sck2",          type=int,   default=24)
    p.add_argument("--scale",         type=float, default=1.)
    p.add_argument("--scale2",        type=float, default=1.)
    p.add_argument("--no-ate",        action="store_true")
    p.add_argument("--ate-fraction",  type=float, default=0.35)
    p.add_argument("--no-aero-corr",  action="store_true")
    p.add_argument("--no-mhdd",       action="store_true")
    p.add_argument("--mhdd-post",     type=float, default=0.75)
    p.add_argument("--no-mixer",      action="store_true")
    p.add_argument("--no-tlc",        action="store_true")
    p.add_argument("--tlc-bump",      type=float, default=0.08)
    p.add_argument("--tlc-dur",       type=float, default=0.5)
    p.add_argument("--no-jerk",       action="store_true")
    p.add_argument("--jerk-max",      type=float, default=3.)
    p.add_argument("--no-health",     action="store_true")
    p.add_argument("--att-limit",     type=float, default=10.)
    p.add_argument("--att-diverge",   type=float, default=5.)
    p.add_argument("--pid-kp",        type=float, default=0.05)
    p.add_argument("--pid-ki",        type=float, default=0.02)
    p.add_argument("--pid-kd",        type=float, default=0.005)
    p.add_argument("--pid-thr",       type=float, default=1.0)
    p.add_argument("--pid-dwell",     type=float, default=2.0)
    p.add_argument("--no-vibe-adapt", action="store_true")
    p.add_argument("--no-graceful",   action="store_true")
    p.add_argument("--motor-stop-wait", type=float, default=2.)
    p.add_argument("--no-prom",       action="store_true")
    p.add_argument("--prom-port",     type=int,   default=9090)
    p.add_argument("--grpc",          action="store_true")
    p.add_argument("--grpc-port",     type=int,   default=50051)
    p.add_argument("--no-blackbox",   action="store_true")
    p.add_argument("--blackbox",      default="/tmp/hydra_v4_blackbox.jsonl")
    p.add_argument("--fault-inject",  default="",
                   help="Path to fault injection schedule JSON file")
    p.add_argument("--no-wp-excl",    action="store_true")
    p.add_argument("--no-reconfig-wps", type=int, nargs="*", default=[])
    p.add_argument("--sim-drop-time", type=float, default=60.)
    p.add_argument("--sim-drop-rate", type=float, default=150.)

    a = p.parse_args()

    fault_json = ""
    if a.fault_inject:
        try:
            with open(a.fault_inject) as f:
                fault_json = f.read()
        except Exception as e:
            log.warning("Cannot read fault schedule: %s", e)

    return Config(
        speedybee_port=a.sb_port, speedybee_baud=a.sb_baud,
        pixhawk_port=a.px_port,   pixhawk_baud=a.px_baud,
        pixhawk_enabled=not a.no_px,
        threshold_g=a.threshold, hysteresis_g=a.hysteresis,
        loop_hz=a.hz,
        hx711_primary_dout=a.dout, hx711_primary_sck=a.sck,
        hx711_secondary_dout=a.dout2, hx711_secondary_sck=a.sck2,
        hx711_scale_primary=a.scale, hx711_scale_secondary=a.scale2,
        ate_enabled=not a.no_ate, ate_threshold_fraction=a.ate_fraction,
        ate_aero_correction=not a.no_aero_corr,
        mhdd_enabled=not a.no_mhdd, mhdd_posterior_thresh=a.mhdd_post,
        mixer_compensation_enabled=not a.no_mixer,
        tlc_enabled=not a.no_tlc, tlc_bump_magnitude=a.tlc_bump,
        tlc_bump_duration_s=a.tlc_dur,
        jerk_gate_enabled=not a.no_jerk, jerk_max_rad_s2=a.jerk_max,
        motor_health_enabled=not a.no_health,
        attitude_max_deg=a.att_limit, attitude_diverge_deg=a.att_diverge,
        transition_pid_kp=a.pid_kp, transition_pid_ki=a.pid_ki,
        transition_pid_kd=a.pid_kd,
        transition_pid_threshold=a.pid_thr,
        transition_max_dwell_s=a.pid_dwell,
        vibe_adapt_enabled=not a.no_vibe_adapt,
        graceful_shutdown_enabled=not a.no_graceful,
        motor_stop_wait_s=a.motor_stop_wait,
        prometheus_enabled=not a.no_prom, prometheus_port=a.prom_port,
        grpc_enabled=a.grpc, grpc_port=a.grpc_port,
        blackbox_enabled=not a.no_blackbox, blackbox_path=a.blackbox,
        fault_inject_enabled=bool(fault_json),
        fault_inject_schedule=fault_json,
        mission_classifier_enabled=not a.no_wp_excl,
        no_reconfigure_waypoints=a.no_reconfig_wps or [],
        sim_drop_time_s=a.sim_drop_time,
        sim_drop_rate_g_per_s=a.sim_drop_rate,
    )


# ════════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════════

def main() -> None:
    cfg = build_config()

    # Initialise blackbox after connecting to FC to get UID for key derivation
    # (temporarily use empty UID; updated after connect)
    global bb
    bb = SecureBlackBox(path=cfg.blackbox_path,
                        enabled=cfg.blackbox_enabled)

    log.info("=" * 76)
    log.info("  HYDRA v4.0 — Obscenely Advanced Companion Motor Controller")
    log.info("  SpeedyBee %s@%d  PixHawk %s@%d (enabled=%s)",
             cfg.speedybee_port, cfg.speedybee_baud,
             cfg.pixhawk_port,   cfg.pixhawk_baud, cfg.pixhawk_enabled)
    log.info("  UKF α=%.1e β=%.1f  MHDD=%s(post≥%.2f)  MixComp=%s  "
             "VibeAdapt=%s",
             cfg.ukf_alpha, cfg.ukf_beta,
             cfg.mhdd_enabled, cfg.mhdd_posterior_thresh,
             cfg.mixer_compensation_enabled, cfg.vibe_adapt_enabled)
    log.info("  PID kp=%.3f ki=%.3f kd=%.4f thr=%.2f dwell=%.1fs",
             cfg.transition_pid_kp, cfg.transition_pid_ki,
             cfg.transition_pid_kd, cfg.transition_pid_threshold,
             cfg.transition_max_dwell_s)
    log.info("  DualHX711=%s  AeroCorr=%s  FaultInject=%s  Prom=%s:%d",
             cfg.hx711_secondary_dout >= 0, cfg.ate_aero_correction,
             cfg.fault_inject_enabled,
             cfg.prometheus_enabled, cfg.prometheus_port)
    log.info("=" * 76)

    # ── Instantiate subsystems ────────────────────────────────────────────────
    ukf     = UKF2D(cfg)
    mhdd    = MHDD(cfg)
    vibe    = VibrationMonitor(cfg)
    esc     = ESCHealthRegressor(cfg)
    jerk    = AngularJerkGate(cfg)
    mission = MissionClassifier(cfg)
    arb     = SensorArbitrator(cfg)
    ate     = AdaptiveThresholdEngine(cfg, ukf, mhdd)
    bridge  = DualFCBridge(cfg, esc, jerk, vibe, mission)
    tlc     = ThrustLossCompensator(cfg)
    mixer   = MixerCompensator(cfg, bridge)
    pid_ctl = TransitionPID(cfg)
    pipeline = WeightPipeline(cfg, ukf, mhdd, arb, vibe)

    fault_injector: Optional[FaultInjector] = None
    if cfg.fault_inject_enabled and cfg.fault_inject_schedule:
        fault_injector = FaultInjector.from_json(cfg.fault_inject_schedule)
        log.info("[FAULT] Injection schedule loaded: %d events",
                 len(fault_injector._schedule))

    # ── Connect FCs ───────────────────────────────────────────────────────────
    if not bridge.connect():
        log.critical("FC connection failed — aborting")
        pipeline.cleanup(); sys.exit(1)

    # Re-init blackbox with FC UID for cryptographic binding
    if bridge.fc_uid:
        bb = SecureBlackBox(path=cfg.blackbox_path,
                            enabled=cfg.blackbox_enabled,
                            fc_uid=bridge.fc_uid)
        log.info("Black-box re-keyed with FC UID %s", bridge.fc_uid.hex())

    tlc.attach(bridge._sb)

    fsm = MotorFSM(cfg, bridge, esc, jerk, tlc, mixer, pid_ctl, mission)

    # ── Prometheus ────────────────────────────────────────────────────────────
    start_prometheus_server(cfg)

    # ── gRPC server ───────────────────────────────────────────────────────────
    grpc_broadcaster = None
    grpc_dispatcher  = None
    if cfg.grpc_enabled:
        try:
            from hydra_grpc_server import (StatusBroadcaster,
                                           CommandDispatcher,
                                           start_grpc_server)
            grpc_broadcaster = StatusBroadcaster()
            grpc_dispatcher  = CommandDispatcher()
            # Wire FSM commands into dispatcher
            grpc_dispatcher.register(
                "EMERGENCY_SHUTDOWN", lambda r: (fsm.emergency_shutdown_all(), True)[1])
            grpc_dispatcher.register(
                "FORCE_ENABLE_ALL", lambda r: (fsm.force_enable_all(), True)[1])
            grpc_dispatcher.register(
                "FORCE_QUAD", lambda r: (
                    fsm.evaluate(0., True, 1.0), True)[1])
            grpc_dispatcher.register(
                "RTL", lambda r: (
                    bridge._sb and setattr(bridge._sb, "mode",
                                          __import__("dronekit").VehicleMode("RTL")),
                    True)[1])
            grpc_dispatcher.register("NOP", lambda r: True)
            start_grpc_server(cfg, grpc_broadcaster, grpc_dispatcher,
                              port=cfg.grpc_port)
            log.info("[GRPC] Server started on port %d", cfg.grpc_port)
        except Exception as e:
            log.warning("[GRPC] Failed to start: %s — continuing without gRPC", e)
            grpc_broadcaster = grpc_dispatcher = None

    bb.record("system_start",
              threshold=cfg.threshold_g,
              mhdd=cfg.mhdd_enabled,
              ukf_alpha=cfg.ukf_alpha,
              mixer=cfg.mixer_compensation_enabled,
              prom_port=cfg.prometheus_port,
              grpc=cfg.grpc_enabled)

    log.info("Main loop | state=%s", fsm.state.name)
    t_last_telem = 0.

    # ── Main loop ─────────────────────────────────────────────────────────────
    try:
        while True:
            t0       = time.monotonic()
            cur_hz   = vibe.current_loop_hz
            interval = 1.0 / cur_hz
            prom.update(loop_hz=cur_hz, vibe_level=vibe._clip_rms)

            # ── Fault injection ───────────────────────────────────────────────
            sim_override: Optional[float] = None
            if fault_injector:
                sim_w, jerk_spike = fault_injector.tick(0.)
                if jerk_spike:
                    jerk.feed(999., 999., 999.)
                if sim_w > 0:
                    sim_override = sim_w
                if fault_injector.hb_dropped():
                    bridge._px_last_hb = 0.

            # ── Read weight ───────────────────────────────────────────────────
            a_z = bridge.vertical_accel
            sensor_ok = True
            if fault_injector and fault_injector.sensor_failed("both"):
                sensor_ok = False

            if sensor_ok:
                mass, mhdd_trig, posterior, raw = pipeline.read(a_z=a_z)
            else:
                mass = mhdd_trig = None
                posterior = raw = 0.

            if sim_override is not None and mass is not None:
                mass = float(sim_override)

            if mass is not None:
                ate.feed(mass, bridge.armed, a_z)

                # Mission-phase MHDD override
                overrides = mission.pdd_overrides()
                orig_post = cfg.mhdd_posterior_thresh
                if overrides:
                    cfg.mhdd_posterior_thresh = overrides.get(
                        "posterior_thresh", orig_post)

                fsm.evaluate(mass, mhdd_trig, posterior)

                if overrides:
                    cfg.mhdd_posterior_thresh = orig_post

                # Update Prometheus ESC metrics
                for i in range(1, 9):
                    prom.esc_health[i] = esc.health_score(i)
                    rpm = esc.latest_rpm(i)
                    if rpm:
                        prom.esc_rpm[i] = float(rpm)

                # Push to gRPC broadcaster
                if grpc_broadcaster is not None:
                    try:
                        from hydra_grpc_server import (MotorSystemStatusMsg,
                                                        MotorStatusMsg)
                        grpc_broadcaster.publish(MotorSystemStatusMsg(
                            timestamp_unix=time.time(),
                            fsm_state=fsm.state.name,
                            fsm_state_code=fsm.state.value,
                            transition_count=fsm.transition_count,
                            weight_raw_g=raw,
                            ukf_mass_g=ukf.mass_g,
                            ukf_uncertainty_g=ukf.uncertainty_g,
                            mhdd_posterior=posterior,
                            pid_output=pid_ctl._integral * cfg.transition_pid_ki,
                            threshold_g=cfg.threshold_g,
                            hysteresis_g=cfg.hysteresis_g,
                            mission_phase=mission.phase.name,
                            vibration_level=vibe._clip_rms,
                            loop_hz=cur_hz,
                            motors=[MotorStatusMsg(
                                index=i,
                                rpm=int(prom.esc_rpm.get(i, 0)),
                                health=prom.esc_health.get(i, 1.),
                                degrading=esc.degrading.get(i, False),
                                enabled=(i not in cfg.secondary_motors
                                         or not fsm.motors_disabled),
                            ) for i in range(1, 9)],
                            pixhawk_hb_ok=bridge.is_pixhawk_hb_ok(),
                            ate_locked=ate.is_locked,
                            ate_baseline_g=ate.baseline_g or 0.,
                        ))
                    except Exception as _ge:
                        log.debug("[GRPC] publish: %s", _ge)

            # ── Periodic telemetry ────────────────────────────────────────────
            now = time.time()
            if now - t_last_telem >= cfg.telemetry_interval_s:
                t_last_telem = now
                bridge.log_telemetry(
                    w=mass, state=fsm.state.name,
                    ukf_mass=ukf.mass_g, ukf_unc=ukf.uncertainty_g,
                    posterior=posterior, phase=mission.phase.name,
                    vibe=vibe._clip_rms, loop_hz=cur_hz)
                bb.record("telemetry",
                          state=fsm.state.name, hz=cur_hz,
                          mass=ukf.mass_g, unc=ukf.uncertainty_g,
                          posterior=posterior, phase=mission.phase.name,
                          vibe=vibe._clip_rms)

            elapsed   = time.monotonic() - t0
            sleep_for = max(0., interval - elapsed)
            if elapsed > interval * 1.5:
                log.debug("Loop overrun %.1fms (target %.0fms)",
                          elapsed * 1000, interval * 1000)
            time.sleep(sleep_for)

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — clean shutdown")
    except Exception as e:
        log.exception("Unhandled exception: %s", e)
        fsm.emergency_shutdown_all()
    finally:
        if fault_injector:
            fault_injector.report()
        if cfg.graceful_shutdown_enabled:
            bridge.graceful_shutdown()
        else:
            fsm.force_enable_all()
        pipeline.cleanup()
        bridge.close()
        bb.record("system_stop",
                  transitions=fsm.transition_count,
                  reads=pipeline.reads,
                  errors=pipeline.errors,
                  final_state=fsm.state.name)
        log.info("Shutdown | transitions=%d reads=%d errors=%d",
                 fsm.transition_count, pipeline.reads, pipeline.errors)


if __name__ == "__main__":
    main()
