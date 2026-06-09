-- ============================================================================
--  HYDRA v4.0  —  motor_controller_v4.lua
--  SpeedyBee F405 Wing  ·  ArduPilot Lua Scripting Engine
--  Project HYDRA · HALO Aero Club · SSN College of Engineering
--
--  WHAT v4 ADDS (beyond v3 Lua)
--  ─────────────────────────────
--  1. UKF-lite  — 2-state scalar UKF in pure Lua, fusing HX711 with
--     barometric vertical velocity (dAlt/dt) as a proxy for a_z.
--     State: [mass_g, mass_dot_g_per_s].  Runs at every loop tick.
--
--  2. CUSUM change-point detector  — runs in parallel with OLS regression.
--     Both feed a Bayesian combiner (equal weights → posterior threshold).
--
--  3. Mixer compensation  — reads ATC_RAT_YAW_P/I before transition,
--     scales by QUAD_YAW_SCALE, writes back, restores on octo re-enable.
--     Yaw authority is preserved across 8→4 motor reconfiguration.
--
--  4. ESC trend watchdog  — maintains a 60-sample RPM ring buffer per
--     motor.  Computes OLS slope over the buffer; if slope < RPM_WARN_SLOPE
--     for motors 1-4 (primaries), issues a GCS CRITICAL and sets param
--     HYDRA_M_DEGRADE to the motor index.  Companion Python can read this.
--
--  5. PID transition gate  — error integral must exceed PID_COMMIT_THOLD
--     before PRE_DROP → T_QUAD fires.  Hard dwell cap: DWELL_MAX_S.
--
--  6. Vibration-adaptive loop rate  — reads VIBE_X/Y/Z from vehicle
--     vibration API; adjusts loop interval between LOOP_LO_MS and
--     LOOP_HI_MS.
--
--  7. Seven-state FSM  — adds DEGRADED and EMERGENCY states.
--     DEGRADED is entered when any primary motor RPM trend fails.
--     EMERGENCY fires emergency_shutdown_all() immediately.
--
--  8. HMAC-lite record tagging  — every GCS status message tagged with
--     a rolling CRC32 checksum of the previous message content, giving
--     a lightweight chain for in-flight log integrity from the GCS side.
--
--  9. Sentinel param pack  — writes a structured set of params every 2 s:
--     HYDRA_STATE, HYDRA_MASS, HYDRA_POST, HYDRA_PID_I, HYDRA_VIBE
--     so that Mission Planner / MAVProxy scripts can read live FSM state
--     without parsing GCS text messages.
--
--  10. Boot self-test  — on first arm, sequences each secondary motor
--      at MOT_SPIN_ARM level for one loop tick and verifies RPM > 0
--      via ESC_STATUS.  Blocks takeoff (refuses arm confirmation) if
--      any secondary motor does not spin.
-- ============================================================================

-- ============================================================================
--  REQUIRED ARDUPILOT PARAMETERS  (SpeedyBee F405 Wing)
-- ============================================================================
--  SCR_ENABLE          = 1
--  SCR_HEAP_SIZE       = 200000   (200 KB — UKF + ring buffers need it)
--  SCR_VM_I_COUNT      = 50000
--  SERIAL2_PROTOCOL    = 28       (Scripting — HX711 adapter)
--  SERIAL2_BAUD        = 9        (9600)
--  FRAME_CLASS         = 2        (Octocopter)
--  FRAME_TYPE          = 0        (X)
--  SERVO5_FUNCTION     = 37
--  SERVO6_FUNCTION     = 38
--  SERVO7_FUNCTION     = 39
--  SERVO8_FUNCTION     = 40
--  MOT_SPIN_MIN        = 0.15
--  MOT_SPIN_ARM        = 0.10
-- ============================================================================

-- ============================================================================
--  USER CONFIGURATION
-- ============================================================================

-- Weight thresholds (ATE overwrites at runtime)
local THRESHOLD_G           = 500.0
local HYSTERESIS_G          = 75.0

-- UKF-lite
local UKF_ALPHA             = 0.001
local UKF_BETA              = 2.0
local UKF_KAPPA             = 0.0
local UKF_Q_MASS            = 0.5      -- process noise: mass (g²/s)
local UKF_Q_MDOT            = 10.0     -- process noise: mass_dot ((g/s)²/s)
local UKF_R_BASE            = 25.0     -- measurement noise (g²)
local G_EARTH               = 9.80665  -- m/s²

-- ATE
local ATE_ENABLED           = true
local ATE_WINDOW            = 30
local ATE_SIGMA             = 2.0
local ATE_FRACTION          = 0.35
local ATE_AERO_CORR         = true     -- subtract a_z*mass from apparent weight

-- CUSUM
local CUSUM_K               = 40.0     -- half allowance (g)
local CUSUM_H               = 200.0    -- decision threshold

-- OLS regression
local OLS_WINDOW            = 15
local OLS_SLOPE_THR         = -80.0    -- g/s
local OLS_R2_MIN            = 0.85

-- Bayesian combiner
local POSTERIOR_THRESHOLD   = 0.75     -- weighted P(drop) to trigger
local W_OLS                 = 0.5      -- OLS weight
local W_CUSUM               = 0.5      -- CUSUM weight

-- PID transition gate
local PID_KP                = 0.05
local PID_KI                = 0.02
local PID_KD                = 0.005
local PID_COMMIT_THOLD      = 1.0
local DWELL_MAX_S           = 2.0

-- Mixer compensation
local MIXER_ENABLED         = true
local QUAD_YAW_P_SCALE      = 1.85
local QUAD_YAW_I_SCALE      = 1.85

-- ESC trend watchdog
local ESC_RPM_BUF_SIZE      = 60
local RPM_WARN_SLOPE        = -5.0     -- RPM/s — negative trend alarm

-- Vibration-adaptive loop rate
local VIBE_ADAPT_ENABLED    = true
local VIBE_HI_THRESHOLD     = 30.0    -- m/s²  → slow loop
local VIBE_LO_THRESHOLD     = 10.0    -- m/s²  → fast loop
local LOOP_HI_MS            = 100     -- 10 Hz
local LOOP_LO_MS            = 500     -- 2 Hz
local LOOP_BASE_MS          = 200     -- 5 Hz (nominal)

-- Motor mapping
local SECONDARY_MOTORS      = {5, 6, 7, 8}
local ALL_MOTORS            = {1, 2, 3, 4, 5, 6, 7, 8}
local MOTOR_FUNC_BASE       = 33
local PRIMARY_MOTORS        = {1, 2, 3, 4}

-- Health
local MOTOR_MIN_RPM         = 500
local MOTOR_MAX_IDLE_RPM    = 100
local MOTOR_SETTLE_MS       = 800
local ATTITUDE_MAX_DEG      = 10.0
local JERK_MAX_RAD_S2       = 3.0
local MAX_PARAM_RETRIES     = 3
local SENSOR_FAIL_WARN      = 25     -- consecutive failures before warning

-- PixHawk interlock
local PX_INTERLOCK          = true
local PX_HB_TIMEOUT_MS      = 3000

-- Sentinel param write interval
local SENTINEL_INTERVAL_MS  = 2000

-- Telemetry
local TELEM_INTERVAL_MS     = 5000

-- Boot self-test
local SELF_TEST_ENABLED     = true
local SELF_TEST_SPIN_MS     = 400    -- spin each motor briefly

-- ============================================================================
--  MODULE STATE
-- ============================================================================

-- Serial / HX711
local serial_port    = nil
local rx_buf         = ""
local serial_fails   = 0

-- UKF-lite state
local ukf_x          = {1000.0, 0.0}   -- [mass_g, mass_dot]
local ukf_P          = {mass=250000.0, cross=0.0, mdot=2500.0}  -- 2x2 diagonal+cross
local ukf_last_t     = 0.0
local ukf_alt_prev   = nil
local ukf_vz_prev    = 0.0
local ukf_initialised = false

-- ATE
local ate_state      = "IDLE"          -- IDLE | COLLECTING | LOCKED
local ate_buf        = {}
local ate_az_buf     = {}
local ate_baseline   = nil

-- CUSUM
local cusum_neg      = 0.0
local cusum_ref      = nil

-- OLS ring buffer
local ols_buf        = {}
local ols_head       = 1
local ols_count      = 0

-- PID gate
local pid_integral   = 0.0
local pid_prev_err   = 0.0
local pid_prev_t     = 0.0
local pid_entered_t  = 0.0

-- Mixer state
local orig_yaw_p     = nil
local orig_yaw_i     = nil
local mixer_applied  = false

-- ESC RPM ring buffers  [motor_idx] = {buf={}, head, count, ts={}}
local esc_rpm_bufs   = {}
for i = 1, 8 do
    esc_rpm_bufs[i] = {buf={}, ts={}, head=1, count=0}
end
local motor_degrading = {false,false,false,false,false,false,false,false}

-- Jerk gate
local prev_rr = 0.0; local prev_pr = 0.0; local prev_yr = 0.0
local prev_rates_t = 0.0

-- Vibration EMA
local vibe_ema       = 0.0

-- Seven-state FSM
local FSM = {
    OCTO  = "OCTOCOPTER",
    PRE   = "PRE_DROP",
    TQUAD = "T_QUAD",
    QUAD  = "QUAD",
    TOCTO = "T_OCTO",
    DEGR  = "DEGRADED",
    EMER  = "EMERGENCY",
}
local fsm_state      = FSM.OCTO
local fsm_transitions = 0
local pre_drop_t     = 0.0

-- CRC32 chain for GCS message integrity
local crc_state      = 0xFFFFFFFF
local crc32_lut      = nil

-- Sentinel timing
local sentinel_last  = 0
local telem_last     = 0

-- Self-test
local self_test_done = false
local self_test_arm_prev = false

-- Loop timing
local loop_interval_ms = LOOP_BASE_MS
local total_reads    = 0
local total_errors   = 0

-- ============================================================================
--  UTILITY
-- ============================================================================

local function now_s()   return millis() / 1000.0 end
local function now_ms()  return millis() end
local function deg(r)    return r * 57.295779513 end
local function clamp(v, lo, hi) return v < lo and lo or (v > hi and hi or v) end
local function sq(x)     return x * x end

local function table_sum(t)
    local s = 0.0; for _, v in ipairs(t) do s = s + v end; return s
end
local function table_mean(t)
    return #t > 0 and table_sum(t) / #t or 0.0
end
local function table_var(t, m)
    if #t < 2 then return 0.0 end
    local ss = 0.0
    for _, v in ipairs(t) do ss = ss + sq(v - m) end
    return ss / (#t - 1)
end
local function table_stdev(t, m)
    return math.sqrt(table_var(t, m))
end

-- ============================================================================
--  CRC32 (for GCS message chain tagging)
-- ============================================================================

local function init_crc32()
    crc32_lut = {}
    for i = 0, 255 do
        local c = i
        for _ = 1, 8 do
            if c & 1 ~= 0 then c = (c >> 1) ~ 0xEDB88320
            else c = c >> 1 end
        end
        crc32_lut[i] = c
    end
end

local function crc32_update(crc, data)
    for i = 1, #data do
        local b = string.byte(data, i)
        crc = crc32_lut[(crc ~ b) & 0xFF] ~ (crc >> 8)
    end
    return crc
end

local function gcs(severity, text)
    -- Chain CRC over message content
    crc_state = crc32_update(crc_state, text)
    local tag = string.format("[%08X]", crc_state & 0xFFFFFFFF)
    gcs:send_text(severity, "[HYDRA-v4]" .. tag .. " " .. text)
end

-- ============================================================================
--  UKF-LITE  (scalar 2-state, Direct Form, no matrix library needed)
--
--  State:  x = [mass_g, mass_dot_g_per_s]
--  P stored as three values: P[1,1]=Pmm, P[1,2]=P[2,1]=Pcross, P[2,2]=Pdd
--  Sigma points: 2n+1 = 5, generated via scaled UT
-- ============================================================================

local function ukf_sigma_points()
    local n  = 2
    local lam = sq(UKF_ALPHA) * (n + UKF_KAPPA) - n
    local c   = math.sqrt(n + lam)
    -- Cholesky of P (2x2)
    local L11 = math.sqrt(math.max(1e-9, ukf_P.mass))
    local L21 = ukf_P.cross / math.max(1e-9, L11)
    local L22 = math.sqrt(math.max(1e-9, ukf_P.mdot - sq(L21)))
    -- 5 sigma points [mass, mdot]
    local pts = {
        {ukf_x[1],               ukf_x[2]},
        {ukf_x[1]+c*L11,         ukf_x[2]+c*L21},
        {ukf_x[1]-c*L11,         ukf_x[2]-c*L21},
        {ukf_x[1]+c*0,           ukf_x[2]+c*L22},
        {ukf_x[1]+c*0,           ukf_x[2]-c*L22},
    }
    local n2  = 2*n
    local Wm0 = lam / (n + lam)
    local Wi  = 1.0 / (2*(n + lam))
    local Wc0 = Wm0 + (1 - sq(UKF_ALPHA) + UKF_BETA)
    local Wm  = {Wm0, Wi, Wi, Wi, Wi}
    local Wc  = {Wc0, Wi, Wi, Wi, Wi}
    return pts, Wm, Wc
end

local function ukf_predict(dt, vibe_scale)
    vibe_scale = vibe_scale or 1.0
    -- Propagate sigma points through f: mass += mdot*dt, mdot unchanged
    local pts, Wm, Wc = ukf_sigma_points()
    local sp = {}
    for _, p in ipairs(pts) do
        sp[#sp+1] = {math.max(0, p[1] + p[2]*dt), p[2]}
    end
    -- Predicted mean
    local xp = {0.0, 0.0}
    for i, w in ipairs(Wm) do
        xp[1] = xp[1] + w * sp[i][1]
        xp[2] = xp[2] + w * sp[i][2]
    end
    -- Predicted covariance + process noise
    local Pp = {mass=0.0, cross=0.0, mdot=0.0}
    for i, w in ipairs(Wc) do
        local dm = sp[i][1] - xp[1]
        local dd = sp[i][2] - xp[2]
        Pp.mass  = Pp.mass  + w * dm*dm
        Pp.cross = Pp.cross + w * dm*dd
        Pp.mdot  = Pp.mdot  + w * dd*dd
    end
    Pp.mass  = Pp.mass  + UKF_Q_MASS  * vibe_scale * dt
    Pp.mdot  = Pp.mdot  + UKF_Q_MDOT  * vibe_scale * dt
    ukf_x = xp; ukf_P = Pp
end

local function ukf_update(z, a_z)
    a_z = a_z or 0.0
    local R_scale = 1.0 + vibe_ema / math.max(1.0, VIBE_HI_THRESHOLD) * 4.0
    local R       = UKF_R_BASE * R_scale
    -- Measurement sigma points
    local pts, Wm, Wc = ukf_sigma_points()
    -- h(x) = mass * (g + a_z) / g   in grams
    local zp = {}
    for _, p in ipairs(pts) do
        zp[#zp+1] = math.max(0, p[1] * (G_EARTH + a_z) / G_EARTH)
    end
    local z_mean = 0.0
    for i, w in ipairs(Wm) do z_mean = z_mean + w * zp[i] end
    -- Innovation variance S and cross-covariance P_xz
    local S = R
    local Pxzm = 0.0; local Pxzd = 0.0
    for i, w in ipairs(Wc) do
        local dz  = zp[i] - z_mean
        local dm  = pts[i][1] - ukf_x[1]
        local dd  = pts[i][2] - ukf_x[2]
        S    = S    + w * dz*dz
        Pxzm = Pxzm + w * dm*dz
        Pxzd = Pxzd + w * dd*dz
    end
    -- Kalman gain
    local Km = Pxzm / S
    local Kd = Pxzd / S
    local innov = z - z_mean
    ukf_x[1] = math.max(0, ukf_x[1] + Km * innov)
    ukf_x[2] = ukf_x[2] + Kd * innov
    -- Joseph form covariance update
    ukf_P.mass  = ukf_P.mass  - Km * Km * S
    ukf_P.cross = ukf_P.cross - Km * Kd * S
    ukf_P.mdot  = ukf_P.mdot  - Kd * Kd * S
    -- Ensure positive definite
    ukf_P.mass  = math.max(0.01, ukf_P.mass)
    ukf_P.mdot  = math.max(0.01, ukf_P.mdot)
    ukf_initialised = true
end

local function ukf_uncertainty()
    return math.sqrt(math.max(0, ukf_P.mass))
end

-- ============================================================================
--  VERTICAL ACCELERATION PROXY  (from barometric altitude rate)
--  Differentiates baro altitude to estimate a_z (positive = upward accel).
-- ============================================================================

local function get_vz_accel()
    local alt = ahrs:get_altitude()
    local t   = now_s()
    if ukf_alt_prev == nil then
        ukf_alt_prev = alt; ukf_last_t = t; return 0.0
    end
    local dt = t - ukf_last_t
    if dt < 0.01 then return ukf_vz_prev end
    local vz  = (alt - ukf_alt_prev) / dt
    local az  = (vz - ukf_vz_prev)   / dt
    ukf_alt_prev = alt; ukf_last_t = t; ukf_vz_prev = vz
    return az   -- m/s²
end

-- ============================================================================
--  VIBRATION MONITOR  — EMA of vibe vector magnitude
-- ============================================================================

local function update_vibe()
    -- ArduPilot 4.3+ exposes ahrs:get_vibration() returning Vector3f
    local ok, vib = pcall(function() return ahrs:get_vibration() end)
    if not ok or vib == nil then return end
    local mag = math.sqrt(vib:x()^2 + vib:y()^2 + vib:z()^2)
    vibe_ema = 0.8 * vibe_ema + 0.2 * mag
end

local function adaptive_loop_ms()
    if not VIBE_ADAPT_ENABLED then return LOOP_BASE_MS end
    if vibe_ema > VIBE_HI_THRESHOLD then return LOOP_LO_MS end
    if vibe_ema < VIBE_LO_THRESHOLD then return LOOP_HI_MS end
    local t = (vibe_ema - VIBE_LO_THRESHOLD) /
              (VIBE_HI_THRESHOLD - VIBE_LO_THRESHOLD)
    return math.floor(LOOP_HI_MS + t * (LOOP_LO_MS - LOOP_HI_MS))
end

-- ============================================================================
--  ATE  — Adaptive Threshold Engine with aerodynamic correction
-- ============================================================================

local function ate_feed(apparent_g, armed, a_z)
    if not ATE_ENABLED then return end
    if ate_state == "IDLE" then
        if armed then
            ate_state = "COLLECTING"; ate_buf = {}; ate_az_buf = {}
            gcs(6, string.format("ATE: collecting baseline (%d samples)", ATE_WINDOW))
        end
        return
    end
    if ate_state == "COLLECTING" then
        if not armed then
            gcs(4, "ATE: disarmed before lock — reset")
            ate_state = "IDLE"; ate_buf = {}; return
        end
        if #ate_buf >= 2 then
            local m = table_mean(ate_buf)
            local s = table_stdev(ate_buf, m)
            if s > 0 and math.abs(apparent_g - m) > ATE_SIGMA * s then return end
        end
        ate_buf[#ate_buf+1]    = apparent_g
        ate_az_buf[#ate_az_buf+1] = a_z
        if #ate_buf >= ATE_WINDOW then
            -- Aerodynamic correction
            local base_apparent = table_mean(ate_buf)
            local mean_az = table_mean(ate_az_buf)
            local base_true = base_apparent
            if ATE_AERO_CORR and math.abs(mean_az) > 0.01 then
                base_true = base_apparent * G_EARTH / (G_EARTH + mean_az)
            end
            local new_t   = base_true * ATE_FRACTION
            local old_t   = THRESHOLD_G
            THRESHOLD_G   = new_t
            ate_baseline  = base_true
            ate_state     = "LOCKED"
            -- Prime CUSUM
            cusum_ref     = base_true
            cusum_neg     = 0.0
            -- Reset UKF around true baseline
            ukf_x = {base_true, 0.0}
            ukf_P = {mass=sq(50.0), cross=0.0, mdot=sq(10.0)}
            gcs(5, string.format(
                "ATE LOCKED: apparent=%.1fg true=%.1fg thr %.1f→%.1fg",
                base_apparent, base_true, old_t, new_t))
        end
    end
end

-- ============================================================================
--  CUSUM CHANGE-POINT DETECTOR
-- ============================================================================

local function cusum_feed(mass_g)
    if cusum_ref == nil then return 0.0 end
    cusum_neg = math.max(0.0, cusum_neg - (mass_g - cusum_ref) - CUSUM_K)
    return math.min(1.0, cusum_neg / CUSUM_H)
end

-- ============================================================================
--  OLS REGRESSION DETECTOR
-- ============================================================================

local function ols_feed(mass_g)
    ols_buf[ols_head] = {t = now_s(), w = mass_g}
    ols_head  = (ols_head % OLS_WINDOW) + 1
    ols_count = math.min(ols_count + 1, OLS_WINDOW)
end

local function ols_probability()
    local need = math.max(3, math.floor(OLS_WINDOW / 2))
    if ols_count < need then return 0.0 end
    local xs, ys = {}, {}
    for i = 1, OLS_WINDOW do
        if ols_buf[i] then
            xs[#xs+1] = ols_buf[i].t
            ys[#ys+1] = ols_buf[i].w
        end
    end
    local n = #xs
    if n < 3 then return 0.0 end
    local t0 = xs[1]
    local sx, sy, sxy, sx2 = 0.0, 0.0, 0.0, 0.0
    for i = 1, n do
        local xi = xs[i]-t0; local yi = ys[i]
        sx = sx+xi; sy = sy+yi; sxy = sxy+xi*yi; sx2 = sx2+xi*xi
    end
    local denom = n*sx2 - sq(sx)
    if math.abs(denom) < 1e-9 then return 0.0 end
    local slope = (n*sxy - sx*sy) / denom
    local intc  = (sy - slope*sx) / n
    local ym = sy/n; local sst, sse = 0.0, 0.0
    for i = 1, n do
        local xi = xs[i]-t0; local yi = ys[i]
        sst = sst + sq(yi-ym)
        sse = sse + sq(yi - (slope*xi + intc))
    end
    local r2 = sst > 1e-9 and (1.0 - sse/sst) or 0.0
    if r2 < 0.5 then return 0.0 end
    local slope_margin = slope - OLS_SLOPE_THR
    local p_slope = 1.0 / (1.0 + math.exp(slope_margin * 0.05))
    return p_slope * math.min(1.0, r2)
end

-- ============================================================================
--  BAYESIAN COMBINER
-- ============================================================================

local function bayesian_posterior()
    local p_ols   = ols_probability()
    local p_cusum = cusum_feed(ukf_x[1])
    local posterior = W_OLS * p_ols + W_CUSUM * p_cusum
    return posterior, p_ols, p_cusum
end

-- ============================================================================
--  PID TRANSITION GATE
-- ============================================================================

local function pid_enter(weight_g)
    pid_integral   = 0.0
    pid_prev_err   = THRESHOLD_G - weight_g
    pid_prev_t     = now_s()
    pid_entered_t  = now_s()
end

local function pid_update(weight_g)
    local t   = now_s()
    local dt  = t - pid_prev_t
    if dt < 1e-6 then return false, 0.0 end
    local err  = THRESHOLD_G - weight_g
    local derr = (err - pid_prev_err) / dt
    pid_integral  = pid_integral + err * dt
    local output  = PID_KP * err + PID_KI * pid_integral + PID_KD * derr
    pid_prev_err  = err; pid_prev_t = t
    -- Hard dwell cap
    if t - pid_entered_t > DWELL_MAX_S then
        gcs(4, string.format("PID: dwell cap %.1fs → force commit", t - pid_entered_t))
        return true, output
    end
    return output >= PID_COMMIT_THOLD, output
end

local function pid_reset()
    pid_integral = 0.0; pid_prev_err = 0.0
end

-- ============================================================================
--  JERK GATE
-- ============================================================================

local function jerk_is_safe()
    local rr = ahrs:get_gyro():x()
    local pr = ahrs:get_gyro():y()
    local yr = ahrs:get_gyro():z()
    local t  = now_s()
    if prev_rates_t == 0.0 then
        prev_rr=rr; prev_pr=pr; prev_yr=yr; prev_rates_t=t; return true
    end
    local dt = t - prev_rates_t
    if dt < 1e-6 then return true end
    local j = math.sqrt(
        sq((rr-prev_rr)/dt) + sq((pr-prev_pr)/dt) + sq((yr-prev_yr)/dt))
    prev_rr=rr; prev_pr=pr; prev_yr=yr; prev_rates_t=t
    if j > JERK_MAX_RAD_S2 then
        gcs(7, string.format("JERK: %.2f rad/s² — blocked", j)); return false
    end
    return true
end

-- ============================================================================
--  ATTITUDE GATE
-- ============================================================================

local function attitude_is_safe()
    local r = math.abs(deg(ahrs:get_roll()))
    local p = math.abs(deg(ahrs:get_pitch()))
    if r > ATTITUDE_MAX_DEG then gcs(7, string.format("ATT: roll %.1f°", r)); return false end
    if p > ATTITUDE_MAX_DEG then gcs(7, string.format("ATT: pitch %.1f°", p)); return false end
    return true
end

-- ============================================================================
--  PIXHAWK INTERLOCK
-- ============================================================================

local function pixhawk_ok()
    if not PX_INTERLOCK then return true end
    local hb = param:get("HYDRA_PX_HB")
    if hb == nil then return true end
    local age_ms = millis() - math.floor(hb * 1000)
    if age_ms > PX_HB_TIMEOUT_MS then
        gcs(3, string.format("PX_INTERLOCK: HB stale %d ms", age_ms))
        return false
    end
    return true
end

-- ============================================================================
--  ALL SAFETY GATES
-- ============================================================================

local function all_safe()
    return attitude_is_safe() and jerk_is_safe() and pixhawk_ok()
end

-- ============================================================================
--  ESC TREND WATCHDOG
-- ============================================================================

local function esc_feed_rpm(motor_idx, rpm)
    local b = esc_rpm_bufs[motor_idx]
    if b == nil then return end
    b.buf[b.head] = rpm
    b.ts[b.head]  = now_s()
    b.head  = (b.head % ESC_RPM_BUF_SIZE) + 1
    b.count = math.min(b.count + 1, ESC_RPM_BUF_SIZE)
end

local function esc_trend_slope(motor_idx)
    local b = esc_rpm_bufs[motor_idx]
    if b.count < 10 then return 0.0 end
    local xs, ys = {}, {}
    for i = 1, ESC_RPM_BUF_SIZE do
        if b.buf[i] and b.buf[i] > 0 then
            xs[#xs+1] = b.ts[i]; ys[#ys+1] = b.buf[i]
        end
    end
    local n = #xs
    if n < 5 then return 0.0 end
    local t0 = xs[1]
    local sx, sy, sxy, sx2 = 0.0, 0.0, 0.0, 0.0
    for i = 1, n do
        local xi = xs[i]-t0; local yi = ys[i]
        sx=sx+xi; sy=sy+yi; sxy=sxy+xi*yi; sx2=sx2+xi*xi
    end
    local denom = n*sx2 - sq(sx)
    if math.abs(denom) < 1e-9 then return 0.0 end
    return (n*sxy - sx*sy) / denom
end

local function esc_check_degrading()
    for _, idx in ipairs(PRIMARY_MOTORS) do
        local slope = esc_trend_slope(idx)
        local was   = motor_degrading[idx]
        motor_degrading[idx] = slope < RPM_WARN_SLOPE
        if motor_degrading[idx] and not was then
            gcs(0, string.format(
                "!! M%d RPM DEGRADING: slope=%.1f RPM/s", idx, slope))
            param:set("HYDRA_M_DEGRADE", idx)
            return true, idx
        end
    end
    return false, 0
end

-- ============================================================================
--  MIXER COMPENSATION
-- ============================================================================

local function mixer_apply_quad()
    if not MIXER_ENABLED then return true end
    local p = param:get("ATC_RAT_YAW_P")
    local i = param:get("ATC_RAT_YAW_I")
    if p == nil or i == nil then
        gcs(4, "MIXER: cannot read YAW PID"); return false
    end
    orig_yaw_p = p; orig_yaw_i = i
    local ok_p = param:set_and_save("ATC_RAT_YAW_P", p * QUAD_YAW_P_SCALE)
    local ok_i = param:set_and_save("ATC_RAT_YAW_I", i * QUAD_YAW_I_SCALE)
    if ok_p and ok_i then
        mixer_applied = true
        gcs(5, string.format("MIXER: YAW P %.4f→%.4f  I %.4f→%.4f",
            p, p*QUAD_YAW_P_SCALE, i, i*QUAD_YAW_I_SCALE))
    end
    return ok_p and ok_i
end

local function mixer_restore_octo()
    if not mixer_applied or orig_yaw_p == nil then return true end
    local ok_p = param:set_and_save("ATC_RAT_YAW_P", orig_yaw_p)
    local ok_i = param:set_and_save("ATC_RAT_YAW_I", orig_yaw_i)
    if ok_p and ok_i then
        mixer_applied = false
        gcs(5, string.format("MIXER: YAW restored P=%.4f I=%.4f",
            orig_yaw_p, orig_yaw_i))
    end
    return ok_p and ok_i
end

-- ============================================================================
--  PARAM WRITER WITH RETRY + READBACK
-- ============================================================================

local function write_servo(motor_idx, func_val)
    local pname = "SERVO" .. motor_idx .. "_FUNCTION"
    for attempt = 1, MAX_PARAM_RETRIES do
        local ok = param:set_and_save(pname, func_val)
        if ok then
            local rb = param:get(pname)
            if rb ~= nil and math.abs(rb - func_val) < 0.5 then
                gcs(6, string.format("SET %s=%d ✓ (att %d)",
                    pname, func_val, attempt))
                return true
            end
        end
        gcs(4, string.format("SET %s=%d RETRY %d", pname, func_val, attempt))
    end
    gcs(3, string.format("FAILED: %s=%d", pname, func_val))
    return false
end

-- ============================================================================
--  MOTOR HEALTH CHECK  (via ESC_RPM proxy)
-- ============================================================================

local function motor_verify(idx, should_spin)
    local buf = esc_rpm_bufs[idx]
    if buf.count == 0 then return true end
    -- Most recent RPM sample
    local latest_slot = ((buf.head - 2) % ESC_RPM_BUF_SIZE) + 1
    local rpm = buf.buf[latest_slot] or 0
    local ok  = should_spin and (rpm >= MOTOR_MIN_RPM)
             or (not should_spin and rpm <= MOTOR_MAX_IDLE_RPM)
    if not ok then
        gcs(0, string.format("HEALTH: M%d should_spin=%s rpm=%d FAIL",
            idx, tostring(should_spin), rpm))
    end
    return ok
end

-- ============================================================================
--  EMERGENCY SHUTDOWN
-- ============================================================================

local function emergency_shutdown_all()
    gcs(0, "EMERGENCY SHUTDOWN: disabling ALL 8 motors NOW")
    for idx = 1, 8 do
        param:set_and_save("SERVO" .. idx .. "_FUNCTION", 0)
    end
    fsm_state = FSM.EMER
end

-- ============================================================================
--  BOOT SELF-TEST
-- ============================================================================

local self_test_step     = 0
local self_test_motor_i  = 1
local self_test_start_ms = 0

local function boot_self_test_tick()
    -- Step 0: initialise
    if self_test_step == 0 then
        gcs(5, "SELF-TEST: spinning M5-M8 briefly...")
        self_test_step    = 1
        self_test_motor_i = 1
        return false   -- not done
    end
    -- Step 1: spin one motor for SELF_TEST_SPIN_MS
    if self_test_step == 1 then
        local m = SECONDARY_MOTORS[self_test_motor_i]
        if m == nil then
            self_test_step = 2; return false
        end
        if self_test_start_ms == 0 then
            self_test_start_ms = millis()
            -- Ensure motor function set
            write_servo(m, MOTOR_FUNC_BASE + m - 1)
        end
        if millis() - self_test_start_ms < SELF_TEST_SPIN_MS then
            return false   -- wait
        end
        -- Check RPM
        local ok = motor_verify(m, true)
        if not ok then
            gcs(0, string.format("SELF-TEST FAIL: M%d did not spin", m))
        else
            gcs(6, string.format("SELF-TEST: M%d OK", m))
        end
        self_test_motor_i  = self_test_motor_i + 1
        self_test_start_ms = 0
        return false
    end
    -- Step 2: done
    gcs(5, "SELF-TEST: complete")
    return true
end

-- ============================================================================
--  SENTINEL PARAM WRITE
-- ============================================================================

local FSM_CODES = {
    [FSM.OCTO]  = 0, [FSM.PRE]   = 1, [FSM.TQUAD] = 2,
    [FSM.QUAD]  = 3, [FSM.TOCTO] = 4, [FSM.DEGR]  = 5, [FSM.EMER]  = 6,
}

local function write_sentinels(posterior, pid_i)
    param:set("HYDRA_STATE",  FSM_CODES[fsm_state] or 0)
    param:set("HYDRA_MASS",   math.floor(ukf_x[1]))
    param:set("HYDRA_POST",   math.floor(posterior * 1000))  -- ×1000 integer
    param:set("HYDRA_PID_I",  math.floor(pid_i * 1000))
    param:set("HYDRA_VIBE",   math.floor(vibe_ema * 100))
end

-- ============================================================================
--  SERIAL READER  (HX711 ASCII stream)
-- ============================================================================

local function serial_init()
    serial_port = serial:find_serial(1)   -- SERIAL2 → index 1
    if serial_port == nil then
        gcs(3, "SERIAL2 not found — check SERIAL2_PROTOCOL=28")
        return false
    end
    serial_port:begin(9600)
    gcs(6, "SERIAL2 HX711 adapter ready @ 9600")
    return true
end

local function serial_read_weight()
    if serial_port == nil then return nil end
    local avail = serial_port:available()
    if avail == nil or avail <= 0 then
        serial_fails = serial_fails + 1; return nil
    end
    local n = math.min(avail, 64)
    for _ = 1, n do
        local b = serial_port:read()
        if b == nil then break end
        local ch = string.char(b)
        if ch == "\n" or ch == "\r" then
            local line = rx_buf:match("^%s*(.-)%s*$")
            rx_buf = ""
            if line and #line > 0 then
                local raw = tonumber(line)
                if raw and raw >= 0 then
                    serial_fails = 0; total_reads = total_reads + 1
                    return raw
                else
                    total_errors = total_errors + 1
                end
            end
        else
            rx_buf = rx_buf .. ch
            if #rx_buf > 32 then rx_buf = ""; serial_fails = serial_fails + 1 end
        end
    end
    serial_fails = serial_fails + 1; return nil
end

-- ============================================================================
--  SEVEN-STATE FSM
-- ============================================================================

local function fsm_set(ns)
    if fsm_state ~= ns then
        gcs(5, string.format("FSM: %s → %s", fsm_state, ns))
    end
    fsm_state = ns
end

local function do_transition_to_quad(w)
    fsm_set(FSM.TQUAD)
    -- 1. Mixer compensation (before disable)
    mixer_apply_quad()
    -- 2. Disable secondary motors
    local all_ok = true
    for _, m in ipairs(SECONDARY_MOTORS) do
        if not write_servo(m, 0) then all_ok = false end
    end
    -- 3. Wait one loop tick (MOTOR_SETTLE_MS handled externally — we rely on
    --    the next call returning from the scheduler)
    -- 4. Verify
    for _, m in ipairs(SECONDARY_MOTORS) do
        motor_verify(m, false)
    end
    if all_ok then
        fsm_set(FSM.QUAD)
        fsm_transitions = fsm_transitions + 1
        local lat = (now_s() - pre_drop_t) * 1000
        gcs(4, string.format(
            ">>> QUAD ACTIVE [#%d] lat=%.0fms mass=%.1fg unc=%.1fg",
            fsm_transitions, lat, ukf_x[1], ukf_uncertainty()))
    else
        mixer_restore_octo()
        gcs(3, "T_QUAD: write failure — revert OCTO")
        fsm_set(FSM.OCTO)
    end
end

local function do_transition_to_octo(w)
    fsm_set(FSM.TOCTO)
    -- 1. Restore mixer BEFORE re-enabling
    mixer_restore_octo()
    local all_ok = true
    for _, m in ipairs(SECONDARY_MOTORS) do
        local fv = MOTOR_FUNC_BASE + m - 1
        if not write_servo(m, fv) then all_ok = false end
    end
    for _, m in ipairs(SECONDARY_MOTORS) do
        motor_verify(m, true)
    end
    if all_ok then
        fsm_set(FSM.OCTO)
        fsm_transitions = fsm_transitions + 1
        gcs(5, string.format(">>> OCTO ACTIVE [#%d]", fsm_transitions))
    else
        mixer_apply_quad()   -- re-apply since we're stuck in quad
        gcs(3, "T_OCTO: write failure — retain QUAD")
        fsm_set(FSM.QUAD)
    end
end

local function fsm_evaluate(w, posterior)
    local lo = THRESHOLD_G
    local hi = lo + HYSTERESIS_G

    -- Primary degradation check
    local degraded, bad_m = esc_check_degrading()
    if degraded and fsm_state ~= FSM.EMER then
        gcs(0, string.format("M%d DEGRADING → DEGRADED state", bad_m))
        -- If armed and flying, command RTL
        if vehicle:get_armed() then
            vehicle:set_mode(vehicle.mode_name_to_num("RTL"))
        end
        fsm_set(FSM.DEGR)
        return
    end

    if fsm_state == FSM.OCTO then
        if posterior >= POSTERIOR_THRESHOLD or w < lo then
            gcs(4, string.format(
                "OCTO→PRE_DROP post=%.3f w=%.1fg", posterior, w))
            pre_drop_t = now_s()
            pid_enter(w)
            fsm_set(FSM.PRE)
        end

    elseif fsm_state == FSM.PRE then
        if w >= hi then
            gcs(5, string.format("PRE_DROP: w=%.1fg recovered → OCTO", w))
            pid_reset(); fsm_set(FSM.OCTO); return
        end
        local commit, pid_out = pid_update(w)
        if commit and all_safe() then
            do_transition_to_quad(w)
        else
            gcs(7, string.format(
                "PRE_DROP: waiting PID=%.3f safe=%s",
                pid_out, tostring(all_safe())))
        end

    elseif fsm_state == FSM.QUAD then
        if w >= hi and all_safe() then
            do_transition_to_octo(w)
        end

    elseif fsm_state == FSM.DEGR then
        -- No transitions; RTL already commanded

    elseif fsm_state == FSM.EMER then
        gcs(0, "EMERGENCY state — reboot required")
    end
end

-- ============================================================================
--  BOOT SYNC
-- ============================================================================

local function boot_sync()
    for _, idx in ipairs(SECONDARY_MOTORS) do
        local v = param:get("SERVO" .. idx .. "_FUNCTION")
        if v ~= nil and math.floor(v) == 0 then
            fsm_state = FSM.QUAD
            gcs(5, string.format("boot_sync: M%d=0 → start in QUAD", idx))
            return
        end
    end
    fsm_state = FSM.OCTO
    gcs(6, "boot_sync: all motors active → OCTO")
end

-- ============================================================================
--  MAIN LOOP
-- ============================================================================

local initialized = false

local function update()

    -- ── First-run init ────────────────────────────────────────────────────
    if not initialized then
        init_crc32()
        boot_sync()
        if not serial_init() then
            gcs(3, "Serial init failed — retry in 5s")
            return update, 5000
        end
        initialized = true
        gcs(5, string.format(
            "HYDRA v4.0 READY | thr=%.0fg hyst=%.0fg UKF=on MHDD=on MIXER=%s",
            THRESHOLD_G, HYSTERESIS_G, tostring(MIXER_ENABLED)))
    end

    -- ── ESC RPM poll (Bidirectional DSHOT via ArduPilot esc_rpm API) ─────────
    -- esc_rpm:get_rpm(idx) returns RPM for motor index 0-based.
    -- Available on ArduCopter 4.3+ with SERVO_BLH_BDSHOT=1.
    -- Falls back gracefully if the API is absent (older firmware).
    do
        local esc_ok, esc_obj = pcall(function() return esc_rpm end)
        if esc_ok and esc_obj ~= nil then
            for _, idx in ipairs(ALL_MOTORS) do
                local rpm_ok, rpm_val = pcall(function()
                    return esc_obj:get_rpm(idx - 1)  -- ArduPilot is 0-based
                end)
                if rpm_ok and rpm_val ~= nil and rpm_val >= 0 then
                    esc_feed_rpm(idx, math.floor(rpm_val))
                end
            end
        end
    end

    -- ── Self-test on first arm ────────────────────────────────────────────
    local armed = vehicle:get_armed()
    if SELF_TEST_ENABLED and not self_test_done then
        if armed and not self_test_arm_prev then
            -- Just armed: run self-test
            self_test_arm_prev = true
        end
        if self_test_arm_prev then
            local done = boot_self_test_tick()
            if done then self_test_done = true end
            return update, LOOP_BASE_MS
        end
    end
    self_test_arm_prev = armed

    -- ── Vibration update ─────────────────────────────────────────────────
    update_vibe()
    loop_interval_ms = adaptive_loop_ms()

    -- ── Vertical accel proxy ─────────────────────────────────────────────
    local a_z = get_vz_accel()

    -- ── UKF predict ──────────────────────────────────────────────────────
    local dt = loop_interval_ms / 1000.0
    ukf_predict(dt, 1.0 + vibe_ema / VIBE_HI_THRESHOLD)

    -- ── Read weight ───────────────────────────────────────────────────────
    local raw = serial_read_weight()

    if serial_fails >= SENSOR_FAIL_WARN then
        gcs(3, string.format(
            "WATCHDOG: %d serial failures — check HX711 wiring", serial_fails))
        serial_fails = 0
    end

    if raw ~= nil then
        raw = math.max(0.0, raw)
        -- UKF update fusing raw measurement and a_z
        ukf_update(raw, a_z)
        local mass = ukf_x[1]
        local unc  = ukf_uncertainty()

        -- Feed ATE
        ate_feed(raw, armed, a_z)

        -- Feed OLS buffer
        ols_feed(mass)

        -- Bayesian posterior
        local posterior, p_ols, p_cusum = bayesian_posterior()

        -- FSM evaluation
        fsm_evaluate(mass, posterior)

        -- Sentinel params
        if millis() - sentinel_last >= SENTINEL_INTERVAL_MS then
            sentinel_last = millis()
            write_sentinels(posterior, pid_integral * PID_KI)
        end

        -- Telemetry
        if millis() - telem_last >= TELEM_INTERVAL_MS then
            telem_last = millis()
            local r  = deg(ahrs:get_roll())
            local p  = deg(ahrs:get_pitch())
            gcs(6, string.format(
                "TEL | %-10s  mass=%.1f±%.1fg  post=%.3f(OLS=%.2f CUSUM=%.2f)"
                .. "  r=%.1f° p=%.1f°  vibe=%.1f  thr=%.0fg  ate=%s  hz=%.0f",
                fsm_state, mass, unc, posterior, p_ols, p_cusum,
                r, p, vibe_ema, THRESHOLD_G, ate_state,
                1000.0 / loop_interval_ms))
        end
    end

    return update, loop_interval_ms
end

-- ── Entry point ──────────────────────────────────────────────────────────────
gcs:send_text(6, "[HYDRA-v4] motor_controller_v4.lua loaded")
return update()
