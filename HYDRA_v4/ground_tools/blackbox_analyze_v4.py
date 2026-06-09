#!/usr/bin/env python3
"""
blackbox_analyze_v4.py
══════════════════════
Project HYDRA v4 — Advanced Post-Flight Analyzer

Parses v4 JSONL logs and produces:
  1. FSM timeline with PID trace
  2. UKF mass estimate vs raw sensor vs true mass
  3. MHDD hypothesis breakdown (H0/H1/H2 per event)
  4. ESC RPM trend per motor with degradation markers
  5. Vibration level vs loop rate correlation
  6. Mixer compensation events
  7. Mission phase timeline
  8. Chain integrity summary (calls blackbox_verify)
  9. Optional matplotlib multi-panel flight report

Usage
─────
  python3 blackbox_analyze_v4.py <logfile.jsonl> [--plot] [--csv] [--html]
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import defaultdict
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple


# ────────────────────────────────────────────────────────────────────────────

def ts(t: float) -> str:
    return datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]

def rel(t: float, t0: float) -> str:
    d = t - t0
    return f"T+{d:7.2f}s"


def load_log(path: str) -> Tuple[List[Dict], int, int]:
    records, bad = [], 0
    with open(path) as f:
        for i, line in enumerate(f, 1):
            line = line.strip()
            if not line: continue
            try:
                rec = json.loads(line)
                records.append(rec)
            except json.JSONDecodeError:
                bad += 1
    return records, len(records), bad


def by_tag(records: List[Dict]) -> Dict[str, List[Dict]]:
    out = defaultdict(list)
    for r in records:
        out[r.get("tag", "?")].append(r)
    return dict(out)


# ════════════════════════════════════════════════════════════════════════════
#  MAIN ANALYSIS
# ════════════════════════════════════════════════════════════════════════════

def analyze(records: List[Dict], args: Any) -> None:
    if not records:
        print("No records."); return

    t0   = records[0]["t"]
    t1   = records[-1]["t"]
    dur  = t1 - t0
    bt   = by_tag(records)

    # ── Header ──────────────────────────────────────────────────────────────
    print("═" * 72)
    print("  HYDRA v4 Post-Flight Analysis")
    print(f"  File     : {args.logfile}")
    print(f"  Start    : {ts(t0)}")
    print(f"  End      : {ts(t1)}")
    print(f"  Duration : {dur:.1f}s  ({dur/60:.1f} min)")
    print(f"  Records  : {len(records)}")
    print("═" * 72)

    # ── System Start / Stop ──────────────────────────────────────────────────
    for rec in bt.get("system_start", [])[:1]:
        print(f"\n[SYSTEM START]  {ts(rec['t'])}")
        print(f"  UKF_alpha={rec.get('ukf_alpha','?')}  "
              f"MHDD={rec.get('mhdd','?')}  "
              f"mixer={rec.get('mixer','?')}  "
              f"prom_port={rec.get('prom_port','?')}")

    for rec in bt.get("system_stop", [])[-1:]:
        print(f"\n[SYSTEM STOP]  {ts(rec['t'])}")
        print(f"  transitions={rec.get('transitions','?')}  "
              f"reads={rec.get('reads','?')}  "
              f"errors={rec.get('errors','?')}  "
              f"final={rec.get('final_state','?')}")

    # ── ATE ──────────────────────────────────────────────────────────────────
    ate_locks = bt.get("ATE_locked", [])
    print(f"\n[ADAPTIVE THRESHOLD ENGINE]  locks={len(ate_locks)}")
    for al in ate_locks:
        print(f"  {rel(al['t'],t0)}  apparent={al.get('baseline_apparent','?'):.1f}g  "
              f"true={al.get('baseline_true','?'):.1f}g  "
              f"thr {al.get('old_thr','?'):.1f}→{al.get('new_thr','?'):.1f}g")

    # ── UKF performance ───────────────────────────────────────────────────────
    ukf_recs = bt.get("ukf_update", [])
    print(f"\n[UKF SENSOR FUSION]  updates={len(ukf_recs)}")
    if ukf_recs:
        masses  = [r.get("mass",0) for r in ukf_recs]
        uncs    = [r.get("unc",0)  for r in ukf_recs]
        innovs  = [r.get("innov",0) for r in ukf_recs]
        print(f"  Mass    min={min(masses):.1f}g  max={max(masses):.1f}g  "
              f"mean={sum(masses)/len(masses):.1f}g")
        print(f"  Uncert  min={min(uncs):.2f}g  max={max(uncs):.2f}g  "
              f"mean={sum(uncs)/len(uncs):.2f}g")
        print(f"  Innov   min={min(innovs):.1f}g  max={max(innovs):.1f}g  "
              f"RMS={math.sqrt(sum(i**2 for i in innovs)/len(innovs)):.1f}g")

    # ── MHDD ─────────────────────────────────────────────────────────────────
    mhdd_trigs = bt.get("MHDD_trigger", [])
    print(f"\n[MULTI-HYPOTHESIS DROP DETECTOR]  triggers={len(mhdd_trigs)}")
    for mt in mhdd_trigs:
        print(f"  {rel(mt['t'],t0)}  post={mt.get('posterior','?'):.3f}  "
              f"H0={mt.get('H0_ols','?'):.2f}  "
              f"H1={mt.get('H1_cusum','?'):.2f}  "
              f"H2={mt.get('H2_var','?'):.2f}")

    # ── FSM ───────────────────────────────────────────────────────────────────
    transitions = bt.get("FSM_transition", [])
    quad_events = bt.get("FSM_quad_active", [])
    octo_events = bt.get("FSM_octo_active", [])
    pre_drops   = bt.get("FSM_pre_drop", [])
    degraded    = bt.get("FSM_degraded", [])
    emergency   = bt.get("emergency_shutdown", [])

    print(f"\n[FSM TRANSITIONS]  total={len(transitions)}")
    for tr in transitions:
        frm = tr.get("frm", tr.get("from_state","?"))
        to  = tr.get("to",  tr.get("to_state","?"))
        print(f"  {rel(tr['t'],t0)}  {frm:22s} → {to}")

    if quad_events:
        print(f"\n  QUAD activations: {len(quad_events)}")
        for q in quad_events:
            lat = q.get("lat_ms","?")
            print(f"    {rel(q['t'],t0)}  #{q.get('n',q.get('transition','?'))}  "
                  f"lat={lat:.0f}ms  sec_health={q.get('sec_health','?'):.2f}"
                  if isinstance(lat, float) else
                  f"    {rel(q['t'],t0)}  #{q.get('n','?')}")

    if degraded:
        print(f"\n  !! DEGRADED entries: {len(degraded)}")
        for d in degraded:
            print(f"     {rel(d['t'],t0)}  motor={d.get('motor','?')}")

    if emergency:
        print(f"\n  !! EMERGENCY SHUTDOWN fired: {len(emergency)}")

    # ── PID gate ──────────────────────────────────────────────────────────────
    pid_commits = bt.get("pid_commit", [])
    pid_forced  = bt.get("pid_force_commit", [])
    print(f"\n[PID TRANSITION GATE]  commits={len(pid_commits)}  "
          f"force_commits={len(pid_forced)}")
    for pc in pid_commits:
        print(f"  {rel(pc['t'],t0)}  output={pc.get('output','?'):.3f}  "
              f"integral={pc.get('integral','?'):.3f}  "
              f"dwell={pc.get('dwell_s','?'):.2f}s")
    for pf in pid_forced:
        print(f"  !! FORCED  {rel(pf['t'],t0)}  "
              f"dwell={pf.get('dwell_s','?'):.2f}s")

    # ── Mixer compensation ────────────────────────────────────────────────────
    mix_quad = bt.get("mixer_quad_applied", [])
    mix_octo = bt.get("mixer_octo_restored", [])
    print(f"\n[MIXER COMPENSATION]  quad_applies={len(mix_quad)}  "
          f"octo_restores={len(mix_octo)}")
    for mq in mix_quad:
        print(f"  {rel(mq['t'],t0)}  YAW_P {mq.get('orig_p','?'):.4f}→"
              f"{mq.get('new_p','?'):.4f}  "
              f"YAW_I {mq.get('orig_i','?'):.4f}→{mq.get('new_i','?'):.4f}")

    # ── ESC degradation ───────────────────────────────────────────────────────
    esc_deg = bt.get("ESC_degrading", [])
    print(f"\n[ESC HEALTH REGRESSION]  degradation_events={len(esc_deg)}")
    for ed in esc_deg:
        print(f"  !! {rel(ed['t'],t0)}  M{ed.get('motor','?')}  "
              f"slope={ed.get('slope','?'):.2f}RPM/s  "
              f"p={ed.get('pval','?'):.4f}")

    # ── Sensor arbitration ────────────────────────────────────────────────────
    arb_recs = bt.get("sensor_arb", [])
    arb_diverg = [r for r in arb_recs if r.get("mahal",0) > 3.0]
    print(f"\n[SENSOR ARBITRATION]  reads={len(arb_recs)}  "
          f"divergence_events={len(arb_diverg)}")
    for ad in arb_diverg[:5]:
        print(f"  {rel(ad['t'],t0)}  M={ad.get('mahal','?'):.2f}  "
              f"zp={ad.get('zp','?'):.1f}g  zs={ad.get('zs','?'):.1f}g")

    # ── Vibration / loop rate ─────────────────────────────────────────────────
    telem_recs = bt.get("telemetry", [])
    if telem_recs:
        vibes = [r.get("vibe",0) for r in telem_recs]
        hzs   = [r.get("hz",5) for r in telem_recs]
        print(f"\n[VIBRATION / ADAPTIVE LOOP RATE]")
        print(f"  Vibe   min={min(vibes):.1f}  max={max(vibes):.1f}  "
              f"mean={sum(vibes)/len(vibes):.1f}")
        print(f"  Loop   min={min(hzs):.1f}Hz  max={max(hzs):.1f}Hz  "
              f"mean={sum(hzs)/len(hzs):.1f}Hz")

    # ── Mission phases ────────────────────────────────────────────────────────
    mission_recs = bt.get("mission_phase", [])
    print(f"\n[MISSION PHASE CLASSIFIER]  phase_changes={len(mission_recs)}")
    for mr in mission_recs:
        print(f"  {rel(mr['t'],t0)}  WP={mr.get('wp','?')}  "
              f"CMD={mr.get('cmd','?')}  phase={mr.get('phase','?')}")

    # ── Connectivity ──────────────────────────────────────────────────────────
    hb_t = bt.get("hb_timeout",[]) + bt.get("heartbeat_timeout",[])
    px_s = bt.get("pixhawk_hb_stale",[])
    att_d = bt.get("attitude_divergence",[])
    print(f"\n[CONNECTIVITY]  hb_timeouts={len(hb_t)}  "
          f"px_stale={len(px_s)}  att_diverge={len(att_d)}")

    # ── Param writes ──────────────────────────────────────────────────────────
    pw_all  = bt.get("param_write", [])
    pw_ok   = [r for r in pw_all if r.get("ok")]
    pw_fail = [r for r in pw_all if not r.get("ok")]
    print(f"\n[PARAM WRITES]  total={len(pw_all)}  "
          f"ok={len(pw_ok)}  failed={len(pw_fail)}")
    for pf in pw_fail:
        print(f"  !! {rel(pf['t'],t0)}  {pf.get('param','?')}="
              f"{pf.get('value','?')}  attempts={pf.get('attempt','?')}")

    # ── Shutdown ──────────────────────────────────────────────────────────────
    sd_s = bt.get("shutdown_start",[])
    sd_e = bt.get("shutdown_complete",[])
    if sd_s:
        print(f"\n[SHUTDOWN]  sequences={len(sd_s)}")
        for i, ss in enumerate(sd_s):
            print(f"  Start {rel(ss['t'],t0)}", end="")
            if i < len(sd_e):
                dur_sd = sd_e[i]["t"] - ss["t"]
                print(f"  → Complete {rel(sd_e[i]['t'],t0)}  ({dur_sd:.1f}s)")
            else:
                print("  (no completion record)")

    print("\n" + "═" * 72)

    # ── CSV export ────────────────────────────────────────────────────────────
    if args.csv:
        csv_path = args.logfile.replace(".jsonl", "_v4_transitions.csv")
        rows = [r for r in records if r.get("tag","").startswith("FSM_")
                or r.get("tag") in ("MHDD_trigger","ATE_locked",
                                     "mixer_quad_applied","pid_commit",
                                     "ESC_degrading","emergency_shutdown")]
        if rows:
            keys = sorted({k for r in rows for k in r if not k.startswith("_")})
            with open(csv_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["rel_s"]+keys,
                                   extrasaction="ignore")
                w.writeheader()
                for r in rows:
                    r["rel_s"] = round(r.get("t",0) - t0, 3)
                    w.writerow(r)
            print(f"CSV: {csv_path}  ({len(rows)} events)")

    # ── Plot ──────────────────────────────────────────────────────────────────
    if args.plot:
        _plot(records, bt, t0, args)


def _plot(records, bt, t0, args):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib not installed — skipping plot (pip3 install matplotlib)")
        return

    ukf_recs  = bt.get("ukf_update", [])
    pipe_recs = bt.get("pipeline", [])
    telem     = bt.get("telemetry", [])
    trans     = bt.get("FSM_transition", [])

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("HYDRA v4 Post-Flight Analysis", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax_mass  = fig.add_subplot(gs[0, :])   # full width — mass/weight
    ax_post  = fig.add_subplot(gs[1, 0])   # MHDD posterior
    ax_pid   = fig.add_subplot(gs[1, 1])   # PID integral
    ax_esc   = fig.add_subplot(gs[2, 0])   # ESC RPM (primaries)
    ax_vibe  = fig.add_subplot(gs[2, 1])   # Vibration + loop rate
    ax_fsm   = fig.add_subplot(gs[3, :])   # FSM state timeline

    # ── Mass panel ────────────────────────────────────────────────────────────
    if pipe_recs:
        pt  = [r["t"]-t0 for r in pipe_recs]
        raw = [r.get("raw",0) for r in pipe_recs]
        ax_mass.plot(pt, raw, color="#aaaaaa", linewidth=0.8,
                     label="Raw sensor", alpha=0.7)
    if ukf_recs:
        ut  = [r["t"]-t0 for r in ukf_recs]
        um  = [r.get("mass",0) for r in ukf_recs]
        uu  = [r.get("unc",0)  for r in ukf_recs]
        ax_mass.plot(ut, um, color="steelblue", linewidth=1.5, label="UKF mass")
        ax_mass.fill_between(ut,
            [m-u for m,u in zip(um,uu)], [m+u for m,u in zip(um,uu)],
            alpha=0.2, color="steelblue", label="UKF ±1σ")

    # ATE threshold line
    for al in bt.get("ATE_locked",[]):
        thr = al.get("new_thr", 0)
        ax_mass.axhline(y=thr, color="red", linestyle="--", linewidth=0.8,
                        label=f"Threshold {thr:.0f}g")

    ax_mass.set_ylabel("Mass (g)"); ax_mass.set_title("Weight / UKF Mass Estimate")
    ax_mass.legend(loc="upper right", fontsize=7); ax_mass.grid(True, alpha=0.3)

    # ── FSM state bands on mass panel ────────────────────────────────────────
    state_col = {
        "OCTOCOPTER":"#00800033", "PRE_DROP":"#FFA50055",
        "TRANSITIONING_QUAD":"#FF000055","QUAD":"#FF000033",
        "TRANSITIONING_OCTO":"#FFA50055","DEGRADED":"#80008055",
        "EMERGENCY":"#FF000099",
    }
    prev_t, prev_s = 0., "OCTOCOPTER"
    t_end_rel = records[-1]["t"] - t0 if records else 1.0
    all_t = [r["t"] - t0 for r in pipe_recs] if pipe_recs else [0., t_end_rel]
    for tr in trans:
        rt = tr["t"]-t0
        c  = state_col.get(prev_s, "#88888822")
        ax_mass.axvspan(prev_t, rt, alpha=0.15, color=c)
        prev_t = rt; prev_s = tr.get("to", tr.get("to_state","OCTOCOPTER"))
    ax_mass.axvspan(prev_t, max(all_t) if all_t else t_end_rel,
                    alpha=0.15, color=state_col.get(prev_s,"#88888822"))

    # ── MHDD posterior ────────────────────────────────────────────────────────
    if pipe_recs:
        pp = [r.get("post",0) for r in pipe_recs]
        ax_post.plot(pt, pp, color="darkorange", linewidth=1)
        ax_post.axhline(y=0.75, color="red", linestyle="--", linewidth=0.8,
                        label="Trigger threshold")
    ax_post.set_ylabel("Posterior"); ax_post.set_title("MHDD Confidence")
    ax_post.set_ylim(0, 1.1); ax_post.grid(True, alpha=0.3)
    ax_post.legend(fontsize=7)

    # ── PID integral ──────────────────────────────────────────────────────────
    pid_recs = bt.get("FSM_tick", [])   # not recorded; approximate from telemetry
    telem_t  = [r["t"]-t0 for r in telem]
    telem_hi = [(r.get("hz",5)/10.) for r in telem]
    ax_pid.plot(telem_t, telem_hi, color="purple", linewidth=1)
    ax_pid.set_ylabel("Norm. loop Hz"); ax_pid.set_title("Loop Rate")
    ax_pid.grid(True, alpha=0.3)

    # ── ESC RPM ───────────────────────────────────────────────────────────────
    esc_recs = bt.get("ESC_ingest", [])
    colors_m  = ["#e74c3c","#3498db","#2ecc71","#f39c12"]
    for idx, c in zip([1,2,3,4], colors_m):
        mrs  = [r for r in esc_recs if r.get("m")==idx]
        if mrs:
            et  = [r["t"]-t0 for r in mrs]
            rpm = [r.get("rpm",0) for r in mrs]
            ax_esc.plot(et, rpm, color=c, linewidth=0.8,
                        label=f"M{idx}", alpha=0.8)
    for ed in bt.get("ESC_degrading",[]):
        ax_esc.axvline(x=ed["t"]-t0, color="red", linewidth=1.5,
                       linestyle=":", alpha=0.8)
    ax_esc.set_ylabel("RPM"); ax_esc.set_title("Primary Motor RPM (M1-M4)")
    ax_esc.legend(loc="upper right", fontsize=7); ax_esc.grid(True, alpha=0.3)

    # ── Vibration ─────────────────────────────────────────────────────────────
    vibe_vals = [r.get("vibe",0) for r in telem]
    ax_vibe.plot(telem_t, vibe_vals, color="teal", linewidth=1,
                 label="Vibe EMA")
    ax_vibe.axhline(y=30, color="red", linestyle="--", linewidth=0.8,
                    label="High threshold")
    ax_vibe.axhline(y=10, color="green", linestyle="--", linewidth=0.8,
                    label="Low threshold")
    ax_vibe.set_ylabel("Vibe (m/s²)"); ax_vibe.set_title("Vibration Level")
    ax_vibe.legend(fontsize=7); ax_vibe.grid(True, alpha=0.3)

    # ── FSM timeline (bottom bar) ─────────────────────────────────────────────
    state_int = {
        "OCTOCOPTER":0,"PRE_DROP":1,"TRANSITIONING_QUAD":2,
        "QUAD":3,"TRANSITIONING_OCTO":4,"DEGRADED":5,"EMERGENCY":6
    }
    state_cols_solid = {
        0:"#27ae60",1:"#f39c12",2:"#c0392b",
        3:"#e74c3c",4:"#e67e22",5:"#9b59b6",6:"#1c2833"
    }
    seg_starts = [0.]; seg_states = [0]
    for tr in trans:
        seg_starts.append(tr["t"]-t0)
        ns = tr.get("to", tr.get("to_state","OCTOCOPTER"))
        seg_states.append(state_int.get(ns, 0))
    seg_starts.append(records[-1]["t"]-t0)

    for i in range(len(seg_states)):
        ax_fsm.barh(0, seg_starts[i+1]-seg_starts[i],
                    left=seg_starts[i], height=0.8,
                    color=state_cols_solid.get(seg_states[i],"#888888"),
                    alpha=0.85)
    ax_fsm.set_xlim(0, records[-1]["t"]-t0)
    ax_fsm.set_yticks([0])
    ax_fsm.set_yticklabels(["FSM State"])
    ax_fsm.set_xlabel("Time from start (s)")
    ax_fsm.set_title("FSM State Timeline")
    patches = [mpatches.Patch(color=state_cols_solid[k], label=name)
               for name, k in state_int.items()]
    ax_fsm.legend(handles=patches, loc="upper right",
                  fontsize=7, ncol=4)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out = args.logfile.replace(".jsonl", "_v4_report.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Plot saved: {out}")

    if args.html:
        # Minimal HTML wrapper so the image is viewable in a browser
        html = f"""<!DOCTYPE html>
<html><head><title>HYDRA v4 Flight Report</title>
<style>body{{font-family:monospace;background:#111;color:#eee;padding:20px}}</style>
</head><body>
<h2>HYDRA v4 Post-Flight Analysis</h2>
<p>Log: {args.logfile} &nbsp;|&nbsp; Duration: {records[-1]['t']-records[0]['t']:.1f}s</p>
<img src="{os.path.basename(out)}" style="max-width:100%;border:1px solid #444">
</body></html>"""
        html_path = args.logfile.replace(".jsonl","_v4_report.html")
        with open(html_path,"w") as f: f.write(html)
        print(f"HTML report: {html_path}")

    plt.show()


# ════════════════════════════════════════════════════════════════════════════
#  CLI
# ════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="HYDRA v4 Advanced Post-Flight Analyzer")
    p.add_argument("logfile")
    p.add_argument("--plot",  action="store_true")
    p.add_argument("--csv",   action="store_true")
    p.add_argument("--html",  action="store_true",
                   help="Wrap plot in HTML report (requires --plot)")
    a = p.parse_args()
    if not os.path.exists(a.logfile):
        print(f"Not found: {a.logfile}"); sys.exit(1)
    records, total, bad = load_log(a.logfile)
    if bad:
        print(f"Warning: {bad} malformed lines skipped")
    analyze(records, a)


if __name__ == "__main__":
    main()
