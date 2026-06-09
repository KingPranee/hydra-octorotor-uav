#!/usr/bin/env python3
"""
blackbox_verify.py
══════════════════
Project HYDRA v4 — Cryptographic Black-Box Log Verifier

Verifies the HMAC-SHA256 chain of a v4 JSONL log file.
Each record carries a "_mac" field = HMAC(key, prev_mac || payload).
Key is derived from the FC UID embedded in the system_start record.

Usage
─────
  python3 blackbox_verify.py <logfile.jsonl> [--fc-uid <hex>] [--fix]

  --fc-uid   Override FC UID (hex string) if not present in log
  --fix      Emit a sanitised log with bad records removed
  --stats    Print detailed per-tag statistics
  --export   Export clean timeline to CSV
"""

import argparse
import csv
import hashlib
import hmac
import json
import os
import sys
import time
from collections import defaultdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple


# ════════════════════════════════════════════════════════════════════════════
#  KEY DERIVATION  (must match SecureBlackBox in companion)
# ════════════════════════════════════════════════════════════════════════════

def derive_key(fc_uid_bytes: bytes) -> bytes:
    return hashlib.sha256(b"HYDRA_BB_v4" + fc_uid_bytes).digest()


def compute_mac(key: bytes, prev_mac: bytes, payload: str) -> bytes:
    return hmac.new(key, prev_mac + payload.encode(), hashlib.sha256).digest()


# ════════════════════════════════════════════════════════════════════════════
#  VERIFICATION
# ════════════════════════════════════════════════════════════════════════════

def verify(path: str, key: bytes,
           fix: bool = False,
           fix_path: Optional[str] = None) \
        -> Tuple[int, int, int, List[Dict]]:
    """
    Returns (total, ok, bad_count, bad_records).
    """
    total = ok = bad = 0
    bad_records: List[Dict] = []
    prev_mac = b"\x00" * 32
    clean_lines: List[str] = []

    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            total += 1
            try:
                rec = json.loads(line)
                mac_hex = rec.pop("_mac", "")
                payload = json.dumps(rec, separators=(",", ":"), default=str)
                expected = compute_mac(key, prev_mac, payload)

                if hmac.compare_digest(expected.hex(), mac_hex):
                    ok += 1
                    prev_mac = expected
                    if fix:
                        clean_lines.append(line)
                else:
                    bad += 1
                    bad_records.append({
                        "lineno": lineno,
                        "seq":    rec.get("_seq", "?"),
                        "tag":    rec.get("tag", "?"),
                        "ts":     rec.get("t", 0),
                    })
                    # Don't update prev_mac — chain is broken here
            except json.JSONDecodeError as e:
                bad += 1
                bad_records.append({"lineno": lineno, "error": str(e)})

    if fix and fix_path:
        with open(fix_path, "w") as f:
            f.write("\n".join(clean_lines) + "\n")
        print(f"Clean log written to {fix_path} ({len(clean_lines)} records)")

    return total, ok, bad, bad_records


# ════════════════════════════════════════════════════════════════════════════
#  STATISTICS
# ════════════════════════════════════════════════════════════════════════════

def compute_stats(path: str) -> Dict:
    by_tag = defaultdict(list)
    t_start = t_end = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try:
                rec = json.loads(line)
                tag = rec.get("tag", "unknown")
                by_tag[tag].append(rec)
                ts = rec.get("t", 0)
                if t_start is None or ts < t_start: t_start = ts
                if t_end   is None or ts > t_end:   t_end   = ts
            except Exception:
                pass
    return {
        "by_tag":  dict(by_tag),
        "t_start": t_start or 0,
        "t_end":   t_end   or 0,
    }


def print_stats(stats: Dict) -> None:
    t0   = stats["t_start"]
    t1   = stats["t_end"]
    dur  = t1 - t0
    by_tag = stats["by_tag"]

    print(f"\n{'─'*60}")
    print(f"  Record Statistics")
    print(f"  Duration: {dur:.1f}s  ({dur/60:.1f} min)")
    print(f"{'─'*60}")
    total = sum(len(v) for v in by_tag.values())
    print(f"  {'TAG':<35s} {'COUNT':>7s}  {'RATE':>8s}")
    print(f"  {'─'*50}")
    for tag in sorted(by_tag, key=lambda t: -len(by_tag[t])):
        count = len(by_tag[tag])
        rate  = count / dur if dur > 0 else 0
        print(f"  {tag:<35s} {count:>7d}  {rate:>7.2f}/s")
    print(f"  {'─'*50}")
    print(f"  {'TOTAL':<35s} {total:>7d}")


# ════════════════════════════════════════════════════════════════════════════
#  CSV EXPORT  — clean timeline of key events
# ════════════════════════════════════════════════════════════════════════════

EXPORT_TAGS = {
    "FSM_transition", "FSM_quad_active", "FSM_octo_active",
    "FSM_pre_drop", "FSM_degraded", "emergency_shutdown",
    "MHDD_trigger", "ATE_locked", "TLC_trigger",
    "motor_health", "motor_stuck_spin", "motor_stuck_stop",
    "ESC_degrading", "mixer_quad_applied", "mixer_octo_restored",
    "pid_commit", "pid_force_commit",
    "param_write", "attitude_divergence",
    "pixhawk_hb_stale", "hb_timeout",
    "shutdown_start", "shutdown_complete",
    "system_start", "system_stop",
}

def export_csv(path: str, out_path: str, t_start: float) -> None:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try:
                rec = json.loads(line)
                if rec.get("tag") in EXPORT_TAGS:
                    rec["rel_s"] = round(rec.get("t", 0) - t_start, 3)
                    rows.append(rec)
            except Exception:
                pass
    if not rows:
        print("No exportable records found")
        return
    all_keys = ["rel_s", "t", "tag"]
    for r in rows:
        for k in r:
            if k not in all_keys and not k.startswith("_"):
                all_keys.append(k)
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=all_keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"Timeline exported: {out_path} ({len(rows)} events)")


# ════════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════════

def main() -> None:
    p = argparse.ArgumentParser(
        description="HYDRA v4 Black-Box Cryptographic Verifier")
    p.add_argument("logfile", help="Path to .jsonl log file")
    p.add_argument("--fc-uid", default="",
                   help="FC UID as hex string (overrides value in log)")
    p.add_argument("--fix",    action="store_true",
                   help="Write sanitised log with bad records removed")
    p.add_argument("--stats",  action="store_true",
                   help="Print per-tag record statistics")
    p.add_argument("--export", action="store_true",
                   help="Export key-event timeline to CSV")
    a = p.parse_args()

    if not os.path.exists(a.logfile):
        print(f"File not found: {a.logfile}")
        sys.exit(1)

    # ── Resolve FC UID ────────────────────────────────────────────────────
    fc_uid = bytes.fromhex(a.fc_uid) if a.fc_uid else b""

    if not fc_uid:
        # Try to extract from system_start record in log
        with open(a.logfile) as f:
            for line in f:
                try:
                    rec = json.loads(line.strip())
                    if rec.get("tag") == "system_start":
                        uid_hex = rec.get("fc_uid", "")
                        if uid_hex:
                            fc_uid = bytes.fromhex(uid_hex)
                            print(f"FC UID from log: {uid_hex}")
                            break
                except Exception:
                    pass

    if not fc_uid:
        print("WARNING: No FC UID — using empty key. "
              "Pass --fc-uid <hex> for proper verification.")

    key = derive_key(fc_uid)

    # ── Verify chain ──────────────────────────────────────────────────────
    fix_path = a.logfile.replace(".jsonl", "_clean.jsonl") if a.fix else None
    print(f"Verifying {a.logfile} ...")
    t0_real = time.time()
    total, ok, bad, bad_recs = verify(a.logfile, key, a.fix, fix_path)
    elapsed = time.time() - t0_real

    print(f"\n{'═'*60}")
    print(f"  HYDRA v4 Black-Box Verification Report")
    print(f"{'═'*60}")
    print(f"  File     : {a.logfile}")
    print(f"  FC UID   : {fc_uid.hex() or '(empty)'}")
    print(f"  Records  : {total}")
    print(f"  Valid    : {ok}  ({100*ok/max(1,total):.1f}%)")
    print(f"  Invalid  : {bad}")
    print(f"  Time     : {elapsed*1000:.1f}ms")

    if bad > 0:
        print(f"\n  FAILED RECORDS:")
        for br in bad_recs[:20]:
            print(f"    line={br.get('lineno','?')}  "
                  f"seq={br.get('seq','?')}  "
                  f"tag={br.get('tag','?')}  "
                  f"error={br.get('error','MAC mismatch')}")
        if len(bad_recs) > 20:
            print(f"    ... and {len(bad_recs)-20} more")
    else:
        print("\n  ✓ Chain intact — no tampering or truncation detected")

    print(f"{'═'*60}")

    # ── Statistics ────────────────────────────────────────────────────────
    if a.stats:
        stats = compute_stats(a.logfile)
        print_stats(stats)

    # ── CSV Export ────────────────────────────────────────────────────────
    if a.export:
        stats = compute_stats(a.logfile)
        csv_path = a.logfile.replace(".jsonl", "_events.csv")
        export_csv(a.logfile, csv_path, stats["t_start"])

    sys.exit(0 if bad == 0 else 1)


if __name__ == "__main__":
    main()
