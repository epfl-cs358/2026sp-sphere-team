#!/usr/bin/env python3
"""tune_pid: persistent telemetry capture daemon + bounded inspection commands.

The capture daemon is owned by the user (laptop), not Claude — long-lived,
append-only, auto-reconnect with exp backoff. Per-session manifest tracks
gain history, arming history, and seq gaps so reconnects are loss-visible.

The inspection subcommands are designed so an LLM can never accidentally
suck the raw stream into context: every row-emitting command honors
``--max-rows`` (default 200) and ``summary`` always returns the same
top-level JSON shape regardless of window length.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import datetime as dt
import json
import math
import os
import signal
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

# Headless backend — tests run without a display.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CSV_COLUMNS = [
    "seq", "t_us", "dt_measured", "dt_used",
    "cmd_vx_raw", "cmd_vy_raw", "cmd_omega_raw",
    "cmd_vx", "cmd_vy", "cmd_omega", "cmd_age_ms",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "accel_x", "accel_y", "accel_z",
    "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
    "gx", "gy", "gz", "tilt_mag_sin",
    "pitch_actual", "roll_actual",
    "gyro_pitch_rate", "gyro_roll_rate",
    "pitch_target", "roll_target",
    "pitch_err", "pitch_P", "pitch_I", "pitch_D", "pitch_out_raw", "pitch_out",
    "roll_err",  "roll_P",  "roll_I",  "roll_D",  "roll_out_raw",  "roll_out",
    "body_vx_cmd", "body_vy_cmd", "body_omega_cmd",
    "wheel_target_rpm_0", "wheel_target_rpm_1", "wheel_target_rpm_2",
    "wheel_meas_rpm_0",   "wheel_meas_rpm_1",   "wheel_meas_rpm_2",
    "wheel_P_0", "wheel_P_1", "wheel_P_2",
    "wheel_I_0", "wheel_I_1", "wheel_I_2",
    "wheel_D_0", "wheel_D_1", "wheel_D_2",
    "wheel_out_0", "wheel_out_1", "wheel_out_2",
    "pitch_Kp", "pitch_Ki", "pitch_Kd",
    "roll_Kp",  "roll_Ki",  "roll_Kd",
    "pitch_deadband", "roll_deadband",
    "max_output_velocity",
    "envelope_enter_sin", "envelope_exit_sin",
    "gyro_pitch_sign", "gyro_roll_sign",
    "tilt_per_velocity", "max_tilt_setpoint",
    "armed_state", "in_fault", "cmd_stale",
    "event_flags",
]

EVENT_BITS = {
    "ARMED_EDGE": 1 << 0, "DISARMED_EDGE": 1 << 1, "KILLED_EDGE": 1 << 2, "KILL_CLEARED": 1 << 3,
    "FAULT_ENTER": 1 << 4, "FAULT_EXIT": 1 << 5,
    "PITCH_DEADBAND_RESET": 1 << 6, "ROLL_DEADBAND_RESET": 1 << 7,
    "PITCH_I_SATURATED": 1 << 8, "ROLL_I_SATURATED": 1 << 9,
    "PITCH_OUT_SATURATED": 1 << 10, "ROLL_OUT_SATURATED": 1 << 11,
    "GAIN_CHANGED": 1 << 12, "CONFIG_SAVED": 1 << 13, "CONFIG_RESET": 1 << 14,
    "STEP_INJECTED": 1 << 15,
}

DEFAULT_MAX_ROWS = 200
HARD_MAX_ROWS = 2000  # absolute cap even with --downsample
GAIN_COLS = [
    "pitch_Kp", "pitch_Ki", "pitch_Kd",
    "roll_Kp", "roll_Ki", "roll_Kd",
    "pitch_deadband", "roll_deadband",
    "max_output_velocity",
    "envelope_enter_sin", "envelope_exit_sin",
    "tilt_per_velocity", "max_tilt_setpoint",
]


# --- session + manifest --------------------------------------------------


def _now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _root_default() -> Path:
    return Path.home() / ".bb8-telemetry"


def _sessions_dir(root: Path) -> Path:
    return root / "sessions"


def _resolve_session(root: Path, session: str | None) -> Path:
    sessions = _sessions_dir(root)
    if session in (None, "current"):
        cur = root / "current"
        if cur.is_symlink() or cur.exists():
            return cur.resolve()
        # Fallback: newest session by mtime
        if sessions.exists():
            kids = [p for p in sessions.iterdir() if p.is_dir()]
            if kids:
                kids.sort(key=lambda p: p.stat().st_mtime, reverse=True)
                return kids[0]
        raise SystemExit(f"no current session under {root}")
    p = sessions / session
    if not p.exists():
        raise SystemExit(f"session not found: {p}")
    return p


def _load_manifest(session_dir: Path) -> dict:
    mp = session_dir / "manifest.json"
    if mp.exists():
        return json.loads(mp.read_text())
    return {
        "session_id": session_dir.name,
        "host": "",
        "started_at": _now_iso(),
        "ended_at": None,
        "gain_history": [],
        "arming_history": [],
        "fault_count": 0,
        "saturation_counts": {},
        "row_count": 0,
        "seq_gaps": [],
        "csv_bytes": 0,
    }


def _save_manifest(session_dir: Path, manifest: dict) -> None:
    tmp = session_dir / "manifest.json.tmp"
    tmp.write_text(json.dumps(manifest, indent=2))
    tmp.replace(session_dir / "manifest.json")


# --- CAPTURE -------------------------------------------------------------


async def _capture_loop(host: str, port: int, root: Path, base_backoff: float, cap_backoff: float) -> None:
    import websockets  # local import — keeps inspection commands import-light

    session_id = f"{_now_iso()}-{host}"
    session_dir = _sessions_dir(root) / session_id
    session_dir.mkdir(parents=True, exist_ok=True)
    csv_path = session_dir / "telemetry.csv"
    events_path = session_dir / "events.csv"
    header_path = session_dir / "header.csv"
    header_path.write_text(",".join(CSV_COLUMNS) + "\n")

    # Symlink current/ -> this session
    cur = root / "current"
    try:
        if cur.is_symlink() or cur.exists():
            cur.unlink()
    except FileNotFoundError:
        pass
    try:
        cur.symlink_to(session_dir)
    except OSError:
        pass

    manifest = _load_manifest(session_dir)
    manifest["host"] = host
    manifest["session_id"] = session_id
    _save_manifest(session_dir, manifest)

    write_header = not csv_path.exists() or csv_path.stat().st_size == 0
    csv_f = open(csv_path, "a", buffering=1)
    events_f = open(events_path, "a", buffering=1)
    if write_header:
        csv_f.write(",".join(CSV_COLUMNS) + "\n")
        events_f.write(",".join(CSV_COLUMNS) + "\n")

    last_seq: int | None = None
    backoff = base_backoff

    stop = asyncio.Event()

    def _handle_signal() -> None:
        stop.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _handle_signal)
        except NotImplementedError:
            pass

    try:
        while not stop.is_set():
            uri = f"ws://{host}:{port}/telemetry"
            try:
                async with websockets.connect(uri, open_timeout=2, ping_interval=None) as ws:
                    backoff = base_backoff
                    while not stop.is_set():
                        try:
                            msg = await asyncio.wait_for(ws.recv(), timeout=10.0)
                        except asyncio.TimeoutError:
                            continue
                        if isinstance(msg, bytes):
                            msg = msg.decode("utf-8", errors="replace")
                        msg = msg.strip()
                        if not msg:
                            continue
                        # Skip a leading header line (firmware emits one per
                        # connect to make clients column-order agnostic).
                        first = msg.split(",", 1)[0]
                        if first == "seq" or not first.lstrip("-").isdigit():
                            continue
                        seq = int(first)
                        if last_seq is not None and seq > last_seq + 1:
                            gap = {
                                "from": last_seq,
                                "to": seq,
                                "missing": seq - last_seq - 1,
                            }
                            manifest.setdefault("seq_gaps", []).append(gap)
                            _save_manifest(session_dir, manifest)
                        last_seq = seq
                        csv_f.write(msg + "\n")
                        manifest["row_count"] = manifest.get("row_count", 0) + 1
                        # Quick event sniff: 80th field is event_flags
                        parts = msg.split(",")
                        if len(parts) == len(CSV_COLUMNS):
                            try:
                                ef = int(parts[CSV_COLUMNS.index("event_flags")])
                            except ValueError:
                                ef = 0
                            if ef != 0:
                                events_f.write(msg + "\n")
            except (OSError, ConnectionError, asyncio.TimeoutError, Exception) as e:
                # Reconnect with exponential backoff.
                if stop.is_set():
                    break
                try:
                    await asyncio.wait_for(stop.wait(), timeout=backoff)
                except asyncio.TimeoutError:
                    pass
                backoff = min(cap_backoff, backoff * 2)
    finally:
        manifest["ended_at"] = _now_iso()
        if csv_path.exists():
            manifest["csv_bytes"] = csv_path.stat().st_size
        _save_manifest(session_dir, manifest)
        csv_f.close()
        events_f.close()


def cmd_capture(args: argparse.Namespace) -> int:
    root = Path(args.root).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    try:
        asyncio.run(
            _capture_loop(args.host, args.port, root, args.reconnect_base, args.reconnect_cap)
        )
    except KeyboardInterrupt:
        pass
    return 0


# --- shared loader -------------------------------------------------------


def _load_session_df(session_dir: Path) -> pd.DataFrame:
    csv_path = session_dir / "telemetry.csv"
    if not csv_path.exists():
        raise SystemExit(f"no telemetry.csv in {session_dir}")
    df = pd.read_csv(csv_path)
    return df


def _parse_window(w: str) -> float:
    w = w.strip().lower()
    if w.endswith("ms"):
        return float(w[:-2]) / 1000.0
    if w.endswith("s"):
        return float(w[:-1])
    return float(w)


# --- SUMMARY -------------------------------------------------------------

_SUMMARY_AXES = ("pitch", "roll")
_SUMMARY_STAT_KEYS = ("min", "max", "mean", "std")
_SUMMARY_PER_AXIS_FIELDS = ("actual", "target", "err", "P", "I", "D", "out")


def _zero_stats() -> dict:
    return {k: 0.0 for k in _SUMMARY_STAT_KEYS}


def _stats(s: pd.Series) -> dict:
    if len(s) == 0:
        return _zero_stats()
    return {
        "min": float(s.min()),
        "max": float(s.max()),
        "mean": float(s.mean()),
        "std": float(s.std(ddof=0)) if len(s) > 1 else 0.0,
    }


def _axis_block(df: pd.DataFrame, axis: str) -> dict:
    return {
        "actual": _stats(df.get(f"{axis}_actual", pd.Series(dtype=float))),
        "target": _stats(df.get(f"{axis}_target", pd.Series(dtype=float))),
        "err": _stats(df.get(f"{axis}_err", pd.Series(dtype=float))),
        "P": _stats(df.get(f"{axis}_P", pd.Series(dtype=float))),
        "I": _stats(df.get(f"{axis}_I", pd.Series(dtype=float))),
        "D": _stats(df.get(f"{axis}_D", pd.Series(dtype=float))),
        "out": _stats(df.get(f"{axis}_out", pd.Series(dtype=float))),
    }


def _detect_period(series: np.ndarray, sample_dt: float) -> float | None:
    """Autocorrelation period in seconds, or None."""
    n = len(series)
    if n < 16 or sample_dt <= 0:
        return None
    x = series - np.mean(series)
    if np.allclose(x, 0):
        return None
    # FFT-based autocorrelation
    f = np.fft.rfft(x, n=2 * n)
    ac = np.fft.irfft(f * np.conj(f))[:n]
    ac /= ac[0] if ac[0] != 0 else 1.0
    # Find first peak after the zero-crossing.
    zc = None
    for i in range(1, n):
        if ac[i] < 0:
            zc = i
            break
    if zc is None:
        return None
    peak_i = zc + int(np.argmax(ac[zc:]))
    if peak_i <= 0 or peak_i >= n - 1:
        return None
    if ac[peak_i] < 0.2:
        return None
    return float(peak_i * sample_dt)


def _window_slice(df: pd.DataFrame, window_s: float, end: str | None) -> pd.DataFrame:
    if "t_us" not in df.columns or df.empty:
        return df
    if end is None or end == "now":
        end_us = int(df["t_us"].iloc[-1])
    else:
        end_us = int(float(end) * 1e6) + int(df["t_us"].iloc[0])
    start_us = end_us - int(window_s * 1e6)
    return df[(df["t_us"] >= start_us) & (df["t_us"] <= end_us)]


def _event_counts(df: pd.DataFrame) -> dict:
    counts = {name: 0 for name in EVENT_BITS}
    if "event_flags" not in df.columns:
        return counts
    ef = df["event_flags"].astype(int).to_numpy()
    for name, bit in EVENT_BITS.items():
        counts[name] = int(np.sum((ef & bit) != 0))
    return counts


def _saturation_pct(df: pd.DataFrame) -> dict:
    total = max(len(df), 1)
    out = {}
    for name in ("PITCH_OUT_SATURATED", "ROLL_OUT_SATURATED",
                 "PITCH_I_SATURATED", "ROLL_I_SATURATED"):
        bit = EVENT_BITS[name]
        if "event_flags" in df.columns:
            n = int(np.sum((df["event_flags"].astype(int).to_numpy() & bit) != 0))
        else:
            n = 0
        out[name.lower()] = 100.0 * n / total
    return out


def cmd_summary(args: argparse.Namespace) -> int:
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    df = _load_session_df(session_dir)
    win = _parse_window(args.window)
    sub = _window_slice(df, win, args.end)
    sample_dt = float(sub["dt_measured"].mean()) if len(sub) and "dt_measured" in sub.columns else 0.01
    if not (sample_dt > 0):
        sample_dt = 0.01
    pitch_period = (
        _detect_period(sub["pitch_actual"].to_numpy(), sample_dt)
        if "pitch_actual" in sub.columns and len(sub) > 16 else None
    )
    roll_period = (
        _detect_period(sub["roll_actual"].to_numpy(), sample_dt)
        if "roll_actual" in sub.columns and len(sub) > 16 else None
    )
    out = {
        "session_id": session_dir.name,
        "window_s": win,
        "n_rows": int(len(sub)),
        "seq_first": int(sub["seq"].iloc[0]) if len(sub) and "seq" in sub.columns else 0,
        "seq_last": int(sub["seq"].iloc[-1]) if len(sub) and "seq" in sub.columns else 0,
        "pitch": _axis_block(sub, "pitch"),
        "roll": _axis_block(sub, "roll"),
        "oscillation": {
            "pitch_period_s": pitch_period if pitch_period is not None else 0.0,
            "roll_period_s": roll_period if roll_period is not None else 0.0,
        },
        "events": _event_counts(sub),
        "saturation_pct": _saturation_pct(sub),
    }
    sys.stdout.write(json.dumps(out, indent=2) + "\n")
    return 0


# --- EVENTS --------------------------------------------------------------


def _decode_event_flags(flags: int) -> list[str]:
    return [name for name, bit in EVENT_BITS.items() if flags & bit]


def cmd_events(args: argparse.Namespace) -> int:
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    df = _load_session_df(session_dir)
    if args.window:
        sub = _window_slice(df, _parse_window(args.window), None)
    else:
        sub = df
    sub = sub[sub["event_flags"].astype(int) != 0]
    truncated = False
    if len(sub) > args.max_rows:
        truncated = True
        sub = sub.head(args.max_rows)
    rows = []
    for _, r in sub.iterrows():
        flags = int(r["event_flags"])
        rows.append({
            "seq": int(r["seq"]),
            "t_us": int(r["t_us"]),
            "event_flags": flags,
            "events": _decode_event_flags(flags),
            "armed_state": int(r.get("armed_state", 0)),
            "in_fault": int(r.get("in_fault", 0)),
            "pitch_actual": float(r.get("pitch_actual", 0.0)),
            "roll_actual": float(r.get("roll_actual", 0.0)),
        })
    if truncated:
        sys.stderr.write(
            f"events: truncated to {args.max_rows} rows; use --window or --max-rows to adjust\n"
        )
    sys.stdout.write(json.dumps({"rows": rows, "truncated": truncated}, indent=2) + "\n")
    return 0


# --- SLICE ---------------------------------------------------------------


def cmd_slice(args: argparse.Namespace) -> int:
    if args.max_rows > HARD_MAX_ROWS and args.downsample <= 1:
        sys.stderr.write(
            f"slice: --max-rows {args.max_rows} exceeds hard cap {HARD_MAX_ROWS}; "
            "pass --downsample N to take every Nth sample\n"
        )
        return 2
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    df = _load_session_df(session_dir)
    around = args.around
    # If around is an int and matches a seq, anchor by seq; else interpret as t_us.
    try:
        anchor = int(around)
    except ValueError:
        raise SystemExit(f"slice: --around must be integer (t_us or seq), got {around}")
    if "seq" in df.columns and anchor in set(df["seq"].astype(int).tolist()):
        anchor_us = int(df.loc[df["seq"].astype(int) == anchor, "t_us"].iloc[0])
    else:
        anchor_us = anchor
    before_us = int(_parse_window(args.before) * 1e6)
    after_us = int(_parse_window(args.after) * 1e6)
    sub = df[(df["t_us"] >= anchor_us - before_us) & (df["t_us"] <= anchor_us + after_us)]
    if args.downsample > 1:
        sub = sub.iloc[:: args.downsample]
    if args.columns:
        wanted = [c.strip() for c in args.columns.split(",") if c.strip()]
        keep = [c for c in wanted if c in sub.columns]
        keep = list(dict.fromkeys(["seq", "t_us", *keep]))
        sub = sub[keep]
    if len(sub) > args.max_rows:
        sub = sub.head(args.max_rows)
    rows = sub.to_dict(orient="records")
    # Cast numpy types to plain python for json.
    def _cast(v):
        if isinstance(v, (np.integer,)):
            return int(v)
        if isinstance(v, (np.floating,)):
            return float(v)
        return v
    rows = [{k: _cast(v) for k, v in r.items()} for r in rows]
    sys.stdout.write(json.dumps({"rows": rows, "anchor_us": anchor_us}, indent=2) + "\n")
    return 0


# --- PLOT ----------------------------------------------------------------


def cmd_plot(args: argparse.Namespace) -> int:
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    df = _load_session_df(session_dir)
    sub = _window_slice(df, _parse_window(args.window), None) if args.window else df
    if sub.empty:
        sys.stderr.write("plot: empty window\n")
        return 2
    t = (sub["t_us"].to_numpy() - sub["t_us"].iloc[0]) * 1e-6
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, sub["pitch_actual"], label="pitch_actual")
    if "pitch_target" in sub.columns:
        axes[0].plot(t, sub["pitch_target"], label="pitch_target", linestyle="--")
    axes[0].set_ylabel("pitch")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].plot(t, sub["roll_actual"], label="roll_actual")
    if "roll_target" in sub.columns:
        axes[1].plot(t, sub["roll_target"], label="roll_target", linestyle="--")
    axes[1].set_ylabel("roll")
    axes[1].legend(loc="upper right", fontsize=8)
    if "pitch_out" in sub.columns:
        axes[2].plot(t, sub["pitch_out"], label="pitch_out")
    if "roll_out" in sub.columns:
        axes[2].plot(t, sub["roll_out"], label="roll_out")
    axes[2].set_ylabel("out")
    axes[2].set_xlabel("t [s]")
    axes[2].legend(loc="upper right", fontsize=8)
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return 0


# --- OSCILLATION ---------------------------------------------------------


def cmd_oscillation(args: argparse.Namespace) -> int:
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    df = _load_session_df(session_dir)
    win = _parse_window(args.window)
    sub = _window_slice(df, win, None)
    col = f"{args.axis}_actual"
    if col not in sub.columns or len(sub) < 16:
        sys.stderr.write(f"oscillation: insufficient data on {col}\n")
        return 2
    sample_dt = float(sub["dt_measured"].mean())
    if not (sample_dt > 0):
        sample_dt = 0.01
    series = sub[col].to_numpy()
    period = _detect_period(series, sample_dt)
    amp = float((series.max() - series.min()) / 2.0)
    out = {
        "axis": args.axis,
        "window_s": win,
        "period_s": period if period is not None else 0.0,
        "freq_hz": (1.0 / period) if period else 0.0,
        "amplitude": amp,
    }
    sys.stdout.write(json.dumps(out, indent=2) + "\n")
    return 0


# --- STEP RESPONSE -------------------------------------------------------


def _detect_step(target: np.ndarray, dt_meas: np.ndarray) -> int | None:
    """Index of the largest discrete jump in target (dt_measured[i] < 0.2 s).

    Returns None if no row satisfies the discrete-jump criterion.
    """
    if len(target) < 2:
        return None
    diffs = np.abs(np.diff(target))
    dt_ok = dt_meas[1:] < 0.2
    # Mask out non-discrete or zero-diff rows.
    masked = np.where(dt_ok & (diffs > 0), diffs, -1.0)
    if not np.any(masked > 0):
        return None
    return int(np.argmax(masked)) + 1  # +1: diff at i is between i-1 and i


def _step_metrics(t: np.ndarray, actual: np.ndarray, y0: float, y_target: float,
                  delta: float) -> dict:
    """Compute rise/peak/overshoot/settle/decay from the post-step trace.

    `t` is seconds relative to the step instant (t[0] == 0 corresponds to the
    row where the target jumped).
    """
    sign = 1.0 if delta >= 0 else -1.0
    abs_delta = abs(delta)
    progress = (actual - y0) * sign  # signed so the curve always rises
    # Rise time: first crossing of 0.9 * abs_delta
    rise_mask = progress >= 0.9 * abs_delta
    rise_time = float(t[np.argmax(rise_mask)]) if rise_mask.any() else float(t[-1])

    # Peak search window: [0, 5 * rise_time], clamped to available samples
    peak_end = min(len(t), int(np.searchsorted(t, max(rise_time * 5.0, rise_time + 0.05))) + 1)
    peak_end = max(peak_end, 2)
    excess = (actual[:peak_end] - y_target) * sign
    peak_i = int(np.argmax(excess))
    y_peak = float(actual[peak_i])
    overshoot_pct = max(0.0, float(excess[peak_i] / abs_delta) * 100.0)
    time_to_peak = float(t[peak_i])

    # Settling time: last row outside ±5% band.
    band = 0.05 * abs_delta
    outside = np.abs(actual - y_target) > band
    if outside.any():
        last_out = int(np.where(outside)[0][-1])
        settling_time = float(t[last_out])
        settled = settling_time < float(t[-1]) - 1e-9
    else:
        settling_time = 0.0
        settled = True

    # Decay ratio: second peak in the same direction as Δ after a trough.
    # Search after peak_i for a trough, then for the next maximum past it.
    decay_ratio = 0.0
    if peak_i + 2 < len(actual):
        # Find first index after peak_i where excess decreases below 0
        excess_full = (actual - y_target) * sign
        post = excess_full[peak_i:]
        trough_rel = int(np.argmin(post))
        trough_i = peak_i + trough_rel
        if trough_i + 2 < len(actual):
            tail = excess_full[trough_i:]
            second_rel = int(np.argmax(tail))
            second_i = trough_i + second_rel
            if second_i > trough_i and excess_full[second_i] > 0:
                decay_ratio = float(excess_full[second_i] / excess_full[peak_i])

    # Steady-state error: mean of actual over last 200 ms, minus y_target.
    if t[-1] >= 0.2:
        tail_mask = t >= (t[-1] - 0.2)
        ss_mean = float(np.mean(actual[tail_mask])) if tail_mask.any() else float(actual[-1])
    else:
        ss_mean = float(actual[-1])
    ss_error = ss_mean - y_target

    # Zero-crossings of (actual - y_target).
    err = actual - y_target
    signs = np.sign(err)
    n_oscillations = int(np.sum(np.diff(signs) != 0))

    return {
        "rise_time_s": rise_time,
        "time_to_peak_s": time_to_peak,
        "peak": y_peak,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "settled": settled,
        "decay_ratio": decay_ratio,
        "ss_error": ss_error,
        "n_oscillations": n_oscillations,
    }


def _step_response_plot(out: Path, t_rel: np.ndarray, actual: np.ndarray,
                        target: np.ndarray, y_target: float, abs_delta: float,
                        metrics: dict, axis: str) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    ax.plot(t_rel, actual, label=f"{axis}_actual")
    ax.plot(t_rel, target, label=f"{axis}_target", linestyle="--")
    band = 0.05 * abs_delta
    ax.axhline(y_target + band, color="gray", linestyle=":", linewidth=0.8)
    ax.axhline(y_target - band, color="gray", linestyle=":", linewidth=0.8)
    for label, key in (("t_r", "rise_time_s"), ("t_p", "time_to_peak_s"), ("t_s", "settling_time_s")):
        x = metrics.get(key, 0.0)
        ax.axvline(x, color="red", linestyle=":", linewidth=0.8)
        ax.text(x, ax.get_ylim()[1], label, color="red", fontsize=8, va="top")
    ax.set_xlabel("t [s] (relative to step)")
    ax.set_ylabel(axis)
    ax.set_title(
        f"{axis} step response  os={metrics['overshoot_pct']:.1f}%  "
        f"t_r={metrics['rise_time_s']*1000:.0f}ms  t_s={metrics['settling_time_s']*1000:.0f}ms"
    )
    ax.legend(loc="lower right", fontsize=8)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)


def cmd_step_response(args: argparse.Namespace) -> int:
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    df = _load_session_df(session_dir)
    if args.window:
        df = _window_slice(df, _parse_window(args.window), None)
    if df.empty:
        sys.stderr.write("step-response: empty window\n")
        sys.stdout.write(json.dumps({"error": "empty window"}) + "\n")
        return 1
    target_col = f"{args.axis}_target"
    actual_col = f"{args.axis}_actual"
    if target_col not in df.columns or actual_col not in df.columns:
        sys.stderr.write(f"step-response: missing {target_col}/{actual_col}\n")
        sys.stdout.write(json.dumps({"error": "missing columns"}) + "\n")
        return 1

    target = df[target_col].to_numpy()
    actual = df[actual_col].to_numpy()
    t_us = df["t_us"].to_numpy()
    dt_meas = df["dt_measured"].to_numpy() if "dt_measured" in df.columns else np.full(len(df), 0.01)

    if args.at is not None:
        at_us = int(args.at)
        idxs = np.where(t_us >= at_us)[0]
        if len(idxs) == 0 or idxs[0] == 0:
            sys.stderr.write(f"step-response: --at t_us={at_us} outside window\n")
            sys.stdout.write(json.dumps({"error": "at outside window"}) + "\n")
            return 1
        step_i = int(idxs[0])
    else:
        step_i = _detect_step(target, dt_meas)
        if step_i is None:
            sys.stderr.write(
                "step-response: no step detected; widen with --window or pin with --at <t_us>\n"
            )
            sys.stdout.write(json.dumps({"error": "no step detected"}) + "\n")
            return 1

    y0 = float(actual[step_i - 1])
    y_target = float(target[step_i])
    delta = y_target - y0
    if abs(delta) < 1e-4:
        sys.stderr.write("step-response: step too small (<1e-4 rad)\n")
        sys.stdout.write(json.dumps({"error": "step too small"}) + "\n")
        return 1

    t0_us = int(t_us[step_i])
    t_rel = (t_us[step_i:] - t0_us) * 1e-6
    actual_post = actual[step_i:]
    target_post = target[step_i:]
    metrics = _step_metrics(t_rel, actual_post, y0, y_target, delta)

    out_json = {
        "axis": args.axis,
        "session_id": session_dir.name,
        "t0_us": t0_us,
        "target": y_target,
        "y0": y0,
        "step": delta,
        **metrics,
        "window_rows": int(len(df)),
    }

    if args.out:
        _step_response_plot(
            Path(args.out).expanduser(), t_rel, actual_post, target_post,
            y_target, abs(delta), metrics, args.axis,
        )

    sys.stdout.write(json.dumps(out_json, indent=2) + "\n")
    return 0


# --- MANIFEST ------------------------------------------------------------


def _rebuild_manifest(session_dir: Path) -> dict:
    manifest = _load_manifest(session_dir)
    df = _load_session_df(session_dir)
    history: list[dict] = []
    prev: dict[str, float] = {}
    if "event_flags" not in df.columns:
        manifest["gain_history"] = history
        return manifest
    gain_changed_rows = df[df["event_flags"].astype(int) & EVENT_BITS["GAIN_CHANGED"] != 0]
    # Walk every row to track baseline; record diffs at every row where
    # GAIN_CHANGED bit fires AND a tracked gain column moved.
    for col in GAIN_COLS:
        if col in df.columns and len(df) > 0:
            prev[col] = float(df[col].iloc[0])
    for _, r in df.iterrows():
        flags = int(r["event_flags"])
        if flags & EVENT_BITS["GAIN_CHANGED"]:
            for col in GAIN_COLS:
                if col not in df.columns:
                    continue
                cur = float(r[col])
                if col in prev and cur != prev[col]:
                    history.append({
                        "t_us": int(r["t_us"]),
                        "seq": int(r["seq"]),
                        "gain": col,
                        "from": prev[col],
                        "to": cur,
                    })
                    prev[col] = cur
        # Track current value regardless (so a silent shift before the
        # event bit doesn't trip a false diff later).
        for col in GAIN_COLS:
            if col in df.columns:
                prev[col] = float(r[col])
    manifest["gain_history"] = history
    arming = []
    for _, r in df.iterrows():
        flags = int(r["event_flags"])
        for name in ("ARMED_EDGE", "DISARMED_EDGE", "KILLED_EDGE", "KILL_CLEARED"):
            if flags & EVENT_BITS[name]:
                arming.append({"t_us": int(r["t_us"]), "seq": int(r["seq"]), "event": name})
    manifest["arming_history"] = arming
    fault_count = int(np.sum(
        (df["event_flags"].astype(int).to_numpy() & EVENT_BITS["FAULT_ENTER"]) != 0
    ))
    manifest["fault_count"] = fault_count
    manifest["row_count"] = int(len(df))
    manifest["saturation_counts"] = {
        "pitch_I": int(np.sum((df["event_flags"].astype(int).to_numpy() & EVENT_BITS["PITCH_I_SATURATED"]) != 0)),
        "pitch_out": int(np.sum((df["event_flags"].astype(int).to_numpy() & EVENT_BITS["PITCH_OUT_SATURATED"]) != 0)),
        "roll_I": int(np.sum((df["event_flags"].astype(int).to_numpy() & EVENT_BITS["ROLL_I_SATURATED"]) != 0)),
        "roll_out": int(np.sum((df["event_flags"].astype(int).to_numpy() & EVENT_BITS["ROLL_OUT_SATURATED"]) != 0)),
    }
    return manifest


def cmd_manifest(args: argparse.Namespace) -> int:
    session_dir = _resolve_session(Path(args.root).expanduser(), args.session)
    if args.rebuild:
        manifest = _rebuild_manifest(session_dir)
        _save_manifest(session_dir, manifest)
    else:
        manifest = _load_manifest(session_dir)
    sys.stdout.write(json.dumps(manifest, indent=2) + "\n")
    return 0


# --- SESSIONS ------------------------------------------------------------


def cmd_sessions(args: argparse.Namespace) -> int:
    root = Path(args.root).expanduser()
    sessions = _sessions_dir(root)
    out: list[dict] = []
    if sessions.exists():
        for p in sorted(sessions.iterdir()):
            if not p.is_dir():
                continue
            csv_path = p / "telemetry.csv"
            row_count = 0
            if csv_path.exists():
                with open(csv_path) as f:
                    row_count = max(sum(1 for _ in f) - 1, 0)
            manifest_path = p / "manifest.json"
            started = ended = None
            if manifest_path.exists():
                try:
                    m = json.loads(manifest_path.read_text())
                    started = m.get("started_at")
                    ended = m.get("ended_at")
                except Exception:
                    pass
            out.append({
                "session_id": p.name,
                "row_count": row_count,
                "started_at": started,
                "ended_at": ended,
                "path": str(p),
            })
    sys.stdout.write(json.dumps({"sessions": out}, indent=2) + "\n")
    return 0


# --- argparse ------------------------------------------------------------


def _add_common_session_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--root", default=str(_root_default()), help="Telemetry root dir")
    p.add_argument("--session", default="current", help="Session id or 'current'")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="tune_pid", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    pc = sub.add_parser("capture", help="Long-lived capture daemon")
    pc.add_argument("--host", default="bb8.local")
    pc.add_argument("--port", type=int, default=81)
    pc.add_argument("--root", default=str(_root_default()))
    pc.add_argument("--reconnect-base", type=float, default=1.0)
    pc.add_argument("--reconnect-cap", type=float, default=30.0)
    pc.set_defaults(func=cmd_capture)

    ps = sub.add_parser("summary", help="Constant-size JSON over a window")
    _add_common_session_args(ps)
    ps.add_argument("--window", default="10s")
    ps.add_argument("--end", default="now")
    ps.set_defaults(func=cmd_summary)

    pe = sub.add_parser("events", help="Decoded event_flags rows only")
    _add_common_session_args(pe)
    pe.add_argument("--window", default=None)
    pe.add_argument("--max-rows", type=int, default=DEFAULT_MAX_ROWS)
    pe.set_defaults(func=cmd_events)

    psl = sub.add_parser("slice", help="Narrow window, downsampled rows")
    _add_common_session_args(psl)
    psl.add_argument("--around", required=True, help="t_us or seq to anchor on")
    psl.add_argument("--before", default="1s")
    psl.add_argument("--after", default="2s")
    psl.add_argument("--downsample", type=int, default=1)
    psl.add_argument("--columns", default=None, help="comma-separated column allowlist")
    psl.add_argument("--max-rows", type=int, default=DEFAULT_MAX_ROWS)
    psl.set_defaults(func=cmd_slice)

    pp = sub.add_parser("plot", help="Six-panel PNG with event markers")
    _add_common_session_args(pp)
    pp.add_argument("--window", default="30s")
    pp.add_argument("--out", required=True)
    pp.set_defaults(func=cmd_plot)

    po = sub.add_parser("oscillation", help="Autocorrelation period for an axis")
    _add_common_session_args(po)
    po.add_argument("--axis", choices=["pitch", "roll"], required=True)
    po.add_argument("--window", default="10s")
    po.set_defaults(func=cmd_oscillation)

    psr = sub.add_parser("step-response", help="Step-response curve metrics for an axis")
    _add_common_session_args(psr)
    psr.add_argument("--axis", choices=["pitch", "roll"], required=True)
    psr.add_argument("--window", default="30s")
    psr.add_argument("--at", type=int, default=None, help="t_us to pin step (else auto-detect)")
    psr.add_argument("--out", default=None, help="Optional PNG output path")
    psr.set_defaults(func=cmd_step_response)

    pm = sub.add_parser("manifest", help="Per-session manifest.json")
    _add_common_session_args(pm)
    pm.add_argument("--rebuild", action="store_true", help="Rebuild from CSV before printing")
    pm.set_defaults(func=cmd_manifest)

    pls = sub.add_parser("sessions", help="List all captured sessions")
    pls.add_argument("--root", default=str(_root_default()))
    pls.set_defaults(func=cmd_sessions)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
