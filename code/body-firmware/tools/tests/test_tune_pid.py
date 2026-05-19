"""Tests for tools/tune_pid.py.

The tool is split into:
- a long-lived ``capture`` daemon, exercised via the fake WS server fixture
- inspection subcommands (``summary``, ``events``, ``slice``, ``plot``,
  ``oscillation``, ``manifest``, ``sessions``) exercised against
  pre-written synthetic CSV sessions

Tests follow the order in the plan; each is a real assertion, not a stub.
"""
from __future__ import annotations

import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
TUNE_PID = TOOLS_DIR / "tune_pid.py"


# --- helpers -------------------------------------------------------------

def run_tool(*args: str, check: bool = False, timeout: float = 30.0) -> subprocess.CompletedProcess:
    cmd = [sys.executable, str(TUNE_PID), *args]
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        check=check,
        timeout=timeout,
        env={**os.environ, "PYTHONPATH": str(TOOLS_DIR)},
    )


def start_capture(host: str, port: int, root: Path) -> subprocess.Popen:
    return subprocess.Popen(
        [
            sys.executable,
            str(TUNE_PID),
            "capture",
            "--host",
            host,
            "--port",
            str(port),
            "--root",
            str(root),
            "--reconnect-base",
            "0.1",
            "--reconnect-cap",
            "0.5",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env={**os.environ, "PYTHONPATH": str(TOOLS_DIR)},
    )


def wait_for_rows(csv_path: Path, n: int, timeout: float = 10.0) -> int:
    """Wait until csv_path has at least n+1 lines (header + n data rows)."""
    end = time.time() + timeout
    while time.time() < end:
        if csv_path.exists():
            with open(csv_path) as f:
                lines = sum(1 for _ in f)
            if lines >= n + 1:
                return lines
        time.sleep(0.05)
    return 0


def stop_proc(p: subprocess.Popen) -> tuple[str, str]:
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            out, err = p.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            out, err = p.communicate()
        return out, err
    out, err = p.communicate()
    return out, err


def _csv_frame(seq: int, **overrides) -> str:
    from tune_pid import CSV_COLUMNS

    row = {c: 0 for c in CSV_COLUMNS}
    row["seq"] = seq
    row["t_us"] = seq * 10000
    row["dt_measured"] = 0.01
    row["dt_used"] = 0.01
    row.update(overrides)
    return ",".join(str(row[c]) for c in CSV_COLUMNS)


# --- tests ---------------------------------------------------------------

def test_capture_writes_csv_to_session_dir(fake_ws_server, tmp_path, csv_header_line):
    frames = [csv_header_line] + [_csv_frame(i) for i in range(10)]
    srv = fake_ws_server([{"frames": frames, "close_after": False}])
    proc = start_capture("127.0.0.1", srv.port, tmp_path)
    try:
        sessions_dir = tmp_path / "sessions"
        # find the session dir as soon as it appears
        end = time.time() + 5.0
        session_dir = None
        while time.time() < end:
            if sessions_dir.exists():
                kids = list(sessions_dir.iterdir())
                if kids:
                    session_dir = kids[0]
                    break
            time.sleep(0.05)
        assert session_dir is not None, "session dir was never created"
        csv_path = session_dir / "telemetry.csv"
        n_lines = wait_for_rows(csv_path, 10, timeout=10.0)
        assert n_lines >= 11
    finally:
        stop_proc(proc)

    # Verify header + row count
    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader)
        data = list(reader)
    assert header == csv_header_line.split(",")
    assert len(data) == 10


def test_capture_reconnects_after_disconnect_records_seq_gap(
    fake_ws_server, tmp_path, csv_header_line
):
    phase1 = [csv_header_line] + [_csv_frame(i) for i in range(6)]  # seq 0..5
    phase2 = [csv_header_line] + [_csv_frame(i) for i in range(20, 25)]  # 20..24
    srv = fake_ws_server(
        [
            {"frames": phase1, "close_after": True},
            {"frames": phase2, "close_after": False},
        ]
    )
    proc = start_capture("127.0.0.1", srv.port, tmp_path)
    try:
        sessions_dir = tmp_path / "sessions"
        end = time.time() + 8.0
        session_dir = None
        while time.time() < end:
            if sessions_dir.exists() and any(sessions_dir.iterdir()):
                session_dir = next(sessions_dir.iterdir())
                break
            time.sleep(0.05)
        assert session_dir is not None
        csv_path = session_dir / "telemetry.csv"
        # 6 + 5 = 11 rows
        wait_for_rows(csv_path, 11, timeout=15.0)
    finally:
        time.sleep(0.5)
        stop_proc(proc)

    manifest = json.loads((session_dir / "manifest.json").read_text())
    gaps = manifest.get("seq_gaps", [])
    assert any(
        g.get("from") == 5 and g.get("to") == 20 and g.get("missing") == 14
        for g in gaps
    ), f"expected gap 5->20 missing=14, got {gaps}"


def test_summary_constant_size(synthetic_csv):
    session_dir = synthetic_csv(n_rows=2000)
    root = session_dir.parents[1]
    keys_per_window = []
    for win in ["1s", "10s", "60s"]:
        r = run_tool(
            "summary",
            "--root",
            str(root),
            "--session",
            session_dir.name,
            "--window",
            win,
        )
        assert r.returncode == 0, r.stderr
        j = json.loads(r.stdout)
        keys_per_window.append(set(_walk_keys(j)))
    assert keys_per_window[0] == keys_per_window[1] == keys_per_window[2]


def _walk_keys(obj, prefix: str = "") -> list[str]:
    out = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            p = f"{prefix}.{k}" if prefix else k
            out.append(p)
            out.extend(_walk_keys(v, p))
    return out


def test_summary_computes_pitch_stats(synthetic_csv):
    session_dir = synthetic_csv(n_rows=5000, sample_rate_hz=1000.0, pitch_freq_hz=5.0, pitch_amplitude=2.0)
    root = session_dir.parents[1]
    r = run_tool("summary", "--root", str(root), "--session", session_dir.name, "--window", "5s")
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    pa = j["pitch"]["actual"]
    assert pa["min"] == pytest.approx(-2.0, abs=0.05)
    assert pa["max"] == pytest.approx(2.0, abs=0.05)
    assert pa["mean"] == pytest.approx(0.0, abs=0.05)


def test_slice_enforces_max_rows(synthetic_csv):
    session_dir = synthetic_csv(n_rows=10000)
    root = session_dir.parents[1]
    r = run_tool(
        "slice",
        "--root", str(root),
        "--session", session_dir.name,
        "--around", "5000",
        "--before", "1s",
        "--after", "2s",
        "--max-rows", "100",
        "--downsample", "200",
    )
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    assert len(j["rows"]) <= 100


def test_slice_refuses_unbounded_request(synthetic_csv):
    session_dir = synthetic_csv(n_rows=10000)
    root = session_dir.parents[1]
    r = run_tool(
        "slice",
        "--root", str(root),
        "--session", session_dir.name,
        "--around", "5000",
        "--before", "1s",
        "--after", "2s",
        "--max-rows", "100000",
    )
    assert r.returncode != 0
    assert "downsample" in r.stderr.lower()


def test_events_decodes_bitfield(synthetic_csv):
    # 0x1011 -> bits 0, 4, 12 = ARMED_EDGE | FAULT_ENTER | GAIN_CHANGED
    # The task spec asks for ARMED_EDGE, PITCH_OUT_SATURATED, GAIN_CHANGED
    # which is bits 0, 10, 12 = 0x1401. We honor the spec literally.
    flags = (1 << 0) | (1 << 10) | (1 << 12)  # 0x1401
    session_dir = synthetic_csv(n_rows=100, events={50: flags})
    root = session_dir.parents[1]
    r = run_tool("events", "--root", str(root), "--session", session_dir.name)
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    matches = [row for row in j["rows"] if int(row["seq"]) == 50]
    assert matches, j
    names = set(matches[0]["events"])
    assert {"ARMED_EDGE", "PITCH_OUT_SATURATED", "GAIN_CHANGED"} <= names


def test_events_respects_max_rows_cap(synthetic_csv):
    events = {i: 1 << 0 for i in range(1000)}
    session_dir = synthetic_csv(n_rows=1000, events=events)
    root = session_dir.parents[1]
    r = run_tool(
        "events",
        "--root", str(root),
        "--session", session_dir.name,
        "--max-rows", "50",
    )
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    assert len(j["rows"]) == 50
    assert "trunc" in r.stderr.lower()


def test_oscillation_period_from_synthetic_sine(synthetic_csv):
    session_dir = synthetic_csv(
        n_rows=5000, sample_rate_hz=1000.0, pitch_freq_hz=5.0, pitch_amplitude=1.0
    )
    root = session_dir.parents[1]
    r = run_tool(
        "oscillation",
        "--root", str(root),
        "--session", session_dir.name,
        "--axis", "pitch",
        "--window", "5s",
    )
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    assert j["period_s"] == pytest.approx(0.2, rel=0.05)


def test_manifest_updates_on_gain_change(synthetic_csv):
    GAIN_CHANGED = 1 << 12
    # Pre-change rows have pitch_Kp=1.0; row 100 flips to 1.5 with the bit set.
    session_dir = synthetic_csv(
        n_rows=500,
        events={100: GAIN_CHANGED},
        gain_changes={100: {"pitch_Kp": 1.5}},
    )
    root = session_dir.parents[1]
    r = run_tool(
        "manifest", "--root", str(root), "--session", session_dir.name, "--rebuild",
    )
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    history = j.get("gain_history", [])
    assert any(
        h.get("gain") == "pitch_Kp" and h.get("from") == 1.0 and h.get("to") == 1.5
        for h in history
    ), history


def test_sessions_lists_all_session_dirs(synthetic_csv):
    a = synthetic_csv(n_rows=10, session="a")
    b = synthetic_csv(n_rows=20, session="b")
    c = synthetic_csv(n_rows=30, session="c")
    root = a.parents[1]
    r = run_tool("sessions", "--root", str(root))
    assert r.returncode == 0, r.stderr
    j = json.loads(r.stdout)
    by_name = {s["session_id"]: s for s in j["sessions"]}
    assert set(by_name) == {"a", "b", "c"}
    assert by_name["a"]["row_count"] == 10
    assert by_name["b"]["row_count"] == 20
    assert by_name["c"]["row_count"] == 30


def test_plot_creates_png(synthetic_csv, tmp_path):
    session_dir = synthetic_csv(n_rows=500)
    root = session_dir.parents[1]
    out = tmp_path / "plot.png"
    r = run_tool(
        "plot",
        "--root", str(root),
        "--session", session_dir.name,
        "--window", "0.5s",
        "--out", str(out),
    )
    assert r.returncode == 0, r.stderr
    assert out.exists()
    assert out.stat().st_size > 0
