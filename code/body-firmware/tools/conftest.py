"""Pytest fixtures for tune_pid tests.

Provides:
- ``fake_ws_server`` factory: spins up a localhost WebSocket that emits
  caller-supplied frames in order, optionally with a mid-stream disconnect
  + restart sequence (so we can exercise the capture daemon's reconnect
  + seq-gap path).
- ``synthetic_csv`` factory: writes a session-dir-shaped CSV the
  inspection subcommands can read without needing a live capture.
"""
from __future__ import annotations

import asyncio
import contextlib
import csv
import math
import os
import socket
import threading
from pathlib import Path
from typing import Callable, Iterable, Sequence

import pytest

# Import the column constant from the tool itself so the fixture and the
# subject-under-test cannot drift out of sync.
import sys
TOOLS_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(TOOLS_DIR))


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class FakeWsServer:
    """Tiny websockets server that serves a scripted frame list.

    ``script`` is a list of phases. Each phase is a dict with optional
    keys: ``frames`` (iterable of str), ``close_after`` (bool — close
    after emitting frames), ``delay_before`` (float — wait before
    accepting next connection).
    """

    def __init__(self, port: int, script: list[dict]) -> None:
        self.port = port
        self._script = script
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._connections = 0

    def start(self) -> None:
        ready = threading.Event()

        def run() -> None:
            loop = asyncio.new_event_loop()
            self._loop = loop
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._serve(ready))

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        ready.wait(timeout=5)

    async def _serve(self, ready: threading.Event) -> None:
        import websockets

        async def handler(ws):
            idx = self._connections
            self._connections += 1
            if idx >= len(self._script):
                # No more script — keep open until client disconnects.
                try:
                    await ws.wait_closed()
                except Exception:
                    pass
                return
            phase = self._script[idx]
            for f in phase.get("frames", []):
                await ws.send(f)
                # tiny yield so the client can flush per-frame
                await asyncio.sleep(0.005)
            if phase.get("close_after", True):
                await ws.close()
            else:
                try:
                    await ws.wait_closed()
                except Exception:
                    pass

        async with websockets.serve(handler, "127.0.0.1", self.port):
            ready.set()
            while not self._stop.is_set():
                await asyncio.sleep(0.05)

    def stop(self) -> None:
        self._stop.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)
        if self._thread is not None:
            self._thread.join(timeout=3)


@pytest.fixture
def fake_ws_server() -> Callable[[list[dict]], FakeWsServer]:
    servers: list[FakeWsServer] = []

    def factory(script: list[dict]) -> FakeWsServer:
        port = _free_port()
        srv = FakeWsServer(port, script)
        srv.start()
        servers.append(srv)
        return srv

    yield factory

    for s in servers:
        with contextlib.suppress(Exception):
            s.stop()


# --- CSV helpers ---------------------------------------------------------

# Importing here so we share the exact column ordering with the tool.
from tune_pid import CSV_COLUMNS  # noqa: E402


def _row(seq: int, t_us: int, **overrides) -> dict:
    row = {c: 0 for c in CSV_COLUMNS}
    row["seq"] = seq
    row["t_us"] = t_us
    row.update(overrides)
    return row


def write_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


@pytest.fixture
def synthetic_csv(tmp_path: Path) -> Callable[..., Path]:
    """Return a factory that writes a synthetic telemetry.csv.

    Default builds a 1 kHz tick stream with sinusoidal pitch_actual at
    user-selectable freq, optional event_flags injection per row, and
    optional gain column overrides.
    """

    def make(
        n_rows: int = 1000,
        sample_rate_hz: float = 1000.0,
        pitch_freq_hz: float = 5.0,
        pitch_amplitude: float = 1.0,
        events: dict[int, int] | None = None,
        gain_changes: dict[int, dict] | None = None,
        session: str = "synthetic",
    ) -> Path:
        session_dir = tmp_path / "sessions" / session
        rows = []
        dt = 1.0 / sample_rate_hz
        cur_gains = {
            "pitch_Kp": 1.0,
            "pitch_Ki": 0.0,
            "pitch_Kd": 0.0,
            "roll_Kp": 1.0,
            "roll_Ki": 0.0,
            "roll_Kd": 0.0,
        }
        for i in range(n_rows):
            t = i * dt
            t_us = int(t * 1e6)
            overrides = {
                "pitch_actual": pitch_amplitude * math.sin(2 * math.pi * pitch_freq_hz * t),
                "dt_measured": dt,
                "dt_used": dt,
                **cur_gains,
            }
            if gain_changes and i in gain_changes:
                cur_gains.update(gain_changes[i])
                overrides.update(cur_gains)
            if events and i in events:
                overrides["event_flags"] = events[i]
            rows.append(_row(seq=i, t_us=t_us, **overrides))
        write_csv(session_dir / "telemetry.csv", rows)
        # Minimal manifest so manifest-aware commands can read it.
        import json
        manifest = {
            "session_id": session,
            "host": "fake",
            "started_at": "1970-01-01T00:00:00Z",
            "ended_at": None,
            "gain_history": [],
            "arming_history": [],
            "fault_count": 0,
            "saturation_counts": {},
            "row_count": n_rows,
            "seq_gaps": [],
            "csv_bytes": (session_dir / "telemetry.csv").stat().st_size,
        }
        (session_dir / "manifest.json").write_text(json.dumps(manifest))
        return session_dir

    return make


@pytest.fixture
def csv_header_line() -> str:
    return ",".join(CSV_COLUMNS)
