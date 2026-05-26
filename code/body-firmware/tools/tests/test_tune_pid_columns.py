"""Lock-step check: tune_pid.CSV_COLUMNS must exactly match the firmware's
kHeaderLine in src/network/BalanceTelemetryWs.cpp.

Drift here is a wire-protocol break; this test parses the .cpp at test time
so it cannot be fooled by stale copies anywhere else.
"""
from __future__ import annotations

import re
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from tune_pid import CSV_COLUMNS, GAIN_COLS  # noqa: E402


def _firmware_header_columns() -> list[str]:
    """Extract kHeaderLine from BalanceTelemetryWs.cpp and split into columns."""
    cpp = (ROOT / "src/network/BalanceTelemetryWs.cpp").read_text()
    # kHeaderLine = "...""...""..." multi-line literal; capture all string
    # literals between `kHeaderLine =` and the next `;`.
    m = re.search(r"kHeaderLine\s*=\s*(.+?);", cpp, re.DOTALL)
    assert m, "kHeaderLine not found in BalanceTelemetryWs.cpp"
    body = m.group(1)
    parts = re.findall(r'"([^"]*)"', body)
    joined = "".join(parts)
    return joined.split(",")


def test_csv_columns_matches_firmware_header_exactly():
    fw = _firmware_header_columns()
    assert CSV_COLUMNS == fw, (
        f"CSV_COLUMNS ({len(CSV_COLUMNS)}) does not match firmware kHeaderLine "
        f"({len(fw)}). diff:\n"
        f"  only in python: {set(CSV_COLUMNS) - set(fw)}\n"
        f"  only in firmware: {set(fw) - set(CSV_COLUMNS)}\n"
        f"  py:  {CSV_COLUMNS}\n"
        f"  fw:  {fw}"
    )


def test_new_yaw_heading_columns_present():
    expected = {
        "gyro_yaw_rate",
        "heading_integrated",
        "heading_setpoint",
        "heading_err",
        "heading_P",
        "omega_target_raw",
        "omega_target",
        "yaw_rate_err",
        "yaw_rate_P",
        "yaw_rate_I",
        "yaw_rate_D",
        "yaw_rate_out",
        "yaw_rate_Kp",
        "yaw_rate_Ki",
        "yaw_rate_Kd",
        "heading_Kp",
        "gyro_yaw_sign",
    }
    missing = expected - set(CSV_COLUMNS)
    assert not missing, f"new yaw/heading columns missing from CSV_COLUMNS: {missing}"


def test_gain_cols_includes_new_yaw_heading_gains():
    for c in ("yaw_rate_Kp", "yaw_rate_Ki", "yaw_rate_Kd", "heading_Kp"):
        assert c in GAIN_COLS, f"{c} missing from GAIN_COLS"
