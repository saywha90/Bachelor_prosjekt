"""Tests for the interactive rear-bin route fine-tuning helpers."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

from config.arm import load_transport_route_calibration


MODULE_PATH = Path(__file__).resolve().parents[1] / "src" / "calibration" / "10_bin_calibration.py"
SPEC = importlib.util.spec_from_file_location("bin_route_calibration_tool", MODULE_PATH)
bin_tool = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = bin_tool
SPEC.loader.exec_module(bin_tool)


def _write_json(path: Path, data: dict) -> Path:
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def test_safe_default_schema_is_two_bin_route_and_validates():
    data = bin_tool._safe_default_route_schema()

    assert sorted(data["bins"]) == ["BLUE_BIN", "RED_BIN"]
    assert "REJECT_BIN" not in data["bins"]

    results = bin_tool.validate_all_waypoints(data)
    assert all(result["ok"] for result in results.values())
    assert results["front_neutral"]["validation"]["intent"] == "carry"
    assert results["rear_transfer"]["validation"]["intent"] == "rear_place"


def test_cli_defaults_to_hardware_mode():
    args = bin_tool.parse_args([])

    assert args.hardware is True
    assert args.port == bin_tool.SERIAL_PORT
    assert args.baud == bin_tool.SERIAL_BAUD


def test_cli_dry_run_and_no_hardware_disable_serial_mode():
    dry_run_args = bin_tool.parse_args(["--dry-run"])
    no_hardware_args = bin_tool.parse_args(["--no-hardware"])

    assert dry_run_args.hardware is False
    assert no_hardware_args.hardware is False


def test_validate_only_dry_run_exits_without_opening_serial(tmp_path, monkeypatch):
    path = _write_json(tmp_path / "bin_calibration.json", bin_tool._safe_default_route_schema())
    opened = False

    def fail_open_serial(port, baud):
        nonlocal opened
        opened = True
        raise AssertionError("validate-only should not open serial")

    monkeypatch.setattr(bin_tool, "_open_serial", fail_open_serial)

    exit_code = bin_tool.main(["--file", str(path), "--validate-only"])

    assert exit_code == 0
    assert opened is False


def test_dry_run_interactive_quit_does_not_open_serial(tmp_path, monkeypatch):
    path = _write_json(tmp_path / "bin_calibration.json", bin_tool._safe_default_route_schema())
    opened = False

    def fail_open_serial(port, baud):
        nonlocal opened
        opened = True
        raise AssertionError("dry-run should not open serial")

    monkeypatch.setattr(bin_tool, "_open_serial", fail_open_serial)
    monkeypatch.setattr(bin_tool, "interactive_loop", lambda data, *, hardware, ser: False)

    exit_code = bin_tool.main(["--file", str(path), "--dry-run"])

    assert exit_code == 0
    assert opened is False


def test_load_or_initialize_drops_reject_bin_from_route_schema(tmp_path):
    data = bin_tool._safe_default_route_schema()
    data["bins"]["REJECT_BIN"] = {
        "approach": {"x": -24.0, "y": 0.0, "z": 38.0},
        "drop": {"x": -24.0, "y": 0.0, "z": 33.0},
    }
    path = _write_json(tmp_path / "bin_calibration.json", data)

    loaded, messages = bin_tool.load_or_initialize_route_schema(path)

    assert sorted(loaded["bins"]) == ["BLUE_BIN", "RED_BIN"]
    assert "REJECT_BIN" not in loaded["bins"]
    assert any("Ignored non-production" in message for message in messages)


def test_load_or_initialize_uses_safe_defaults_for_legacy_incomplete_file(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {"bins": {"RED_BIN": {"x": -24.0, "y": -7.0, "z": 33.0}}},
    )

    loaded, messages = bin_tool.load_or_initialize_route_schema(path)

    assert loaded["schema_version"] == 2
    assert sorted(loaded["bins"]) == ["BLUE_BIN", "RED_BIN"]
    assert "shared_waypoints" in loaded
    assert any("safe two-bin route defaults" in message for message in messages)


def test_update_waypoint_field_reverts_when_candidate_is_invalid():
    data = bin_tool._safe_default_route_schema()
    spec = bin_tool.EDITABLE_WAYPOINTS[0]

    bin_tool.update_waypoint_field(data, spec, "x", 1.25)
    assert data["shared_waypoints"]["front_neutral"]["x"] == 23.25

    bin_tool.update_waypoint_field(data, spec, "m4_offset", 12.6)
    assert data["shared_waypoints"]["front_neutral"]["m4_offset"] == 13


def test_strict_validation_fails_for_rear_yaw_outside_limit():
    data = bin_tool._safe_default_route_schema()
    spec = next(item for item in bin_tool.EDITABLE_WAYPOINTS if item.key == "RED_BIN.drop")
    data["bins"]["RED_BIN"]["drop"]["y"] = 50.0

    result = bin_tool.validate_waypoint(data, spec)

    assert not result["ok"]
    assert "base yaw" in result["reason"]


def test_save_route_schema_creates_backup_and_writes_no_reject_bin(tmp_path):
    path = tmp_path / "bin_calibration.json"
    original = {"bins": {"REJECT_BIN": {"x": 1, "y": 2, "z": 3}}}
    _write_json(path, original)
    data = bin_tool._safe_default_route_schema()
    data["bins"]["REJECT_BIN"] = {
        "approach": {"x": -24.0, "y": 0.0, "z": 38.0},
        "drop": {"x": -24.0, "y": 0.0, "z": 33.0},
    }

    backup = bin_tool.save_route_schema_with_backup(data, path)

    assert backup is not None
    assert backup.exists()
    assert json.loads(backup.read_text(encoding="utf-8")) == original
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert sorted(saved["bins"]) == ["BLUE_BIN", "RED_BIN"]
    assert "REJECT_BIN" not in saved["bins"]

    route = load_transport_route_calibration(path, require_route_schema=True)
    assert sorted(route.bins) == ["BLUE_BIN", "RED_BIN"]
