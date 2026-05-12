"""Tests for the interactive rear-bin route fine-tuning helpers."""

from __future__ import annotations

import importlib.util
import copy
import json
import sys
from pathlib import Path

import pytest

from config.arm import load_transport_route_calibration
from simulation.bin_safety import PHYSICAL_BIN_HEIGHT_CM, SAG_AWARE_BIN_CLEARANCE_CM


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
    assert results["rear_return_lift"]["validation"]["intent"] == "rear_place"
    assert data["shared_waypoints"]["rear_return_lift"]["z"] > data["shared_waypoints"]["rear_transfer"]["z"]


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

    assert loaded["schema_version"] == 4
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


def test_validation_fails_when_claw_pose_touches_bin_volume():
    data = bin_tool._safe_default_route_schema()
    spec = next(item for item in bin_tool.EDITABLE_WAYPOINTS if item.key == "RED_BIN.drop")
    data["bins"]["RED_BIN"]["drop"]["z"] = round(PHYSICAL_BIN_HEIGHT_CM, 2)

    result = bin_tool.validate_waypoint(data, spec)

    assert not result["ok"]
    assert "arm could sit in the bin and break" in result["reason"]
    assert f"height {PHYSICAL_BIN_HEIGHT_CM:.2f} cm" in result["reason"]


def test_validation_fails_when_claw_pose_has_only_125cm_bin_clearance():
    data = bin_tool._safe_default_route_schema()
    spec = next(item for item in bin_tool.EDITABLE_WAYPOINTS if item.key == "RED_BIN.drop")
    data["bins"]["RED_BIN"]["drop"]["z"] = round(PHYSICAL_BIN_HEIGHT_CM + 1.25, 2)

    result = bin_tool.validate_waypoint(data, spec)

    assert not result["ok"]
    assert "1.25 cm" in result["reason"]
    assert "sag-aware clearance" in result["reason"]


def test_rear_bin_capture_normalizes_full_turn_replay_offset_and_validates(monkeypatch, capsys):
    data = bin_tool._safe_default_route_schema()
    spec = next(item for item in bin_tool.EDITABLE_WAYPOINTS if item.key == "RED_BIN.drop")
    arm = bin_tool.ArmIK()
    captured_positions = {"m1": 2104, "m2": 2207, "m3": 2878, "m4": 3024, "m5": 2527}

    monkeypatch.setattr(bin_tool, "read_motor_positions", lambda _ser: captured_positions)
    monkeypatch.setattr(bin_tool, "enable_limp_mode", lambda _ser: True)

    captured = bin_tool.capture_current_waypoint(data, spec, object(), arm)

    assert captured is True
    pose = data["bins"]["RED_BIN"]["drop"]
    assert pose["x"] == pytest.approx(-29.53)
    assert pose["y"] == pytest.approx(-2.54)
    assert pose["z"] == pytest.approx(36.59)
    assert pose["m4_offset"] == -15

    validation = bin_tool.validate_waypoint(data, spec, arm)
    assert validation["ok"] is True
    assert validation["commands"]["m1"] == captured_positions["m1"]
    assert validation["commands"]["m2"] == captured_positions["m2"]
    assert validation["commands"]["m3"] == captured_positions["m3"]
    assert validation["commands"]["m4"] == captured_positions["m4"]

    output = capsys.readouterr().out
    assert "normalized from +4081" in output
    assert "servo-equivalent -15" in output
    assert "clipped from +4081 to +1500" not in output


def test_default_route_bin_drops_clear_physical_bin_height():
    data = bin_tool._safe_default_route_schema()

    red_clearance = data["bins"]["RED_BIN"]["drop"]["z"] - PHYSICAL_BIN_HEIGHT_CM
    blue_clearance = data["bins"]["BLUE_BIN"]["drop"]["z"] - PHYSICAL_BIN_HEIGHT_CM
    assert red_clearance >= SAG_AWARE_BIN_CLEARANCE_CM
    assert blue_clearance >= SAG_AWARE_BIN_CLEARANCE_CM
    assert red_clearance == pytest.approx(6.5)
    assert blue_clearance == pytest.approx(6.5)


def test_save_route_schema_writes_main_file_without_backup_or_reject_bin(tmp_path):
    path = tmp_path / "bin_calibration.json"
    original = {"bins": {"REJECT_BIN": {"x": 1, "y": 2, "z": 3}}}
    _write_json(path, original)
    data = bin_tool._safe_default_route_schema()
    data["bins"]["REJECT_BIN"] = {
        "drop": {"x": -24.0, "y": 0.0, "z": 33.0},
    }

    result = bin_tool.save_route_schema(data, path)

    assert result is None
    assert not list(tmp_path.glob("bin_calibration.backup_*.json"))
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert sorted(saved["bins"]) == ["BLUE_BIN", "RED_BIN"]
    assert "REJECT_BIN" not in saved["bins"]

    route = load_transport_route_calibration(path, require_route_schema=True)
    assert sorted(route.bins) == ["BLUE_BIN", "RED_BIN"]


def test_gui_captures_current_pose_with_c_key_and_saves_unsaved_data(monkeypatch):
    data = bin_tool._safe_default_route_schema()
    captured_calls = []
    saved_payloads = []
    keys = iter([ord("c"), 13, ord("q")])

    monkeypatch.setattr(bin_tool.cv2, "namedWindow", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(bin_tool.cv2, "imshow", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(bin_tool.cv2, "destroyAllWindows", lambda: None)
    monkeypatch.setattr(bin_tool.cv2, "waitKey", lambda _delay=0: next(keys))

    def fake_capture(route_data, spec, ser, arm):
        captured_calls.append((spec.key, ser, arm))
        bin_tool.get_waypoint(route_data, spec).update(
            {"x": 23.5, "y": 1.25, "z": 31.0, "m4_offset": 42, "skip_sag": True}
        )
        return True

    def fake_save(route_data, path):
        saved_payloads.append(copy.deepcopy(route_data))

    monkeypatch.setattr(bin_tool, "capture_current_waypoint", fake_capture)
    monkeypatch.setattr(bin_tool, "save_route_schema", fake_save)

    saved = bin_tool.interactive_loop(data, hardware=True, ser=object())

    assert saved is True
    assert len(captured_calls) == 1
    assert captured_calls[0][0] == "front_neutral"
    saved_pose = saved_payloads[0]["shared_waypoints"]["front_neutral"]
    assert saved_pose == {"x": 23.5, "y": 1.25, "z": 31.0, "m4_offset": 42, "skip_sag": True}


def test_gui_c_key_refuses_dry_run_capture(monkeypatch):
    data = bin_tool._safe_default_route_schema()
    capture_called = False
    keys = iter([ord("c"), ord("q")])

    monkeypatch.setattr(bin_tool.cv2, "namedWindow", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(bin_tool.cv2, "imshow", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(bin_tool.cv2, "destroyAllWindows", lambda: None)
    monkeypatch.setattr(bin_tool.cv2, "waitKey", lambda _delay=0: next(keys))

    def fake_capture(*_args, **_kwargs):
        nonlocal capture_called
        capture_called = True
        return True

    monkeypatch.setattr(bin_tool, "capture_current_waypoint", fake_capture)

    saved = bin_tool.interactive_loop(data, hardware=False, ser=None)

    assert saved is False
    assert capture_called is False
