"""Tests for vision scan-pose validation metadata."""

import json

from config.arm import CLAW_OPEN_POS, SCAN_POSE
from ik import vision_bridge


def _write_homography_calibration(path, scan_pose):
    path.write_text(
        json.dumps(
            {
                "calibrated_at_scan_pose": scan_pose,
                "tolerance": 50,
                "workspace_px": [[0, 0], [1, 0], [1, 1], [0, 1]],
                "workspace_cm": [[0, 0], [1, 0], [1, 1], [0, 1]],
            }
        ),
        encoding="utf-8",
    )
    return path


def test_vision_scan_pose_validation_derives_m5_from_claw_open(monkeypatch, tmp_path):
    stale_scan_pose = dict(SCAN_POSE)
    stale_scan_pose["m5"] = 2048
    calibration_path = _write_homography_calibration(
        tmp_path / "homography_calibration.json",
        stale_scan_pose,
    )
    monkeypatch.setattr(vision_bridge, "_CALIBRATION_FILE", calibration_path)

    bridge = vision_bridge.VisionBridge(use_camera=False)

    assert bridge._calibrated_scan_pose["m5"] == CLAW_OPEN_POS
    assert bridge._calibrated_scan_pose["m1"] == stale_scan_pose["m1"]
    assert bridge.verify_pose(dict(SCAN_POSE)) is True


def test_vision_scan_pose_validation_rejects_stale_closed_m5(monkeypatch, tmp_path):
    stale_scan_pose = dict(SCAN_POSE)
    stale_scan_pose["m5"] = 2048
    calibration_path = _write_homography_calibration(
        tmp_path / "homography_calibration.json",
        stale_scan_pose,
    )
    monkeypatch.setattr(vision_bridge, "_CALIBRATION_FILE", calibration_path)

    bridge = vision_bridge.VisionBridge(use_camera=False)
    current_positions = dict(SCAN_POSE)
    current_positions["m5"] = 2048

    assert bridge.verify_pose(current_positions) is False

