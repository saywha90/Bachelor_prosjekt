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


def test_main_manual_exposure_uses_main_detection_controls(monkeypatch, tmp_path, capsys):
    calibration_path = _write_homography_calibration(
        tmp_path / "homography_calibration.json",
        dict(SCAN_POSE),
    )
    monkeypatch.setattr(vision_bridge, "_CALIBRATION_FILE", calibration_path)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_MANUAL_EXPOSURE_US", 12000)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_MANUAL_ISO", 200)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_MANUAL_WB_K", 4500)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_POST_APPLY_DISCARD_FRAMES", 8)

    class FakeCamera:
        def __init__(self):
            self.manual_calls = []

        def set_manual_exposure_white_balance(self, exposure_us, iso, white_balance_k=None, discard_frames=0):
            self.manual_calls.append((exposure_us, iso, white_balance_k, discard_frames))
            return True

    bridge = vision_bridge.VisionBridge(use_camera=True)
    bridge._cam = FakeCamera()

    assert bridge.apply_main_manual_exposure() is True

    assert bridge._cam.manual_calls == [(12000, 200, 4500, 8)]
    output = capsys.readouterr().out
    assert "MAIN CAMERA EXPOSURE SETUP" in output
    assert "exposure=12000 µs" in output
    assert "manual exposure applied: 12000 µs" in output


def test_scan_for_balls_reapplies_fixed_manual_exposure_before_capture(monkeypatch, tmp_path, capsys):
    calibration_path = _write_homography_calibration(
        tmp_path / "homography_calibration.json",
        dict(SCAN_POSE),
    )
    monkeypatch.setattr(vision_bridge, "_CALIBRATION_FILE", calibration_path)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_MANUAL_EXPOSURE_US", 12000)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_MANUAL_ISO", 200)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_MANUAL_WB_K", 4500)
    monkeypatch.setattr(vision_bridge.vcfg, "MAIN_DETECTION_POST_APPLY_DISCARD_FRAMES", 8)

    class FakeCamera:
        def __init__(self):
            self.manual_calls = []
            self.read_count = 0

        def set_manual_exposure_white_balance(self, exposure_us, iso, white_balance_k=None, discard_frames=0):
            self.manual_calls.append((exposure_us, iso, white_balance_k, discard_frames))
            return True

        def read(self):
            self.read_count += 1
            import numpy as np
            return True, np.zeros((400, 640, 3), dtype=np.uint8)

    class FakeDetector:
        def __init__(self):
            self.reset_count = 0

        def reset_tracker(self):
            self.reset_count += 1

        def detect_balls(self, frame):
            return [], {}

        def get_statistics(self):
            return {}

    bridge = vision_bridge.VisionBridge(use_camera=True)
    bridge._cam = FakeCamera()
    bridge._detector = FakeDetector()

    assert bridge.scan_for_balls(num_frames=2) == []

    assert bridge._cam.manual_calls == [(12000, 200, 4500, 8)]
    assert bridge._cam.read_count == 2
    assert bridge._detector.reset_count == 1
    assert "MAIN CAMERA EXPOSURE SETUP" not in capsys.readouterr().out
