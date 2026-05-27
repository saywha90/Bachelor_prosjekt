"""Tests for touch-calibration temporal ball clustering."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

from vision.detector import BallColor


def _load_touch_calibration_module():
    path = Path(__file__).resolve().parents[1] / "src" / "calibration" / "09_touch_calibration.py"
    spec = importlib.util.spec_from_file_location("touch_calibration_09", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tc = _load_touch_calibration_module()


def _sample(x: float, y: float, frame_idx: int, radius: float = 18.0):
    return tc._CentroidSample(
        x=x,
        y=y,
        radius=radius,
        confidence=0.90,
        color=BallColor.RED,
        frame_idx=frame_idx,
    )


def test_temporal_clustering_keeps_nearby_distinct_expected_balls():
    clusters = []

    # Two physical balls separated by 31 px.  The previous fixed 40 px temporal
    # threshold absorbed this into one cluster even when the user expected both.
    for frame_idx in range(20):
        tc._add_sample_to_clusters(clusters, _sample(100.0 + (frame_idx % 2) * 0.2, 100.0, frame_idx), expected_count=2)
        tc._add_sample_to_clusters(clusters, _sample(131.0 + (frame_idx % 2) * 0.2, 100.0, frame_idx), expected_count=2)

    points = tc._finalize_temporal_clusters(clusters, total_frames=20, expected_count=2)

    assert len(points) == 2
    assert abs(points[0][0] - 100.1) < 1.0
    assert abs(points[1][0] - 131.1) < 1.0


def test_temporal_clustering_rejects_short_lived_false_candidate():
    clusters = []

    for frame_idx in range(20):
        tc._add_sample_to_clusters(clusters, _sample(100.0, 100.0, frame_idx), expected_count=2)
    for frame_idx in range(3):
        tc._add_sample_to_clusters(clusters, _sample(220.0 + frame_idx * 4.0, 40.0, frame_idx), expected_count=2)

    points = tc._finalize_temporal_clusters(clusters, total_frames=20, expected_count=2)

    assert points == [(100.0, 100.0)]


def test_border_artifact_filter_rejects_top_edge_candidate():
    assert tc._is_border_artifact(_sample(327.0, 13.0, 0, radius=18.0), (400, 640, 3))
    assert tc._is_border_artifact(_sample(327.0, 35.2, 0, radius=39.7), (400, 640, 3))
    assert not tc._is_border_artifact(_sample(327.0, 44.0, 0, radius=18.0), (400, 640, 3))


def test_temporal_duplicate_removal_merges_overlapping_split_red_ball():
    clusters = []

    # Glare can split one red/orange ball into two nearby temporal clusters.
    # They are too close relative to their radii to be two physical 50 mm balls,
    # so final duplicate removal should keep the higher-support cluster only.
    for frame_idx in range(26):
        cluster = tc._TemporalBallCluster(color=BallColor.RED)
        x = 508.5 + (frame_idx % 2) * 0.2
        y = 56.0 + (frame_idx % 2) * 0.2
        cluster.add(_sample(x, y, frame_idx, radius=46.6))
        clusters.append(cluster)
        break
    for frame_idx in range(1, 26):
        clusters[0].add(_sample(508.5 + (frame_idx % 2) * 0.2, 56.0, frame_idx, radius=46.6))

    duplicate = tc._TemporalBallCluster(color=BallColor.RED)
    for frame_idx in range(8):
        duplicate.add(_sample(492.6 + (frame_idx % 2) * 0.2, 35.2, frame_idx, radius=39.7))
    clusters.append(duplicate)

    points = tc._finalize_temporal_clusters(clusters, total_frames=40, expected_count=12)

    assert len(points) == 1
    assert abs(points[0][0] - 508.6) < 1.0
    assert abs(points[0][1] - 56.0) < 1.0


def test_prepare_camera_for_calibration_scan_applies_main_matched_manual_exposure(monkeypatch, capsys):
    class FakeCamera:
        def __init__(self):
            self.manual_calls = []
            self.auto_calls = []
            self.lock_calls = []
            self.read_count = 0

        def set_manual_exposure_white_balance(self, exposure_us, iso, white_balance_k=None, discard_frames=0):
            self.manual_calls.append((exposure_us, iso, white_balance_k, discard_frames))
            return True

        def enable_auto_exposure_white_balance(self, discard_frames=0):
            self.auto_calls.append(discard_frames)
            return True

        def lock_auto_exposure_white_balance(self, discard_frames=0):
            self.lock_calls.append(discard_frames)
            return True

        def read(self):
            self.read_count += 1
            return True, np.zeros((400, 640, 3), dtype=np.uint8)

        def discard_frames(self, count):
            return count

    monkeypatch.setattr(tc.vcfg, "CALIBRATION_USE_DIM_MANUAL_EXPOSURE", True)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_MANUAL_EXPOSURE_US", 14000)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_MANUAL_ISO", 200)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_MANUAL_WB_K", 4500)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_POST_APPLY_DISCARD_FRAMES", 8)
    monkeypatch.setattr("builtins.input", lambda _prompt="": "")

    cam = FakeCamera()
    assert tc._prepare_camera_for_calibration_scan(cam)
    assert cam.manual_calls == [(14000, 200, 4500, 8)]
    assert cam.auto_calls == []
    assert cam.lock_calls == []
    assert cam.read_count == 0

    output = capsys.readouterr().out
    assert "Touch calibration camera manual exposure applied" in output
    assert "exposure=14000 µs, ISO=200, WB=4500 K" in output
    assert "discarded 8 post-control frame(s) before auto-detection" in output


def test_prepare_camera_for_calibration_scan_falls_back_to_lock_after_manual_failure(monkeypatch):
    class FakeCamera:
        def __init__(self):
            self.manual_calls = []
            self.auto_calls = []
            self.lock_calls = []
            self.read_count = 0

        def set_manual_exposure_white_balance(self, exposure_us, iso, white_balance_k=None, discard_frames=0):
            self.manual_calls.append((exposure_us, iso, white_balance_k, discard_frames))
            return False

        def enable_auto_exposure_white_balance(self, discard_frames=0):
            self.auto_calls.append(discard_frames)
            return True

        def lock_auto_exposure_white_balance(self, discard_frames=0):
            self.lock_calls.append(discard_frames)
            return True

        def read(self):
            self.read_count += 1
            return True, np.zeros((400, 640, 3), dtype=np.uint8)

        def discard_frames(self, count):
            return count

    monkeypatch.setattr(tc.vcfg, "CALIBRATION_USE_DIM_MANUAL_EXPOSURE", True)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_MANUAL_EXPOSURE_US", 3200)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_MANUAL_ISO", 160)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_MANUAL_WB_K", 4300)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_DIM_POST_APPLY_DISCARD_FRAMES", 7)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_EMPTY_DESK_SETTLE_FRAMES", 3)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_POST_LOCK_DISCARD_FRAMES", 2)
    monkeypatch.setattr("builtins.input", lambda _prompt="": "")

    cam = FakeCamera()
    assert tc._prepare_camera_for_calibration_scan(cam)
    assert cam.manual_calls == [(3200, 160, 4300, 7)]
    assert cam.auto_calls == [2]
    assert cam.lock_calls == [2]
    assert cam.read_count == 3


def test_prepare_camera_for_calibration_scan_can_disable_dim_manual_exposure(monkeypatch):
    class FakeCamera:
        def __init__(self):
            self.manual_calls = []
            self.auto_calls = []
            self.lock_calls = []
            self.read_count = 0

        def set_manual_exposure_white_balance(self, exposure_us, iso, white_balance_k=None, discard_frames=0):
            self.manual_calls.append((exposure_us, iso, white_balance_k, discard_frames))
            return True

        def enable_auto_exposure_white_balance(self, discard_frames=0):
            self.auto_calls.append(discard_frames)
            return True

        def lock_auto_exposure_white_balance(self, discard_frames=0):
            self.lock_calls.append(discard_frames)
            return True

        def read(self):
            self.read_count += 1
            return True, np.zeros((400, 640, 3), dtype=np.uint8)

    monkeypatch.setattr(tc.vcfg, "CALIBRATION_USE_DIM_MANUAL_EXPOSURE", False)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_EMPTY_DESK_SETTLE_FRAMES", 3)
    monkeypatch.setattr(tc.vcfg, "CALIBRATION_POST_LOCK_DISCARD_FRAMES", 2)
    monkeypatch.setattr("builtins.input", lambda _prompt="": "")

    cam = FakeCamera()
    assert tc._prepare_camera_for_calibration_scan(cam)
    assert cam.manual_calls == []
    assert cam.auto_calls == [2]
    assert cam.lock_calls == [2]
    assert cam.read_count == 3


def test_dim_manual_calibration_defaults_are_conservative():
    assert tc.vcfg.CALIBRATION_USE_DIM_MANUAL_EXPOSURE is True
    assert tc.vcfg.CALIBRATION_DIM_MANUAL_EXPOSURE_US == 14000
    assert 100 <= tc.vcfg.CALIBRATION_DIM_MANUAL_ISO <= 300
    assert 3500 <= tc.vcfg.CALIBRATION_DIM_MANUAL_WB_K <= 5500
