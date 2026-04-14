"""
vision_bridge.py
================
Bridge between the OAK-D vision system and the IK state machine.

Converts pixel-space ball detections into arm-frame centimetre coordinates
using a **perspective transform (homography)**.

Calibration
-----------
The camera is mounted on a pillar beside the arm base, looking forward and
slightly downward across the workspace.  A simple pinhole back-projection
would fail because of the oblique viewing angle.

Instead we define four physical corners of the sorting workspace (measured
in cm relative to the arm shoulder origin) and their corresponding pixel
positions in the camera frame.  ``cv2.getPerspectiveTransform`` gives us a
3×3 homography that maps any pixel ``(u, v)`` → ``(x_cm, y_cm)`` on the
workspace plane (Z = 0).

Usage
-----
::

    from vision_bridge import VisionBridge

    bridge = VisionBridge()         # uses defaults or env toggle
    bridge.open()                   # opens OAK camera
    detections = bridge.scan_for_balls()
    # → [{"colour": "red", "x": 20.3, "y": 5.1, "z": 0.0}, ...]
    bridge.close()

Author: Bachelor Project 2026 – Autonomia
"""

import sys
import math
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

# ── Make the vision package importable from src/IK/ ──────────────────
_VISION_DIR = str(Path(__file__).resolve().parent.parent / "vision")
if _VISION_DIR not in sys.path:
    sys.path.insert(0, _VISION_DIR)
_SRC_DIR = str(Path(__file__).resolve().parent.parent)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from vision.oak_camera import OAKCamera
from vision.enhanced_detector import SimpleBallDetector, BallColor
import vision.config as vcfg


# ══════════════════════════════════════════════════════════════════════
#  Homography Calibration Points
# ══════════════════════════════════════════════════════════════════════
#
#  HOW TO CALIBRATE (one-time setup):
#
#  1. Place 4 markers at known positions on your workspace (e.g. the
#     corners of an A3 sheet).  Measure their (x, y) in cm from the
#     arm's shoulder origin.
#
#  2. Run  python src/vision/test_enhanced_detector.py  and hover your
#     mouse over each marker.  OpenCV shows pixel coords in the window
#     title or via cv2.setMouseCallback.
#
#  3. Fill in the two arrays below so that each row in WORKSPACE_PX
#     corresponds to the same physical corner in WORKSPACE_CM.
#
#  Order: top-left, top-right, bottom-right, bottom-left
#  (when looking at the camera image).
#
# ──────────────────────────────────────────────────────────────────────

# Pixel coordinates of the 4 workspace corners in the camera frame
# TODO: Replace with your actual measured pixel positions
WORKSPACE_PX = np.float32([
    [100, 60],       # top-left      in camera image
    [540, 60],       # top-right     in camera image
    [580, 360],      # bottom-right  in camera image
    [60,  360],      # bottom-left   in camera image
])

# Corresponding real-world positions in cm (arm shoulder = origin)
#   x = forward (away from arm base)
#   y = left(+) / right(−)
# TODO: Replace with your actual measured workspace corners
WORKSPACE_CM = np.float32([
    [35.0,  15.0],   # top-left      → far-left of workspace
    [35.0, -15.0],   # top-right     → far-right of workspace
    [10.0, -15.0],   # bottom-right  → near-right of workspace
    [10.0,  15.0],   # bottom-left   → near-left of workspace
])


class VisionBridge:
    """Adapts the OAK-D vision pipeline for the IK state machine.

    Two modes:
        - ``use_camera=True``  → real OAK-D camera + SimpleBallDetector
        - ``use_camera=False`` → returns canned fake detections (for
          testing the state machine and 3-D visualiser without hardware)

    Parameters
    ----------
    use_camera : bool
        Set ``True`` to use the real OAK camera.
    workspace_px : np.ndarray, optional
        4×2 array of workspace corner pixel coordinates.
    workspace_cm : np.ndarray, optional
        4×2 array of corresponding real-world cm coordinates.
    """

    # ── Fake detections for simulation mode ───────────────────────────
    _FAKE_DETECTIONS: List[dict] = [
        {"colour": "red",   "x": 20.0, "y":   5.0, "z": 0.0},
        {"colour": "blue",  "x": 25.0, "y": -10.0, "z": 0.0},
    ]

    def __init__(
        self,
        use_camera: bool = False,
        workspace_px: Optional[np.ndarray] = None,
        workspace_cm: Optional[np.ndarray] = None,
    ):
        self.use_camera = use_camera
        self._cam: Optional[OAKCamera] = None
        self._detector: Optional[SimpleBallDetector] = None
        self._homography: Optional[np.ndarray] = None

        # Build homography matrix from calibration points
        px = workspace_px if workspace_px is not None else WORKSPACE_PX
        cm = workspace_cm if workspace_cm is not None else WORKSPACE_CM
        self._homography = cv2.getPerspectiveTransform(px, cm)

    # ── Lifecycle ─────────────────────────────────────────────────────

    def open(self) -> bool:
        """Initialise camera and detector.  No-op in simulation mode.

        Returns
        -------
        bool
            ``True`` if ready (always ``True`` in simulation mode).
        """
        if not self.use_camera:
            print("[VISION] Simulation mode — no camera needed")
            return True

        print("[VISION] Opening OAK-D camera...")
        self._cam = OAKCamera(resolution=vcfg.CAMERA_RESOLUTION)
        if not self._cam.open():
            print("[VISION] ❌ Could not open camera")
            return False

        focal_px = self._cam.get_focal_length_px(hfov_deg=vcfg.CAMERA_HFOV_DEG)
        print(f"[VISION] ✅ Camera ready  ({vcfg.CAMERA_RESOLUTION[0]}×"
              f"{vcfg.CAMERA_RESOLUTION[1]}, f={focal_px:.1f}px)")

        self._detector = SimpleBallDetector(
            min_radius=vcfg.BALL_MIN_RADIUS,
            max_radius=vcfg.BALL_MAX_RADIUS,
            confidence_threshold=vcfg.BALL_CONFIDENCE_THRESHOLD,
            enable_adaptive_lighting=True,
            max_balls_per_color=4,
            focal_length_px=focal_px,
        )
        print("[VISION] ✅ Detector initialised")
        return True

    def close(self):
        """Release camera resources."""
        if self._cam is not None:
            self._cam.release()
            self._cam = None
        self._detector = None
        print("[VISION] Camera released")

    # ── Coordinate transform ──────────────────────────────────────────

    def pixel_to_cm(self, px: float, py: float) -> Tuple[float, float]:
        """Map a pixel coordinate to workspace cm via the homography.

        Parameters
        ----------
        px, py : float
            Pixel position in the camera frame.

        Returns
        -------
        (x_cm, y_cm) : tuple of float
            Position in cm relative to the arm shoulder origin (Z = 0 plane).
        """
        point = np.float32([[[px, py]]])
        transformed = cv2.perspectiveTransform(point, self._homography)
        x_cm = float(transformed[0, 0, 0])
        y_cm = float(transformed[0, 0, 1])
        return round(x_cm, 1), round(y_cm, 1)

    # ── Main scanning interface ───────────────────────────────────────

    def scan_for_balls(self, num_frames: int = 5) -> List[dict]:
        """Capture frames, detect balls, and return arm-frame coordinates.

        In camera mode, captures ``num_frames`` and picks the detection
        set with the highest total confidence (reduces single-frame noise).

        In simulation mode, returns canned fake detections.

        Parameters
        ----------
        num_frames : int
            Number of frames to capture and pick-best from.

        Returns
        -------
        list of dict
            Each dict has keys ``"colour"`` (str), ``"x"``, ``"y"``,
            ``"z"`` (float, cm).  Ready for ``run_sorting_cycle()``.
        """
        if not self.use_camera:
            print("[VISION] 📷 Returning fake detections (simulation mode)")
            return list(self._FAKE_DETECTIONS)  # shallow copy

        if self._cam is None or self._detector is None:
            print("[VISION] ❌ Camera not opened — call open() first")
            return []

        # Capture num_frames and keep the best detection set
        best_balls = []
        best_score = -1.0

        for _ in range(num_frames):
            ret, frame = self._cam.read()
            if not ret or frame is None:
                continue

            balls, _ = self._detector.detect_balls(frame)

            score = sum(b.confidence for b in balls)
            if score > best_score:
                best_score = score
                best_balls = balls

        if not best_balls:
            print("[VISION] 📷 No balls detected")
            return []

        # Convert DetectedBall objects → arm-frame dicts
        detections = []
        for ball in best_balls:
            colour = ball.color.value  # "red" or "blue"
            if colour == "unknown":
                continue

            px, py = ball.center
            x_cm, y_cm = self.pixel_to_cm(float(px), float(py))

            detections.append({
                "colour": colour,
                "x": x_cm,
                "y": y_cm,
                "z": 0.0,       # balls are on the table surface
            })

        colour_summary = ", ".join(
            f"{d['colour'].upper()} at ({d['x']}, {d['y']})"
            for d in detections
        )
        print(f"[VISION] 📷 Detected {len(detections)} ball(s): {colour_summary}")
        return detections

    # ── Visual servoing helper ────────────────────────────────────────

    def refine_detection(self, approximate_colour: str) -> Optional[dict]:
        """Take a fresh image and return the best detection matching *colour*.

        Used during the APPROACHING state (after the 80% move) to get an
        updated position for the final 20% correction.

        Parameters
        ----------
        approximate_colour : str
            ``"red"`` or ``"blue"`` — filters detections to this colour only.

        Returns
        -------
        dict or None
            Updated ``{"colour", "x", "y", "z"}`` or ``None`` if the ball
            is no longer visible (arm may be occluding it).
        """
        if not self.use_camera:
            # In simulation mode, pretend the ball hasn't moved
            print("  📸 [VISION] Correction image — no change (simulation)")
            return None

        detections = self.scan_for_balls(num_frames=3)
        matches = [d for d in detections if d["colour"] == approximate_colour]

        if not matches:
            print(f"  📸 [VISION] Correction — {approximate_colour} ball not visible")
            return None

        # Return the highest-confidence match (scan_for_balls already
        # uses pick-best logic, so just take the first)
        best = matches[0]
        print(f"  📸 [VISION] Correction — {approximate_colour} now at "
              f"({best['x']}, {best['y']})")
        return best

    # ── Context manager ───────────────────────────────────────────────

    def __enter__(self) -> "VisionBridge":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
