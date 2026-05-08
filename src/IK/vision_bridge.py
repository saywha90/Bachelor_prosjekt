from __future__ import annotations

"""
vision_bridge.py
================
Bridge between the OAK-D vision system and the IK state machine.

Converts pixel-space ball detections into arm-frame centimetre coordinates
using a **perspective transform (homography)**.

Calibration
-----------
The camera is wrist-mounted and moved to a fixed SCAN_POSE before each
scan.  A homography calibrated at that pose maps any pixel ``(u, v)``
→ ``(x_cm, y_cm)`` on the workspace plane (Z = 0).

Calibration data is stored in ``src/calibration/homography_calibration.json``
and loaded at construction time.  The JSON also records the motor positions
(``calibrated_at_scan_pose``) and tolerance used during calibration so we
can verify the arm is in the correct pose before scanning.

Usage
-----
::

    from vision_bridge import VisionBridge

    bridge = VisionBridge()         # loads homography from JSON
    bridge.open()                   # opens OAK camera
    detections = bridge.scan_for_balls()
    # → [{"colour": "red", "x": 20.3, "y": 5.1, "z": 0.0}, ...]
    bridge.close()

Author: Bachelor Project 2026 – Autonomia
"""

import json
import logging
import sys
import time
from collections import deque
from pathlib import Path
from types import TracebackType
from typing import List, Optional, Tuple

import cv2
import numpy as np

# ── Unified import path ───────────────────────────────────────────────
import os as _os
sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))

from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor, DetectedBall
from config import vision as vcfg

from config.arm import (
    CAMERA_OFFSET_X, CAMERA_OFFSET_Y, SCAN_POSE, SCAN_POSE_TOLERANCE,
    CALIBRATION_FILE, CLAW_OPEN_POS,
)

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════════
#  Homography Calibration Points  (dev-reference defaults)
# ══════════════════════════════════════════════════════════════════════
#
# These hardcoded values are kept as a *reference only*.  At runtime the
# calibration is loaded from ``homography_calibration.json`` (see
# ``VisionBridge.__init__``).  Run  python -m src.calibration.09_touch_calibration
# to regenerate the JSON after any physical repositioning.
#
# _DEFAULT_WORKSPACE_PX = np.float32([
#     [   9,   17],      # TL (top-left)
#     [ 619,   16],      # TR (top-right)
#     [ 618,  381],      # BR (bottom-right)
#     [  23,  378]       # BL (bottom-left)
# ])
#
# _DEFAULT_WORKSPACE_CM = np.float32([
#     [28.0,  22.0],   # top-left      → 28cm far, 22cm left
#     [28.0, -22.0],   # top-right     → 28cm far, 22cm right
#     [10.0, -22.0],   # bottom-right  → 10cm near, 22cm right
#     [10.0,  22.0],   # bottom-left   → 10cm near, 22cm left
# ])

# ── Calibration JSON path (imported from config.arm) ─────────────────
_CALIBRATION_FILE = CALIBRATION_FILE


# Colour → BGR mapping for OpenCV drawing
_COLOUR_BGR = {
    "red":  (0, 0, 255),
    "blue": (255, 130, 0),
}
_WINDOW_NAME = "OAK-D Live View"


class VisionBridge:
    """Adapts the OAK-D vision pipeline for the IK state machine.

    Two modes:
        - ``use_camera=True``  → real OAK-D camera + SimpleBallDetector
        - ``use_camera=False`` → returns canned fake detections (for
          testing the state machine and 3-D visualiser without hardware)

    At construction time, loads the homography calibration from
    ``src/calibration/homography_calibration.json``.  If the JSON also
    contains ``calibrated_at_scan_pose`` and ``tolerance``, those are
    stored for runtime pose verification (see :meth:`verify_pose`).

    Parameters
    ----------
    use_camera : bool
        Set ``True`` to use the real OAK camera.
    """

    # ── Fake detections for simulation mode ───────────────────────────
    _FAKE_DETECTIONS: List[dict] = [
        {"colour": "red",   "x": 20.0, "y":   5.0, "z": 0.0},
        {"colour": "blue",  "x": 25.0, "y": -10.0, "z": 0.0},
    ]

    def __init__(
        self,
        use_camera: bool = False,
    ) -> None:
        self.use_camera = use_camera
        self._cam: Optional[OAKCamera] = None
        self._detector: Optional[SimpleBallDetector] = None
        self._homography: Optional[np.ndarray] = None

        # ── FPS & debug statistics tracking ───────────────────────────
        self._fps_prev_time: float = time.time()
        self._fps_frame_count: int = 0
        self._fps_value: float = 0.0
        self._total_scans: int = 0
        self._conf_history: deque[float] = deque(maxlen=500)
        self._CONF_SMOOTH: int = 30  # rolling average window size

        # ── Load homography calibration from JSON ─────────────────────
        if not _CALIBRATION_FILE.is_file():
            raise FileNotFoundError(
                "No homography calibration found. "
                "Run 'python -m src.calibration.09_touch_calibration' first."
            )

        with open(_CALIBRATION_FILE, "r") as fh:
            cal = json.load(fh)

        workspace_px = np.float32(cal["workspace_px"])
        workspace_cm = np.float32(cal["workspace_cm"])

        # The homography may be pre-computed in the JSON (3×3 list-of-lists)
        # or we can derive it from the four corner pairs.
        if "homography" in cal:
            self._homography = np.float64(cal["homography"])
        else:
            self._homography = cv2.getPerspectiveTransform(
                workspace_px, workspace_cm
            )

        # Scan-pose verification data (backwards-compatible with older JSONs).
        # M1-M4 remain tied to the calibrated homography pose, while M5 is
        # always the configured open-claw position used by SCAN_POSE commands.
        self._calibrated_scan_pose: dict = dict(SCAN_POSE)
        self._calibrated_scan_pose.update(cal.get("calibrated_at_scan_pose", {}))
        calibrated_m5 = self._calibrated_scan_pose.get("m5")
        self._calibrated_scan_pose["m5"] = CLAW_OPEN_POS
        if calibrated_m5 is not None and int(calibrated_m5) != CLAW_OPEN_POS:
            logger.warning(
                "[VISION] Calibration metadata has stale SCAN_POSE m5=%d; "
                "using CLAW_OPEN_POS=%d for scan-pose validation",
                int(calibrated_m5),
                CLAW_OPEN_POS,
            )
        self._scan_pose_tolerance: int = cal.get(
            "tolerance", SCAN_POSE_TOLERANCE
        )

        logger.info(
            "[VISION] ✅ Loaded calibration from %s  "
            "(scan_pose=%s, tolerance=%d)",
            _CALIBRATION_FILE.name,
            self._calibrated_scan_pose,
            self._scan_pose_tolerance,
        )

    # ── Pose verification ─────────────────────────────────────────────

    def verify_pose(self, current_motor_positions: dict) -> bool:
        """Check if the arm is at the calibrated SCAN_POSE within tolerance.

        Args:
            current_motor_positions: dict with keys ``"m1"`` through ``"m5"``
                and int step values.

        Returns:
            True if all motors are within tolerance of the calibrated scan
            pose.
        """
        ok = True
        for motor_key in ("m1", "m2", "m3", "m4", "m5"):
            expected = self._calibrated_scan_pose.get(motor_key)
            actual = current_motor_positions.get(motor_key)
            if expected is None or actual is None:
                continue
            delta = abs(int(actual) - int(expected))
            if delta > self._scan_pose_tolerance:
                logger.warning(
                    "[VISION] ⚠️  Motor %s is %d steps from SCAN_POSE "
                    "(expected %d, got %d, tolerance %d)",
                    motor_key, delta, expected, actual,
                    self._scan_pose_tolerance,
                )
                ok = False
        return ok

    # ── Lifecycle ─────────────────────────────────────────────────────

    def open(self) -> bool:
        """Initialise camera and detector.  No-op in simulation mode.

        Returns
        -------
        bool
            ``True`` if ready (always ``True`` in simulation mode).
        """
        if not self.use_camera:
            logger.info("[VISION] Simulation mode — no camera needed")
            return True

        logger.info("[VISION] Opening OAK-D camera...")
        self._cam = OAKCamera(resolution=vcfg.CAMERA_RESOLUTION)
        if not self._cam.open():
            logger.error("[VISION] ❌ Could not open camera")
            return False

        focal_px = self._cam.get_focal_length_px(hfov_deg=vcfg.CAMERA_HFOV_DEG)
        logger.info(
            f"[VISION] ✅ Camera ready  ({vcfg.CAMERA_RESOLUTION[0]}×"
            f"{vcfg.CAMERA_RESOLUTION[1]}, f={focal_px:.1f}px)"
        )

        self._detector = SimpleBallDetector(
            min_radius=vcfg.BALL_MIN_RADIUS,
            max_radius=vcfg.BALL_MAX_RADIUS,
            confidence_threshold=vcfg.BALL_CONFIDENCE_THRESHOLD,
            enable_adaptive_lighting=True,
            max_balls_per_color=4,
            focal_length_px=focal_px,
        )
        logger.info("[VISION] ✅ Detector initialised")
        return True

    def close(self) -> None:
        """Release camera resources and close any OpenCV display windows."""
        if self._cam is not None:
            self._cam.release()
            self._cam = None
        self._detector = None
        cv2.destroyAllWindows()
        logger.info("[VISION] Camera released")

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

        # Apply camera mounting offset
        # The homography maps to camera-frame coordinates; shift to shoulder-frame
        x_cm += CAMERA_OFFSET_X
        y_cm += CAMERA_OFFSET_Y

        return round(x_cm, 1), round(y_cm, 1)

    # ── Live camera display ───────────────────────────────────────────

    def _draw_debug_hud(
        self,
        overlay: np.ndarray,
        balls: List[DetectedBall],
        fps: float,
    ) -> None:
        """Draw a semi-transparent debug statistics panel in the top-left corner.

        The panel shows FPS, detection count, rolling average confidence,
        tracker status, detector method breakdown, and per-ball detail
        lines including circularity / aspect-ratio / shape & colour scores.

        Parameters
        ----------
        overlay : np.ndarray
            The BGR frame to draw on **in-place**.
        balls : list
            ``DetectedBall`` objects for the current frame.
        fps : float
            Current frames-per-second measurement.
        """
        FONT = cv2.FONT_HERSHEY_SIMPLEX
        FONT_SCALE = 0.45
        THICKNESS = 1
        LINE_H = 18
        PAD_X = 8
        PAD_TOP = 6
        WHITE = (255, 255, 255)
        CYAN = (255, 255, 0)
        YELLOW = (0, 255, 255)
        GREEN = (0, 255, 0)
        RED_C = (0, 0, 255)
        GREY = (180, 180, 180)

        # -- Build text lines -----------------------------------------------
        lines: List[Tuple[str, Tuple[int, int, int]]] = []

        # FPS
        lines.append((f"FPS: {fps:.1f}", GREEN if fps >= 10 else YELLOW))

        # Detection count
        n_red = sum(1 for b in balls if b.color == BallColor.RED)
        n_blue = sum(1 for b in balls if b.color == BallColor.BLUE)
        lines.append((f"Detected: {len(balls)} ball(s)  (R:{n_red} B:{n_blue})", WHITE))

        # Rolling average confidence
        for b in balls:
            self._conf_history.append(b.confidence)
        recent = list(self._conf_history)[-self._CONF_SMOOTH:] if self._conf_history else []
        avg_conf = int(sum(recent) / len(recent) * 100) if recent else 0
        lines.append((f"Avg confidence: {avg_conf}%", WHITE))

        # Detector statistics (method breakdown)
        if self._detector is not None:
            stats = self._detector.get_statistics()
            lines.append(
                (f"HSV:{stats.get('total_hsv_detections', 0)}  "
                 f"Hough:{stats.get('total_hough_detections', 0)}  "
                 f"Ensemble:{stats.get('ensemble_detections', 0)}",
                 GREY)
            )
            light = stats.get('lighting_level', '?')
            lines.append((f"Lighting: {light}", GREY))

        # Tracker status
        active_tracks = sum(1 for b in balls if getattr(b, 'track_id', 0) > 0)
        lines.append((f"Tracker: {active_tracks} active track(s)", CYAN))

        # Total scans
        lines.append((f"Scan #{self._total_scans}", GREY))

        # Separator
        lines.append(("---", WHITE))

        # Per-ball detail lines
        for b in balls:
            colour_name = b.color.value
            if colour_name == "unknown":
                continue
            cx, cy = b.center
            x_cm, y_cm = self.pixel_to_cm(float(cx), float(cy))
            conf_pct = int(b.confidence * 100)
            method = getattr(b, 'detection_method', '?')
            shape_c = getattr(b, 'shape_confidence', 0.0)
            color_c = getattr(b, 'color_confidence', 0.0)
            tid = getattr(b, 'track_id', 0)
            tid_s = f"#{tid}" if tid > 0 else "#?"
            dist_s = f" {b.distance_cm:.0f}cm" if getattr(b, 'distance_cm', None) else ""

            lines.append(
                (f"{tid_s} {colour_name.upper()} {conf_pct}% [{method}]{dist_s}",
                 RED_C if colour_name == "red" else (255, 130, 0))
            )
            lines.append(
                (f"   pos=({x_cm},{y_cm})cm  shp={shape_c:.2f} col={color_c:.2f}",
                 GREY)
            )

        # -- Compute panel dimensions ----------------------------------------
        text_lines = [t for t, _ in lines if t.strip() and t != "---"]
        sep_count = sum(1 for t, _ in lines if t == "---")
        max_w = max(
            (cv2.getTextSize(t, FONT, FONT_SCALE, THICKNESS)[0][0]
             for t in text_lines),
            default=120,
        )
        box_w = min(PAD_X * 2 + max_w + 12, overlay.shape[1])
        box_h = min(
            PAD_TOP * 2 + LINE_H * len(text_lines) + 10 * sep_count + LINE_H,
            overlay.shape[0],
        )

        # -- Draw semi-transparent dark background ---------------------------
        roi = overlay[0:box_h, 0:box_w].copy()
        dark = np.full_like(roi, (10, 10, 10))
        cv2.addWeighted(dark, 0.78, roi, 0.22, 0, roi)
        overlay[0:box_h, 0:box_w] = roi
        cv2.rectangle(overlay, (0, 0), (box_w - 1, box_h - 1), (80, 80, 80), 1)

        # -- Render text lines -----------------------------------------------
        y_pos = PAD_TOP
        for text, color in lines:
            if text == "---":
                y_pos += 5
                cv2.line(overlay, (PAD_X // 2, y_pos),
                         (box_w - PAD_X // 2, y_pos), (60, 60, 60), 1)
                y_pos += 5
                continue
            if not text.strip():
                y_pos += LINE_H // 2
                continue
            y_pos += LINE_H
            # Shadow for readability
            cv2.putText(overlay, text, (PAD_X + 1, y_pos + 1),
                        FONT, FONT_SCALE, (0, 0, 0), THICKNESS + 1, cv2.LINE_AA)
            cv2.putText(overlay, text, (PAD_X, y_pos),
                        FONT, FONT_SCALE, color, THICKNESS, cv2.LINE_AA)

    def show_frame(self, frame: np.ndarray, balls: List[DetectedBall]) -> None:
        """Draw detection overlays on *frame* and display it in an OpenCV window.

        This is a **non-blocking** display helper.  For every detected ball
        it draws circles, colour labels with confidence percentages and
        detection-method tags, and world coordinates.  A debug statistics
        panel is rendered in the top-left corner showing FPS, detection
        counts, rolling average confidence, detector method breakdown,
        tracker status, and per-ball shape/colour quality scores.

        Parameters
        ----------
        frame : np.ndarray
            The BGR camera frame (will be annotated on a **copy**).
        balls : list
            ``DetectedBall`` objects returned by the detector.
        """
        # ── FPS measurement ───────────────────────────────────────────
        self._fps_frame_count += 1
        now = time.time()
        elapsed = now - self._fps_prev_time
        if elapsed >= 1.0:
            self._fps_value = self._fps_frame_count / elapsed
            self._fps_frame_count = 0
            self._fps_prev_time = now

        overlay = frame.copy()

        for ball in balls:
            colour_name = ball.color.value          # "red" / "blue" / "unknown"
            if colour_name == "unknown":
                continue

            cx, cy = ball.center
            radius = int(ball.radius)
            bgr = _COLOUR_BGR.get(colour_name, (200, 200, 200))

            # Circle around the ball (thicker outline + inner ring)
            cv2.circle(overlay, (cx, cy), radius, (0, 0, 0), 4)
            cv2.circle(overlay, (cx, cy), radius, bgr, 2)
            # Small filled dot at centre
            cv2.circle(overlay, (cx, cy), 4, (0, 0, 0), -1)
            cv2.circle(overlay, (cx, cy), 3, (255, 255, 255), -1)

            # World coordinates via homography
            x_cm, y_cm = self.pixel_to_cm(float(cx), float(cy))

            # Per-ball label: COLOUR  conf%  [method]  (x, y) cm
            conf_pct = int(ball.confidence * 100)
            method = getattr(ball, 'detection_method', '')
            method_tag = f" [{method}]" if method else ""
            label = f"{colour_name.upper()} {conf_pct}%{method_tag} ({x_cm}, {y_cm}) cm"

            # Text slightly above the circle with dark shadow for readability
            text_y = max(cy - radius - 8, 14)
            text_x = cx - radius
            cv2.putText(
                overlay, label, (text_x + 1, text_y + 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA,
            )
            cv2.putText(
                overlay, label, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1, cv2.LINE_AA,
            )

        # ── Debug HUD panel (top-left) ────────────────────────────────
        self._draw_debug_hud(overlay, balls, self._fps_value)

        cv2.imshow(_WINDOW_NAME, overlay)
        cv2.waitKey(1)

    # ── Main scanning interface ───────────────────────────────────────

    def scan_for_balls(self, num_frames: int = 5) -> List[dict]:
        """Capture frames, detect balls, and return arm-frame coordinates.

        In camera mode, captures ``num_frames`` and picks the detection
        set with the highest total confidence (reduces single-frame noise).
        The best frame is also displayed in a live OpenCV window with
        detection overlays.

        In simulation mode, returns canned fake detections.

        .. important::

           The caller (typically ``main.py``) is responsible for moving
           the arm to ``SCAN_POSE`` **before** calling this method.
           VisionBridge does not have direct access to the serial/motor
           layer; use :meth:`verify_pose` from the caller if you want
           an explicit check.

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
            logger.info("[VISION] 📷 Returning fake detections (simulation mode)")
            return list(self._FAKE_DETECTIONS)  # shallow copy

        if self._cam is None or self._detector is None:
            logger.error("[VISION] ❌ Camera not opened — call open() first")
            return []

        # ── Start of new scan round ───────────────────────────────────────
        self._total_scans += 1

        # Clear stale Kalman tracks from any previous scan round.
        # Between scans the arm may have picked up / removed a ball, so old
        # tracks would produce phantom detections for the first few frames.
        self._detector.reset_tracker()

        # Capture num_frames and keep the best detection set
        best_balls = []
        best_score = -1.0
        best_frame: Optional[np.ndarray] = None

        for _ in range(num_frames):
            ret, frame = self._cam.read()
            if not ret or frame is None:
                continue

            balls, _ = self._detector.detect_balls(frame)

            score = sum(b.confidence for b in balls)
            if score > best_score:
                best_score = score
                best_balls = balls
                best_frame = frame

        # Show the best frame (or the last captured frame) in the live window
        if best_frame is not None:
            self.show_frame(best_frame, best_balls)

        if not best_balls:
            logger.info("[VISION] 📷 No balls detected")
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
                "z": 0.0,  # Ball is on table surface; camera height (CAMERA_HEIGHT) used only for calibration reference
            })

        colour_summary = ", ".join(
            f"{d['colour'].upper()} at ({d['x']}, {d['y']})"
            for d in detections
        )
        logger.info(f"[VISION] 📷 Detected {len(detections)} ball(s): {colour_summary}")
        return detections

    # refine_detection() removed — wrist-mounted camera occludes ball during approach (see ADR 003)

    # ── Context manager ───────────────────────────────────────────────

    def __enter__(self) -> "VisionBridge":
        if not self.open():
            raise RuntimeError("Failed to open VisionBridge")
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()
