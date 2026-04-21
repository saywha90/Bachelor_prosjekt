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

import json
import sys
import math
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

# ── Make the vision package importable from src/IK/ ──────────────────
_SRC_DIR = str(Path(__file__).resolve().parent.parent)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
# Insert the IK directory LAST at position 0 so that bare `import config`
# resolves to src/IK/config.py, NOT src/vision/config.py.
_IK_DIR = str(Path(__file__).resolve().parent)
if _IK_DIR not in sys.path:
    sys.path.insert(0, _IK_DIR)

from vision.oak_camera import OAKCamera
from vision.enhanced_detector import SimpleBallDetector, BallColor
import vision.config as vcfg

from config import CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_HEIGHT


# ══════════════════════════════════════════════════════════════════════
#  Homography Calibration Points
# ══════════════════════════════════════════════════════════════════════
#
#  ⚠️  CRITICAL: WORKSPACE_CM must be measured from the SHOULDER JOINT
#     (motor 2 pivot), NOT from the camera pillar or arm base!
#     CAMERA_OFFSET_X/Y are now 0 — the homography maps directly to
#     the shoulder-origin frame that the IK solver expects.
#
#  HOW TO CALIBRATE (run once after any physical repositioning):
#
#  1. Place 4 markers at known positions on your workspace (e.g. the
#     corners of an A3 sheet, or tape marks).
#
#  2. Measure each marker's (x, y) in cm FROM THE SHOULDER JOINT:
#       x = forward (away from arm base)
#       y = left(+) / right(−)
#
#  3. Run  python3 calibrate_homography.py  — it shows the camera feed
#     and lets you click each marker to capture pixel coordinates.
#     Alternatively, hover over each marker to read pixel coords.
#
#  4. Fill in the two arrays below so that WORKSPACE_PX[i] corresponds
#     to the same physical corner as WORKSPACE_CM[i].
#
#  Order: top-left, top-right, bottom-right, bottom-left
#  (when looking at the camera image).
#
# ──────────────────────────────────────────────────────────────────────

# ── Default (hardcoded) calibration values ────────────────────────────
# These are used as fallback when no calibration file exists.
# Run  python3 calibrate_homography.py  to generate an updated file.
_DEFAULT_WORKSPACE_PX = np.float32([
    [   9,   17],      # TL (top-left)
    [ 619,   16],      # TR (top-right)
    [ 618,  381],      # BR (bottom-right)
    [  23,  378]       # BL (bottom-left)
])

_DEFAULT_WORKSPACE_CM = np.float32([
    [28.0,  22.0],   # top-left      → 28cm far, 22cm left
    [28.0, -22.0],   # top-right     → 28cm far, 22cm right
    [10.0, -22.0],   # bottom-right  → 10cm near, 22cm right
    [10.0,  22.0],   # bottom-left   → 10cm near, 22cm left
])

# ── Load calibration from JSON (auto-saved by calibrate_homography.py) ─
_CALIBRATION_FILE = Path(__file__).resolve().parent / "homography_calibration.json"


def _load_calibration():
    """Load WORKSPACE_PX and WORKSPACE_CM from the calibration JSON file.

    Returns the saved arrays if the file exists and is valid, otherwise
    falls back to the hardcoded defaults above.
    """
    if _CALIBRATION_FILE.is_file():
        try:
            with open(_CALIBRATION_FILE, "r") as f:
                data = json.load(f)
            px = np.float32(data["workspace_px"])
            cm = np.float32(data["workspace_cm"])
            if px.shape == (4, 2) and cm.shape == (4, 2):
                print(f"[VISION] ✅ Loaded calibration from {_CALIBRATION_FILE.name}")
                return px, cm
            else:
                print(f"[VISION] ⚠️  Calibration file has unexpected shape — using defaults")
        except (json.JSONDecodeError, KeyError, ValueError) as exc:
            print(f"[VISION] ⚠️  Could not parse {_CALIBRATION_FILE.name}: {exc} — using defaults")
    return _DEFAULT_WORKSPACE_PX, _DEFAULT_WORKSPACE_CM


WORKSPACE_PX, WORKSPACE_CM = _load_calibration()


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

        # ── FPS & debug statistics tracking ───────────────────────────
        self._fps_prev_time: float = time.time()
        self._fps_frame_count: int = 0
        self._fps_value: float = 0.0
        self._total_scans: int = 0
        self._conf_history: List[float] = []
        self._CONF_SMOOTH: int = 30  # rolling average window size

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
        """Release camera resources and close any OpenCV display windows."""
        if self._cam is not None:
            self._cam.release()
            self._cam = None
        self._detector = None
        cv2.destroyAllWindows()
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

        # Apply camera mounting offset
        # The homography maps to camera-frame coordinates; shift to shoulder-frame
        x_cm += CAMERA_OFFSET_X
        y_cm += CAMERA_OFFSET_Y

        return round(x_cm, 1), round(y_cm, 1)

    # ── Live camera display ───────────────────────────────────────────

    def _draw_debug_hud(
        self,
        overlay: np.ndarray,
        balls,
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
        if len(self._conf_history) > 500:
            self._conf_history = self._conf_history[-500:]
        recent = self._conf_history[-self._CONF_SMOOTH:] if self._conf_history else []
        avg_conf = int(sum(recent) / len(recent) * 100) if recent else 0
        lines.append((f"Avg confidence: {avg_conf}%", WHITE))

        # Detector statistics (method breakdown)
        if self._detector is not None:
            stats = self._detector.get_statistics()
            lines.append(
                (f"HSV:{stats.get('hsv_detections', 0)}  "
                 f"Hough:{stats.get('hough_detections', 0)}  "
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

    def show_frame(self, frame: np.ndarray, balls) -> None:
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
                "z": 0.0,  # Ball is on table surface; camera height (CAMERA_HEIGHT) used only for calibration reference
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
            # In simulation mode, return the fake detection for this colour
            # so the pick-and-place cycle can proceed normally.
            for det in self._FAKE_DETECTIONS:
                if det["colour"] == approximate_colour:
                    print(f"  📸 [VISION] Correction image — returning fake "
                          f"({det['x']}, {det['y']}) (simulation)")
                    return dict(det)
            print("  📸 [VISION] Correction image — no match (simulation)")
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
