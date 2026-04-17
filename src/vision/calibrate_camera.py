"""
calibrate_camera.py
===================
Interactive tool to capture the 4 workspace corner pixel coordinates
needed for the homography calibration in vision_bridge.py.

Usage:
    python3 src/vision/calibrate_camera.py

Controls:
    Left-click  — Record a corner point (up to 4)
    'c'         — Clear all points and start over
    'q'         — Quit

After clicking 4 corners, the script draws the workspace polygon and
prints a ready-to-paste WORKSPACE_PX array for vision_bridge.py.

Author: Bachelor Project 2026 – Autonomia
"""

import sys
import cv2
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.oak_camera import OAKCamera
import vision.config as config

# ── State ─────────────────────────────────────────────────────────────
points: list = []          # list of (x, y) tuples in native camera coords
frame_display = None       # mutable reference for the callback

# Display window size — must match cv2.resizeWindow and cv2.resize calls
DISPLAY_W, DISPLAY_H = 1280, 800

COLORS = [
    (0, 255, 0),       # green   — point 1 (top-left)
    (0, 255, 255),     # yellow  — point 2 (top-right)
    (0, 165, 255),     # orange  — point 3 (bottom-right)
    (255, 0, 255),     # magenta — point 4 (bottom-left)
]

LABELS = ["TL (top-left)", "TR (top-right)", "BR (bottom-right)", "BL (bottom-left)"]


def mouse_callback(event, x, y, flags, param):
    """Handle left-click to record a calibration point.

    Mouse coordinates arrive in display-window space (DISPLAY_W × DISPLAY_H).
    We scale them back to the native camera resolution so that the stored
    points are usable for homography computation in vision_bridge.py and the
    overlay dots line up correctly on the camera frame.
    """
    global points, frame_display

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    if len(points) >= 4:
        print("\n  ⚠️  Already have 4 points. Press 'c' to clear and retry.")
        return

    # --- Scale from display coords → native camera coords ---------------
    native_w, native_h = config.CAMERA_RESOLUTION
    nx = int(x * native_w / DISPLAY_W)
    ny = int(y * native_h / DISPLAY_H)

    idx = len(points)
    points.append((nx, ny))
    color = COLORS[idx]
    label = LABELS[idx]

    print(f"  📌 Point {idx + 1}/4  {label}:  ({nx}, {ny})  "
          f"[display ({x}, {y}) → native ({nx}, {ny})]")

    if len(points) == 4:
        _print_result()


def _draw_overlay(frame):
    """Draw recorded points, labels, and polygon on the frame."""
    overlay = frame.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Instructions at the top
    if len(points) < 4:
        hint = f"Click corner {len(points) + 1}/4: {LABELS[len(points)]}"
        cv2.putText(overlay, hint, (10, 30), font, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, "Press 'c' to clear, 'q' to quit", (10, 58),
                    font, 0.5, (180, 180, 180), 1)
    else:
        cv2.putText(overlay, "DONE — polygon shown. Check terminal for output.",
                    (10, 30), font, 0.65, (0, 255, 0), 2)
        cv2.putText(overlay, "Press 'c' to redo, 'q' to quit", (10, 58),
                    font, 0.5, (180, 180, 180), 1)

    # Draw points
    for i, (px, py) in enumerate(points):
        color = COLORS[i]
        cv2.circle(overlay, (px, py), 8, (0, 0, 0), -1)     # shadow
        cv2.circle(overlay, (px, py), 6, color, -1)           # dot
        cv2.putText(overlay, f"{i+1}", (px + 12, py + 5),
                    font, 0.55, color, 2)

    # Draw polygon edges when we have 2+ points
    if len(points) >= 2:
        for i in range(len(points) - 1):
            cv2.line(overlay, points[i], points[i + 1], (200, 200, 200), 2)

    # Close the polygon when all 4 points are placed
    if len(points) == 4:
        cv2.line(overlay, points[3], points[0], (200, 200, 200), 2)

        # Semi-transparent fill
        pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
        fill = overlay.copy()
        cv2.fillPoly(fill, [pts], (0, 255, 0))
        cv2.addWeighted(fill, 0.15, overlay, 0.85, 0, overlay)

    return overlay


def _print_result():
    """Print the formatted WORKSPACE_PX array to the terminal."""
    print("\n" + "=" * 60)
    print("  ✅  ALL 4 CORNERS RECORDED")
    print("=" * 60)
    print()
    print("Copy this into vision_bridge.py:\n")
    print("WORKSPACE_PX = np.float32([")
    for i, (x, y) in enumerate(points):
        label = LABELS[i]
        comma = "," if i < 3 else ""
        print(f"    [{x:>4d}, {y:>4d}]{comma}      # {label}")
    print("])")
    print()
    print("=" * 60)
    print()


def main():
    print("=" * 60)
    print("  WORKSPACE CALIBRATION TOOL")
    print("=" * 60)
    print()
    print("  Click the 4 corners of your physical workspace in order:")
    print("    1. Top-left      (far-left from camera view)")
    print("    2. Top-right     (far-right from camera view)")
    print("    3. Bottom-right  (near-right from camera view)")
    print("    4. Bottom-left   (near-left from camera view)")
    print()
    print(f"  Opening OAK-D camera ({config.CAMERA_RESOLUTION[0]}×"
          f"{config.CAMERA_RESOLUTION[1]})...")

    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)

    if not cam.open():
        print("  ❌ Could not open OAK camera. Check USB connection.")
        return

    print("  ✅ Camera ready\n")

    WINDOW = "Workspace Calibration"
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, DISPLAY_W, DISPLAY_H)
    cv2.setMouseCallback(WINDOW, mouse_callback)

    global points

    while cam.isOpened():
        ret, frame = cam.read()
        if not ret:
            print("  ❌ Failed to read frame")
            break

        display = _draw_overlay(frame)
        display = cv2.resize(display, (DISPLAY_W, DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)
        cv2.imshow(WINDOW, display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("\nQuitting...")
            break

        elif key == ord("c"):
            points = []
            print("\n  🔄 Points cleared — click again\n")

    cam.release()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
