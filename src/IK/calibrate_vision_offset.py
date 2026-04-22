"""
calibrate_vision_offset.py
==========================
Interactive tool to calibrate CAMERA_OFFSET_X / CAMERA_OFFSET_Y.

Usage
-----
    1. Place a ball at a KNOWN distance from the shoulder joint.
    2. Run this script:  python3 calibrate_vision_offset.py
    3. The camera will detect the ball and print the coordinates it sees.
    4. Compare to your physical measurement and adjust config.py.

The script continuously scans and prints coordinates so you can
move the ball around and verify multiple positions.

Press Ctrl+C to quit.

Author: Bachelor Project 2026 – Autonomia
"""

import sys
import time
from pathlib import Path

import cv2

# Ensure parent packages are importable
_IK_DIR = str(Path(__file__).resolve().parent)
if _IK_DIR not in sys.path:
    sys.path.insert(0, _IK_DIR)

from vision_bridge import VisionBridge
from config import CAMERA_OFFSET_X, CAMERA_OFFSET_Y


def main():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║     VISION OFFSET CALIBRATION TOOL                         ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print(f"║  Current CAMERA_OFFSET_X = {CAMERA_OFFSET_X:>6.1f} cm                      ║")
    print(f"║  Current CAMERA_OFFSET_Y = {CAMERA_OFFSET_Y:>6.1f} cm                      ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  Instructions:                                             ║")
    print("║  1. Place a ball at a known (x, y) from the shoulder joint ║")
    print("║  2. Read the detected coordinates below                    ║")
    print("║  3. Adjust CAMERA_OFFSET_X/Y in config.py until they match ║")
    print("║                                                            ║")
    print("║  x = forward from shoulder, y = left(+) / right(–)        ║")
    print("║  Press Ctrl+C to quit                                      ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    vision = VisionBridge(use_camera=True)
    if not vision.open():
        print("❌ Could not open camera. Check OAK-D connection.")
        return

    scan_num = 0
    try:
        while True:
            scan_num += 1
            detections = vision.scan_for_balls(num_frames=3)

            if not detections:
                print(f"  [{scan_num:3d}] No balls detected — move a ball into view")
            else:
                for det in detections:
                    colour = det["colour"].upper()
                    x = det["x"]
                    y = det["y"]
                    reach = (x**2 + y**2) ** 0.5
                    print(
                        f"  [{scan_num:3d}] {colour:5s}  "
                        f"x = {x:6.1f} cm   y = {y:6.1f} cm   "
                        f"reach = {reach:5.1f} cm"
                    )

            # Keep OpenCV responsive
            cv2.waitKey(500)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n  Shutting down...")

    vision.close()
    print("  Done.\n")

    print("  To adjust the offset, edit config.py:")
    print("    CAMERA_OFFSET_X = <new value>")
    print("    CAMERA_OFFSET_Y = <new value>")
    print()
    print("  Formula: OFFSET = measured_real_x − detected_x_without_offset")


if __name__ == "__main__":
    main()
