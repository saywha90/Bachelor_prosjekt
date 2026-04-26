# Calibration Scripts

This directory contains the numbered calibration pipeline for the 4-DOF robotic arm.

## Steps 00 and 01 — Manual setup (no scripts)

Steps 00 and 01 are performed once using external tools and are not automated:

- **Step 00 – Dynamixel Wizard 2.0**: Set servo IDs (1–4 for joints, 5 for claw), baud rate 1 000 000 bps, and operating mode per servo.
- **Step 01 – Arduino IDE**: Flash `firmware/openrb_bridge/openrb_bridge.ino` onto the OpenRB-150 controller.

See [`docs/calibration.md`](../../docs/calibration.md) for full instructions for both manual steps.

## Steps 02–08 — Automated (run from project root)

| Script | Step | Purpose |
|---|---|---|
| `02_joints.py` | 02 | Joint range + direction verification |
| `02b_claw.py` | 02b | Claw open/close calibration |
| `02c_scan_pose.py` | 02c | Interactive SCAN_POSE tuner: connects to the arm, opens live camera, lets you nudge motors with keyboard and saves the final pose to `src/config/arm.py` |
| `03_sag.py` | 03 | Gravity sag compensation table |
| `04_hsv_tuner.py` | 04 | Interactive HSV colour range tuning |
| `05_hsv_refine.py` | 05 | HSV threshold refinement |
| `06_homography.py` | 06 | Camera-to-workspace homography _(legacy — see `09_touch_calibration.py`)_ |
| `07_vision_offset.py` | 07 | Vision-to-arm offset calibration |
| `08_pick_test.py` | 08 | End-to-end pick success validation |
| `09_touch_calibration.py` | 09 | **Touch-based homography** — arm physically touches corners for accurate calibration (replaces Step 06) |

Run each script with the arm and camera connected:

```bash
python src/calibration/02_joints.py
# … follow on-screen prompts …
python src/calibration/08_pick_test.py
```
