# Calibration Scripts

This directory contains the numbered calibration pipeline for the 4-DOF robotic arm.

## Steps 00 and 01 — Manual setup (no scripts)

Steps 00 and 01 are performed once using external tools and are not automated:

- **Step 00 – Dynamixel Wizard 2.0**: Set servo IDs (1–4 for joints, 5 for claw), baud rate 1 000 000 bps, and operating mode per servo.
- **Step 01 – Arduino IDE**: Flash `firmware/openrb_bridge/openrb_bridge.ino` onto the OpenRB-150 controller.

See [`docs/calibration.md`](../../docs/calibration.md) for full instructions for both manual steps.

## Scripts 02–10 — Calibration and diagnostics (run from project root unless noted)

After Step 02, configure the claw manually as Step 02b: center M5 externally in Dynamixel
Wizard, mount the 3D printed claw while open, slowly close to the desired grip
position, then write the measured values directly into `CLAW_OPEN_POS` and
`CLAW_CLOSED_POS` in `src/config/arm.py`. Use `02b_claw_grip_test.py` as the
Step 02b validation utility for adaptive grip behavior on the real arm.

| Script | Step | Purpose |
|---|---|---|
| `02_joints.py` | 02 | Joint range + direction verification |
| `02b_claw_grip_test.py` | 02b validation | Active adaptive claw/grip diagnostic for validating the configured open/closed claw positions |
| `02c_scan_pose.py` | 02c | Interactive SCAN_POSE tuner: connects to the arm, opens live camera, lets you nudge motors with keyboard and saves the final pose to `src/config/arm.py` |
| `03_sag.py` | 03 | Gravity sag compensation table |
| `04_hsv_tuner.py` | 04 | Interactive HSV colour range tuning |
| `05_hsv_refine.py` | 05 | HSV threshold refinement |
| `06_homography.py` | 06 | Camera-to-workspace homography _(legacy — see `09_touch_calibration.py`)_ |
| `07_vision_offset.py` | 07 | Vision-to-arm offset calibration |
| `08_pick_test.py` | 08 | End-to-end pick success validation |
| `09_touch_calibration.py` | 09 | **Touch-based homography + height/wrist calibration** — arm physically touches corners for accurate px→cm calibration (replaces Step 06). Also records grab height (Z) and wrist tilt (m4 offset) per point; used at runtime by `compute_grab_height()` and `compute_wrist_correction()` to interpolate distance-dependent corrections. Supersedes sag calibration (Step 03) for grab moves. |
| `10_bin_calibration.py` | 10 | Hardware-first rear-bin route calibration and strict route validation |

Run each script with the arm and camera connected:

```bash
python src/calibration/02_joints.py
# … follow on-screen prompts …
python src/calibration/08_pick_test.py
```
