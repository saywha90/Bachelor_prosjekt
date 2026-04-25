# Calibration Guide

> **Autonomia — Autonomous Sorting Robot Arm**
> Bachelor Project 2026

Complete calibration pipeline for first-time setup and recalibration.
Follow Phases A → B → C in order — each step depends on the ones before it.
Total time: approximately 55–80 minutes.

---

## Prerequisites

### Hardware

| Component | Requirement |
|-----------|-------------|
| Robot arm | Fully assembled, 5 × Dynamixel motors (IDs 1–5) daisy-chained |
| Controller board | OpenRB-150 connected via USB-C |
| Power supply | 12 V DC to the OpenRB-150 power jack |
| Camera | Luxonis OAK Series 2 (IMX378), USB-C to host |
| Host computer | Raspberry Pi 5 (or laptop for initial calibration) |
| Workspace surface | Flat, well-lit table (300–700 lux recommended) |
| Balls | Red and blue, 50 mm diameter |
| Ruler / tape measure | For sag and homography measurements |
| U2D2 adapter | Only needed for Step 0 (motor setup) |

### Software

```bash
pip3 install -r requirements.txt
```

Required packages: `numpy`, `opencv-python`, `pyserial`, `depthai`, `scikit-learn`, `matplotlib`.

### Tools

- [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) — for Step 0 only
- Arduino IDE with **ROBOTIS** board support — for Step 1 only

---

## Calibration Overview

| Step | Phase | Script | What It Does | Time |
|:---:|:---:|---|---|---|
| **0** | A | Dynamixel Wizard 2.0 | Set motor IDs, baudrate, operating mode | ~10 min |
| **1** | A | `python src/diagnostics/diagnose_motors.py` | System check — verify all motors respond | ~2 min |
| **2** | A | `python src/calibration/02_joints.py` | Joint calibration — verify motor signs and zeros | ~10–15 min |
| **2b** | A | `python src/calibration/02b_claw.py` | Claw open/close calibration | ~2 min |
| **2c** | A | `python src/calibration/02c_scan_pose.py` | Tune SCAN_POSE for wrist-mounted camera | ~5–10 min |
| **3** | A | `python src/calibration/03_sag.py` | Sag (droop) compensation calibration | ~10–20 min |
| **4** | B | `python src/calibration/04_hsv_tuner.py` | Interactive HSV colour tuning | ~5–15 min |
| **5** | B | `python src/calibration/05_hsv_refine.py` | Statistical HSV refinement | ~5 min |
| **6** | B | `python src/calibration/06_homography.py` | Pixel-to-cm homography calibration | ~5–10 min |
| **6b** | B | `python src/calibration/06b_workspace.py` | Camera height and scan region verification | ~3 min |
| **7** | C | `python src/calibration/07_vision_offset.py` | Fine-tune residual camera-to-shoulder offset | ~5 min |
| **8** | C | `python src/calibration/08_pick_test.py` | End-to-end pick-and-place verification | ~10 min |

---

## Phase A — Arm Hardware (no camera needed)

### Step 0 — Dynamixel Motor Setup

**What it does:** Configures each motor's ID, baud rate, protocol version,
and operating mode using Dynamixel Wizard 2.0 and a U2D2 adapter. This must
be done *before* assembling the arm — each motor must be configured
individually.

**Tool:** [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) + U2D2 adapter

**Motor Configuration Table:**

| Motor | Dynamixel ID | Joint | Model | Baudrate |
|:---:|:---:|---|---|:---:|
| m1 | **1** | Base Pan | XM430 | 115 200 |
| m2 | **2** | Shoulder Tilt | XM540 | 115 200 |
| m3 | **3** | Elbow Tilt | XM430 | 115 200 |
| m4 | **4** | Wrist Tilt | XL430 | 115 200 |
| m5 | **5** | Claw / Gripper | XL430 | 115 200 |

**Procedure:**

1. Connect **one motor at a time** to the U2D2 via TTL cable.
2. Open Dynamixel Wizard 2.0 and **Scan** (factory default: ID 1, baud 57 600).
3. Set **ID** to the value in the table above.
4. Set **Baud Rate** to **115 200** (index 1).
5. Confirm **Protocol Version** = **2.0**.
6. Set **Operating Mode** to **Position Control (value 3)**.
7. Test by writing a **Goal Position** (range 0–4095, centre = 2048) — the motor should move.

**How to verify:** Each motor moves when you write a goal position in Wizard.

> ⚠️ Do this before assembly. Once assembled, all motors share the same bus
> and you would need to disconnect all but one to change IDs.

---

### Step 1 — System Check

**What it does:** Verifies connectivity to all 5 Dynamixel motors through
the OpenRB-150 firmware. Pings each motor at three baud rates (57 600,
115 200, 1 000 000) and reports which respond.

**Prerequisites:** OpenRB-150 firmware uploaded (see [firmware/openrb_bridge/openrb_bridge.ino](../firmware/openrb_bridge/openrb_bridge.ino)). Upload via Arduino IDE (Board: OpenRB-150, Port: `/dev/cu.usbmodem101`).

**Script:**

```bash
python src/diagnostics/diagnose_motors.py
```

**Expected output:**

```
  ✔  ID 1  Base Pan      (XM430)
       Model:    XM430-W350
       Position: 2048
       Baud:     115200  (matches firmware)

  ✔  ID 2  Shoulder Tilt (XM540)
  ...

  Summary:  5/5 motors detected
  ✔  All motors detected and baud rates match!
```

**How to verify:** All 5 motors show ✔ and the summary reads "5/5 motors detected".

**If it fails:**
- 0/5 found → check USB cable, 12 V power, firmware upload.
- Some missing → daisy-chain break between last found and first missing motor.
- Baud mismatch → reconfigure the motor in Dynamixel Wizard (Step 0).

---

### Step 2 — Joint Calibration

**What it does:** Drives each motor one at a time so you can verify that
the motor directions (signs) and zero positions match the IK solver's
conventions. If a motor moves the wrong way, you update the sign/offset in
[`solver.py`](../src/ik/solver.py).

**Script:**

```bash
python src/calibration/02_joints.py
```

**Expected output:** Each motor moves through a small range of motion. The
script prompts you to confirm the direction is correct for each joint.

**How to verify:** When prompted "Does motor m2 move UPWARD for positive
values?", visually confirm the motion matches. All motors should pass the
direction check.

---

### Step 2b — Claw Open/Close Calibration

**What it does:** Finds the correct Dynamixel step values for the claw
(motor 5) that fully open the gripper and firmly grip a 50 mm ball without
straining the motor. Every claw assembly is slightly different.

**Script:**

```bash
python src/calibration/02b_claw.py
```

**Procedure:**

1. The arm moves to a visible position.
2. Interactively adjust m5 (type values, `+`/`-` for ±50, `++`/`--` for ±200).
3. The script tests jaw symmetry and runs 3 open/close cycles.
4. Results are saved to `claw_calibration.json`.

**Expected output:** The script prints recommended values for
[`CLAW_OPEN_POS`](../src/config/arm.py) and [`CLAW_CLOSED_POS`](../src/config/arm.py),
and saves them to `claw_calibration.json`.

**How to verify:** The claw opens fully without motor strain, and grips a
50 mm ball firmly without crushing it.

**Configuration:** Update these constants in [`src/config/arm.py`](../src/config/arm.py) if needed:

```python
CLAW_OPEN_POS   = 2048   # fully open without motor strain
CLAW_CLOSED_POS = 1600   # grips a 50 mm ball without crushing
```

---

#### Step 02c — Tune SCAN_POSE (wrist-mounted camera only)

The wrist-mounted camera moves with the arm, so vision only works
from a known pose. Use the interactive tuning script to find and save
the right joint positions.

Run the interactive tuner:

```
python3 -m src.calibration.02c_scan_pose
```

The script:
1. Connects to the OpenRB-150 and moves the arm to the current SCAN_POSE
2. Opens the OAK-D S2 live camera feed
3. Shows current motor values overlaid on the frame

**Keyboard controls:**
- `W` / `S` → m2 shoulder raise / lower
- `E` / `D` → m3 elbow fold / unfold
- `R` / `F` → m4 wrist tilt up / down
- `T` / `G` → m1 base rotate
- `Y` / `H` → m5 claw open / close
- `[` / `]` → step size ×2 / ÷2
- `ENTER`   → **save** to `src/config/arm.py` and exit
- `Q`       → quit without saving

**Goal:** adjust until the camera looks straight down and the entire
workspace is visible in the frame with the claw out of view.

**Time:** ~5–10 min. Re-run if you change the camera mount.

---

### Step 3 — Sag (Droop) Compensation Calibration

**What it does:** Measures the arm's gravitational droop at different reach
distances and fits a correction model. Further reaches produce more droop;
this step quantifies the relationship so the IK solver can compensate.

**Script:**

```bash
python src/calibration/03_sag.py          # default test height = 5 cm
python src/calibration/03_sag.py 8        # custom test height = 8 cm
```

**Procedure:**

1. The arm moves to 5 different reach distances (12–36 cm) with sag
   compensation **OFF**.
2. At each position, you measure the actual claw height with a ruler.
3. The script computes the error (commanded vs. actual) and fits linear and
   quadratic regression models.
4. Results are saved to `sag_calibration.json` — automatically loaded by
   [`ArmIK.__init__()`](../src/ik/solver.py:112) on next startup.

**Expected output:**

```
[SAG] Linear  model: z_offset_multiplier = 0.042, R² = 0.95
[SAG] Quadratic model: a = 0.00018, b = 0.031, R² = 0.98
[SAG] Recommended model: quadratic
Saved to sag_calibration.json
```

**How to verify:** After restarting the IK solver, command the arm to Z = 5 cm
at multiple reaches. The claw tip should be within ± 3 mm of the target
height.

> **Tip:** If the claw touches the desk at far reaches, use a higher test
> height (e.g., `python src/calibration/03_sag.py 8`).

---

## Phase B — Vision Setup (camera needed, arm optional)

### Step 4 — HSV Colour Tuning

**What it does:** Opens a live camera feed with interactive HSV trackbars.
Adjust the sliders until only the target colour (red or blue) is visible in
the binary mask, then save the ranges.

**Script:**

```bash
python src/calibration/04_hsv_tuner.py
```

**Procedure:**

1. Place red and blue balls in the workspace under your actual lighting.
2. Adjust the trackbars for each colour until the mask cleanly isolates the ball.
3. Press `s` to save the ranges.
4. Update the ranges in [`SimpleBallDetector`](../src/vision/detector.py:322)
   if the defaults don't match.

**Expected output:** A multi-window display showing the raw camera feed,
HSV masks for red and blue, and the combined overlay.

**How to verify:** The binary mask shows a clean, solid blob for each ball
with no background noise. The contour should cover >80 % of the ball surface.

---

### Step 5 — HSV Refinement (Optional)

**What it does:** Statistically analyses a set of training images to suggest
optimal HSV bounds. More robust than manual tuning when you have
representative training data.

**Script:**

```bash
python src/calibration/05_hsv_refine.py
```

**Prerequisites:** Training images captured via
[`capture_data.py`](../src/training/capture_data.py).

**Expected output:** Suggested HSV ranges with statistical confidence bounds
(mean ± 2σ per channel).

**How to verify:** The suggested ranges should be tighter than or equal to
the manually tuned ranges from Step 4. Apply them and re-run
[`diagnose_detection.py`](../src/diagnostics/diagnose_detection.py) to
confirm detection still works.

---

### Step 6 — Homography Calibration

**What it does:** Maps camera pixel coordinates to physical centimetres on
the workspace plane using a 4-point perspective transform. This is the
critical bridge between vision and arm positioning.

**Script:**

```bash
python src/calibration/06_homography.py
```

**Procedure:**

1. The script shows the live camera feed.
2. Click the 4 corners of your workspace (top-left, top-right, bottom-right,
   bottom-left as seen in the camera).
3. Enter the physical (x, y) measurements of each corner in cm **from the
   shoulder joint** (motor 2 pivot):
   - x = forward (away from arm base)
   - y = left (+) / right (−)
4. The script computes `cv2.getPerspectiveTransform` and saves the result to
   `homography_calibration.json`.
5. Optionally verify by detecting a ball and showing its cm coordinates.

**Expected output:**

```
[HOMOGRAPHY] Saved calibration to homography_calibration.json
  Pixel corners: [[9, 17], [619, 16], [618, 381], [23, 378]]
  CM corners:    [[28.0, 22.0], [28.0, -22.0], [10.0, -22.0], [10.0, 22.0]]
```

**How to verify:** Place a ball at a known position (e.g., 20 cm forward,
5 cm left). The displayed cm coordinates should match within ± 1 cm.

**Configuration:** The homography is loaded automatically by
[`VisionBridge`](../src/ik/vision_bridge.py:108) from
`homography_calibration.json`. The hardcoded fallback defaults in
[`vision_bridge.py`](../src/ik/vision_bridge.py:90) are only used when no
calibration file exists.

> ⚠️ This calibration is pose-dependent. If you change `SCAN_POSE` at any point, you must re-run Step 06.

---

### Step 6b — Camera Height and Scan Region Verification

**What it does:** Verifies that the camera height matches the configuration
and that the camera can see balls across the entire arm workspace. Calculates
worst-case parallax error.

**Script:**

```bash
python src/calibration/06b_workspace.py
```

**Procedure:**

1. Measure the camera lens height above the table surface with a ruler.
2. Compare with [`CAMERA_HEIGHT`](../src/config/arm.py:53) (default: 43.0 cm).
3. The script tests 5 positions (centre + 4 workspace corners) for visibility
   and IK reachability.
4. Calculates worst-case parallax error for 50 mm balls.

**Expected output:**

```
  Camera height: 43.0 cm (configured) vs. 42.5 cm (measured) — OK
  Parallax at edges: ±1.2 mm (acceptable)
  5/5 positions visible and reachable
  PASS
```

**How to verify:** All 5 test positions pass both the visibility and
reachability checks. If parallax error exceeds 3 mm, adjust `CAMERA_HEIGHT`
in [`src/config/arm.py`](../src/config/arm.py:53).

> **Tip:** A 5 cm error in camera height causes approximately 3 mm parallax
> at workspace edges.

---

## Phase C — Integration Tuning (arm + camera together)

### Step 7 — Vision Offset Fine-Tune (Optional)

**What it does:** Detects and corrects any residual systematic offset
between the camera's coordinate output and the arm's actual reach. Since the
homography maps directly to the shoulder frame, both
[`CAMERA_OFFSET_X`](../src/config/arm.py:51) and
[`CAMERA_OFFSET_Y`](../src/config/arm.py:52) should be `0.0`. Only run this
step if Step 8 reveals a consistent directional error.

**Script:**

```bash
python src/calibration/07_vision_offset.py
```

**Procedure:**

1. Place a ball at 2–3 known physical positions.
2. Compare the detected coordinates (printed on screen) with the actual ruler
   measurements.
3. If consistently off by more than 3 mm in one direction, update the offset
   values in [`src/config/arm.py`](../src/config/arm.py).

**Expected output:** Detected positions within ± 3 mm of actual positions.

**How to verify:** After applying any offset correction, re-run the test.
The error should be < 3 mm.

---

### Step 8 — End-to-End Pick Test

**What it does:** Runs the full detect → approach → grab → lift → bin
pipeline at 5 positions across the workspace. This is the final validation
that the entire system is calibrated correctly.

**Script:**

```bash
python src/calibration/08_pick_test.py
```

**Test Positions:**

| # | Name | X (cm) | Y (cm) |
|---|------|--------|--------|
| 1 | Centre | 20 | 0 |
| 2 | Near | 14 | 0 |
| 3 | Far | 32 | 0 |
| 4 | Left | 20 | −12 |
| 5 | Right | 20 | 12 |

**Procedure:**

1. The script connects to the arm and camera.
2. For each position: you place a ball, the system detects → approaches →
   lowers → grabs → lifts → places in the correct bin.
3. After each pick, rate the result: **pass** / **partial** / **fail**.
4. The script prints a scored summary with diagnostic hints.
5. Results are saved to `pick_test_results.json`.

**Expected output:**

```
  ✅ Centre: pass (detection offset: 0.5 cm)
  ✅ Near:   pass (detection offset: 0.3 cm)
  ⚠️  Far:   partial — claw too high
  ✅ Left:   pass (detection offset: 0.8 cm)
  ✅ Right:  pass (detection offset: 0.4 cm)

  Score: 4/5 (80%)
  ✅ PASS — Calibration is good!
```

**Pass criteria:** ≥ 80 % clean picks across 5 positions.

**How to verify:** The summary shows ≥ 4/5 passes. If not, use the
diagnostic table below.

### Step 8 Diagnostic Table

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Consistent X/Y offset | Residual camera offset | Adjust `CAMERA_OFFSET_X/Y` in [`config/arm.py`](../src/config/arm.py) or redo Step 7 |
| Off in different directions each time | Bad homography | Redo Step 6 |
| Claw too high or too low | Sag calibration off | Redo Step 3, or nudge `z_offset_multiplier` ± 0.01 |
| Ball squirts out when grabbed | Claw positions wrong | Redo Step 2b |
| Arm can't reach detected balls | Scan region mismatch | Check Step 6b |
| Wrong bin | Colour detection error | Redo Steps 4–5 |

---

## Recalibration Guide

You only need to redo specific steps when something changes:

| What Changed | Steps to Redo |
|-------------|---------------|
| Changed camera mount or SCAN_POSE | 2c, 6, 6b, 7 |
| Moved the camera | 6, 6b, 7 |
| Changed lighting (new room, new lamp) | 4, 5 |
| Rebuilt or tightened the arm | 2, 2b, 3 |
| New claw or gripper attachment | 2b |
| Replaced a motor | 0 (for that motor), 1, 2 |
| Everything (full recalibration) | 0–8 |

### Quick Recalibration Checklist

If the system was working and accuracy has degraded:

1. Run `python src/diagnostics/diagnose_motors.py` — verify 5/5 motors OK.
2. Run `python src/diagnostics/check_motor_errors.py` — check for latched errors.
3. Run `python src/diagnostics/diagnose_detection.py` — verify HSV masks still isolate balls.
4. Run `python src/calibration/08_pick_test.py` — get a fresh accuracy score.
5. Use the diagnostic table above to identify which step(s) to redo.

---

## Calibration File Reference

| File | Created By | Loaded By |
|------|-----------|-----------|
| `sag_calibration.json` | Step 3 — [`03_sag.py`](../src/calibration/03_sag.py) | [`ArmIK.__init__()`](../src/ik/solver.py:115) |
| `homography_calibration.json` | Step 6 — [`06_homography.py`](../src/calibration/06_homography.py) | [`VisionBridge`](../src/ik/vision_bridge.py:108) |
| `claw_calibration.json` | Step 2b — [`02b_claw.py`](../src/calibration/02b_claw.py) | Manual — update [`config/arm.py`](../src/config/arm.py) |
| `pick_test_results.json` | Step 8 — [`08_pick_test.py`](../src/calibration/08_pick_test.py) | Reference only |

---

## Troubleshooting Common Calibration Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| Motor not found in Step 1 | `0/5 motors detected` | Check USB cable, 12 V power, firmware upload |
| Motor direction wrong in Step 2 | Joint moves opposite to expected | Negate the sign in [`solver.py`](../src/ik/solver.py) for that motor |
| Sag model poor fit (R² < 0.90) | Large Z errors after calibration | Re-measure more carefully; ensure ruler is perpendicular to table |
| HSV mask noisy in Step 4 | Background objects show in mask | Tighten S and V minimums; remove coloured objects from workspace |
| Homography off by > 2 cm | Detected cm don't match ruler | Re-measure corner positions from shoulder joint; verify arm is at SCAN_POSE |
| Camera not found in Step 4/6 | `OAKCamera: Could not open camera` | Re-plug USB, check `depthai` install, see [troubleshooting](troubleshooting.md) |
| Pick test < 80 % | Multiple partial/fail results | Identify the dominant failure mode from the diagnostic table above |
