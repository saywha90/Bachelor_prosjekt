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
| **2b** | A | Manual setup + `python src/calibration/02b_claw_grip_test.py` | Configure claw open/closed positions and validate adaptive grip | ~5–10 min |
| **2c** | A | `python src/calibration/02c_scan_pose.py` | Tune SCAN_POSE for wrist-mounted camera | ~5–10 min |
| **3** | A | `python src/calibration/03_sag.py` | Sag (droop) compensation calibration | ~10–20 min |
| **4** | B | `python src/calibration/04_hsv_tuner.py` | Interactive HSV colour tuning | ~5–15 min |
| **5** | B | `python src/calibration/05_hsv_refine.py` | Statistical HSV refinement | ~5 min |
| **6** | B | `python src/calibration/09_touch_calibration.py` | Pixel-to-cm touch-based homography calibration | ~5–10 min |

| **7** | C | `python src/calibration/07_vision_offset.py` | Fine-tune residual camera-to-shoulder offset | ~5 min |
| **8** | C | `python src/calibration/08_pick_test.py` | End-to-end pick-and-place verification | ~10 min |
| **10** | C | `PYTHONPATH=src python3 src/calibration/10_bin_calibration.py` | Hardware-first rear-bin route calibration for two-bin fold-over placement | Requires real hardware |

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
| m5 | **5** | Claw / Gripper | XM430 | 115 200 |

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

### Step 2b — Manual Claw Open/Close Setup + Adaptive Grip Validation

**What it does:** Establishes the real motor-5 limits used by the 3D printed
claw, then validates adaptive claw/grip behavior with
[`src/calibration/02b_claw_grip_test.py`](../src/calibration/02b_claw_grip_test.py).
The usable open/closed values depend on the installed claw geometry and are
written directly into the runtime configuration before running the validation.

**Tools:** Dynamixel Wizard 2.0 for direct M5 positioning, then edit
[`src/config/arm.py`](../src/config/arm.py) and run
[`src/calibration/02b_claw_grip_test.py`](../src/calibration/02b_claw_grip_test.py).

**Procedure:**

1. Center **M5** externally in Dynamixel Wizard before installing the claw.
2. Mount the 3D printed claw while it is **open**.
3. Slowly close the claw in small Wizard increments until it reaches the desired
   safe grip position for the real ball without forcing the printed parts.
4. Write the final open and closed step values directly into
   [`CLAW_OPEN_POS`](../src/config/arm.py:229) and
   [`CLAW_CLOSED_POS`](../src/config/arm.py:230).
5. Run the adaptive grip diagnostic to validate real behavior:

```bash
python src/calibration/02b_claw_grip_test.py
```

**How to verify:** The claw opens fully without motor strain, closes slowly to
the desired grip position, and [`02b_claw_grip_test.py`](../src/calibration/02b_claw_grip_test.py)
confirms the adaptive grip/claw behavior on the real arm.

**Configuration:** Update these constants in [`src/config/arm.py`](../src/config/arm.py) if needed:

```python
CLAW_OPEN_POS   = 2745   # open/neutral position for gripper (XM430-W210 raw goal position)
CLAW_CLOSED_POS = 3350   # safe closed/grip limit for adaptive close
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

**Required hold validation after tuning:**

1. Save the pose, then leave the arm at `SCAN_POSE` for several minutes under
   normal camera/payload conditions.
2. Confirm the elbow does **not** visibly sag and the camera view does not
   drift while holding still.
3. Treat [`M3_SCAN_CURRENT_LIMIT`](../src/config/arm.py:164) as an
   experimental starting value only. If the arm sags, the frame shifts, or
   runtime logs show M3 repeatedly near/at the hold limit, raise the limit in
   small steps and re-test.
4. Do not enable torque-relax unless you have separately validated a safe rest
   pose and a recovery sequence. It is disabled by default.

> ⚠️ A `SCAN_POSE` that looks correct for one frame is not enough. If the arm
> cannot hold that pose repeatably, any later vision calibration done from it
> can be invalid.

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

### Step 6 — Homography Calibration (Touch-Based)

**What it does:** Maps camera pixel coordinates to physical centimetres on
the workspace plane using a 4-point perspective transform. The arm physically
touches each corner to capture IK coordinates as ground truth — no ruler
measurements needed. Additionally, the step records the **grab height (Z)**
and **wrist tilt offset (m4)** at each calibration point, enabling
distance-dependent compensation at runtime.

> **Note:** This step was previously done with `06_homography.py` which required
> manual ruler measurements. The new `09_touch_calibration.py` is more accurate
> because it uses the arm's own IK coordinates. See [ADR 004](decisions/004-touch-calibration-replaces-homography.md).

**Script:**

```bash
python src/calibration/09_touch_calibration.py
```

**Procedure:**

1. The arm moves to `SCAN_POSE` and the script shows the live camera feed.
2. Click the 4 corners of your workspace (top-left, top-right, bottom-right,
   bottom-left as seen in the camera).
3. For each corner, use the following controls to drive the arm's claw so it
   physically touches the corner:
   - **W / S** — move forward / backward (X axis)
   - **A / D** — move left / right (Y axis)
   - **I / K** — adjust wrist tilt (m4 offset) up / down
4. At each point the script records the IK (x, y) coordinates, the Z height
   the claw needed to reach the surface, and the m4 wrist offset used.
5. The script computes `cv2.getPerspectiveTransform` and saves the result —
   along with the height and wrist arrays — to
   `homography_calibration.json`.

#### Limp-mode to WASD refinement safety

When the calibration is started from limp-mode coarse captures,
[`09_touch_calibration.py`](../src/calibration/09_touch_calibration.py)
does not blindly trust that the same X/Y can be replayed at a higher clearance
or fine-tune start height. Before entering the interactive WASD refinement loop,
the script:

1. Computes a conservative reach-aware start height with
   [`_limp_fine_tune_start_height()`](../src/calibration/09_touch_calibration.py:103).
2. Solves the clearance approach and first descent using strict, sag-disabled IK.
3. Runs FK on the resulting motor commands and compares the solved claw X/Y with
   the recorded limp X/Y.
4. Aborts before WASD refinement if the FK X/Y differs by more than
   [`LIMP_FINE_TUNE_XY_TOLERANCE_CM`](../src/calibration/09_touch_calibration.py:100),
   currently **1.0 cm**.

This protects against the previous failure mode where an unreachable clearance
or start pose was silently projected to a different location, so WASD refinement
began more than 1 cm away from the physical limp position.

The underlying IK behavior also changed for this path: [`ArmIK.solve()`](../src/ik/solver.py:329)
with `strict=True` now fails closed on joint-limit violations instead of
silently clamping to [`JOINT_LIMITS`](../src/ik/solver.py:97). Non-strict runtime
moves can still clamp and warn, but strict calibration/refinement moves must be
geometrically reachable and inside the configured joint limits because clamping
would make FK no longer match the requested X/Y.

If a target is unreachable at clearance or at the fine-tune start height, the
operator sees a message like:

```text
❌  Limp-to-WASD safety stop: limp fine-tune approach target X=..., Y=..., Z=... cm ...
    Target is unreachable at the current clearance/start height; refinement would start at the wrong XY.
    Aborting before WASD refinement.
```

When this happens, do **not** continue by increasing the tolerance. Move the
calibration ball/corner inward to a comfortably reachable workspace area, repeat
the limp capture for that point, and re-run Step 6. If the point should be
reachable, verify Step 2 joint calibration and the configured
[`JOINT_LIMITS`](../src/ik/solver.py:97) before retrying.

**Before running this step:** confirm Step 02c passed a real hold test.
If `SCAN_POSE` drifts while parked, the camera calibration will be tied to a
pose the runtime cannot reliably reproduce.

**Keyboard controls summary:**

| Key | Action |
|-----|--------|
| W / S | Arm forward / backward |
| A / D | Arm left / right |
| I / K | Wrist tilt up / down (m4 offset) |
| Enter | Confirm point |
| Q | Quit without saving |

**Expected output:**

```
[TOUCH-CAL] Saved calibration to homography_calibration.json
  Pixel corners: [[9, 17], [619, 16], [618, 381], [23, 378]]
  CM corners:    [[28.0, 22.0], [28.0, -22.0], [10.0, -22.0], [10.0, 22.0]]
  Heights:       [2.1, 2.3, 1.8, 1.9]
  Wrist offsets: [0.05, 0.02, -0.03, 0.01]
```

**How to verify:** Place a ball at a known position (e.g., 20 cm forward,
5 cm left). The displayed cm coordinates should match within ± 1 cm.

**Configuration:** The homography is loaded automatically by
[`VisionBridge`](../src/ik/vision_bridge.py:108) from
`homography_calibration.json`. The hardcoded fallback defaults in
[`vision_bridge.py`](../src/ik/vision_bridge.py:90) are only used when no
calibration file exists.

#### Height and wrist calibration data

The JSON file contains two additional arrays alongside the pixel/cm corners:

| JSON Field | Type | Description |
|------------|------|-------------|
| `height_calibration` | `[{x, y, z}, …]` | Per-point grab height (Z in cm) recorded when the claw touched the surface |
| `wrist_calibration` | `[{x, y, m4_offset}, …]` | Per-point m4 servo offset needed for the wrist to be level at that reach |

At runtime, [`compute_grab_height(x, y)`](../src/config/arm.py) and
[`compute_wrist_correction(x, y)`](../src/config/arm.py) perform **linear
interpolation** over these arrays to determine the correct Z and m4 values
for any given ball position. When calibration data is missing, both
functions fall back to formula-based defaults.

#### Relationship with sag calibration (Step 3)

Touch calibration inherently captures the arm's real droop at each point
because the operator drives the claw to the physical surface. This means
the recorded Z values already include any sag that would otherwise need
correction. To avoid double-compensating, the IK solver's
[`solve()`](../src/ik/solver.py) accepts a `skip_sag` parameter — when
touch calibration data exists, grab moves are solved with `skip_sag=True`
so the sag model from Step 3 is bypassed. Sag calibration remains active
for all other move types (e.g., scanning, binning) where touch data does
not apply.

> ⚠️ This calibration is pose-dependent. If you change `SCAN_POSE`, see a
> warning from [`VisionBridge.verify_pose()`](../src/ik/vision_bridge.py:182),
> or observe scan-pose droop during runtime, you must re-establish a stable
> `SCAN_POSE` and re-run Step 06 before trusting ball coordinates.

---

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
   lowers → grabs → lifts → places in the correct rear bin.
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
| Claw too high or too low | Sag calibration off, or touch cal heights stale | Redo Step 6 (touch calibration records grab height per point); if no touch cal, redo Step 3 |
| Ball squirts out when grabbed | Claw positions wrong or adaptive grip needs validation | Recheck Step 2b values in [`CLAW_OPEN_POS`](../src/config/arm.py:229) / [`CLAW_CLOSED_POS`](../src/config/arm.py:230), then run [`02b_claw_grip_test.py`](../src/calibration/02b_claw_grip_test.py) |
| Arm can't reach detected balls | Scan region mismatch | Re-run touch calibration (Step 6) with corners within arm reach |
| Wrong bin | Colour detection error | Redo Steps 4–5 |

---

### Step 10 — Rear-Bin Route Calibration

**What it does:** Fine-tunes the production rear-placement route stored in
[`bin_calibration.json`](../src/calibration/bin_calibration.json). This is a
hardware-first tool that mirrors touch calibration: hardware mode is the
default, dry-run is optional, movements require explicit confirmations, and
saving requires an explicit `SAVE` confirmation with a timestamped backup.

Rear placement is **fold-over**: the arm reaches behind the robot by folding
over the top, not by rotating the base 180°. For rear route waypoints the base
yaw is guarded by [`DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG`](../src/config/arm.py:227),
defaulting to ±45°, and the JSON field `rear_base_yaw_limit_deg` may override
that limit.

**Commands:**

```bash
PYTHONPATH=src python3 src/calibration/10_bin_calibration.py
PYTHONPATH=src python3 src/calibration/10_bin_calibration.py --dry-run
PYTHONPATH=src python3 src/calibration/10_bin_calibration.py --validate-only
```

**Production route schema:**

- Required shared waypoints: `shared_waypoints.front_neutral`,
  `shared_waypoints.rear_transfer`, and `shared_waypoints.rear_return_lift`.
- Required per-bin poses: `bins.RED_BIN.drop` and `bins.BLUE_BIN.drop`.
- The return route after opening the claw is `rear_return_lift` →
  `front_neutral`, so the arm lifts clear of the rear sorting bin before facing
  forward again.
- Editable pose fields are `x`, `y`, `z`, `m4_offset`, and `skip_sag`.
- The real setup has only two destination bins: `RED_BIN` and `BLUE_BIN`.
  `REJECT_BIN` is not written by the tool and is not used for real sorting.
- No-grip / air-pick cases should open the claw, retreat through the
  prevalidated pickup clearance, then return to scan/look-again rather than
  route to a reject bin.

**Interactive controls:**

| Command | Action |
|---------|--------|
| `1`–`5` | Choose `front_neutral`, `rear_transfer`, `rear_return_lift`, or one per-bin drop pose |
| `x+` / `x-`, `y+` / `y-`, `z+` / `z-` | Adjust the selected Cartesian field by the current centimetre step |
| `m4+` / `m4-` | Adjust the selected wrist trim by the current motor-step size |
| `step <cm>` / `m4step <n>` | Change adjustment step sizes |
| `yaw <deg>` | Set `rear_base_yaw_limit_deg` |
| `v` / `va` | Validate selected waypoint / validate all waypoints with strict IK |
| `move` | Move to the selected validated waypoint after typing `MOVE` |
| `test red`, `test blue`, `test selected`, `test all` | Route-test validated waypoints after typing `TEST RED`, `TEST BLUE`, or `TEST ALL` |
| `limp` | Hardware only: disable torque on motors 1–4 for hand-guiding |
| `lock` | Hardware only: re-enable torque at current goals |
| `capture` | Hardware only: read current motor positions, FK-convert, validate, store in memory, then lock |
| `pos` | Hardware only: read current motor positions |
| `save` | Validate all, require `SAVE`, create backup, then overwrite [`bin_calibration.json`](../src/calibration/bin_calibration.json) |
| `q` | Quit; unsaved edits require `DISCARD` |

**Operational notes:**

- Use `--dry-run` for offline editing/validation only; it cannot move hardware,
  limp, lock, capture, or route-test.
- Use `--validate-only` before production startup or after manual JSON edits.
- Strict IK validation proves the route is kinematically valid, but real route
  `x`/`z` values can still need slow physical fine-tuning to clear bin walls,
  avoid scraping, and place balls reliably.

---

## Rear Route Simulation Demo

Use the simulation demo to strictly prevalidate and visualize rear placement
without moving hardware. It loads the same route schema used by production.

**Red + blue sequence:**

```bash
PYTHONPATH=src python3 src/simulation/route_demo.py --calibration src/calibration/bin_calibration.json --sequence
PYTHONPATH=src python3 src/simulation/route_demo.py --calibration src/simulation/sample_route_calibration.json --sequence
```

**Single-bin testing:**

```bash
PYTHONPATH=src python3 src/simulation/route_demo.py --calibration src/calibration/bin_calibration.json --destination RED_BIN
PYTHONPATH=src python3 src/simulation/route_demo.py --calibration src/calibration/bin_calibration.json --destination BLUE_BIN
```

Add `--no-gui` to load and prevalidate without opening the visualizer.

**Air-pick / no-grip path:**

```bash
PYTHONPATH=src python3 src/simulation/route_demo.py --calibration src/calibration/bin_calibration.json --air-pick
```

The air-pick demo intentionally contains no bin waypoints: it closes on air,
opens while retreating, returns to `SCAN_POSE`, and scans/looks again.

---

## Recalibration Guide

You only need to redo specific steps when something changes:

| What Changed | Steps to Redo |
|-------------|---------------|
| Changed camera mount or SCAN_POSE | 2c, 6, 7 |
| Moved the camera | 6, 7 |
| Changed lighting (new room, new lamp) | 4, 5 |
| Rebuilt or tightened the arm | 2, manual Step 2b if the claw moved, 3 |
| New claw or gripper attachment | Manual Step 2b, then [`02b_claw_grip_test.py`](../src/calibration/02b_claw_grip_test.py) |
| Replaced a motor | 0 (for that motor), 1, 2 |
| Moved rear bins or changed rear route clearance | 10 |
| Everything (full recalibration) | 0–10 |

### Quick Recalibration Checklist

If the system was working and accuracy has degraded:

1. Run `python src/diagnostics/diagnose_motors.py` — verify 5/5 motors OK.
2. Run `python src/diagnostics/check_motor_errors.py` — check for latched errors.
3. Run `python src/diagnostics/diagnose_detection.py` — verify HSV masks still isolate balls.
4. Run `python src/calibration/08_pick_test.py` — get a fresh accuracy score.
5. Run `PYTHONPATH=src python3 src/calibration/10_bin_calibration.py --validate-only` — verify the rear route schema still passes strict IK.
6. Use the diagnostic table above to identify which step(s) to redo.

---

## Calibration File Reference

| File | Created/Updated By | Loaded By |
|------|--------------------|-----------|
| `sag_calibration.json` | Step 3 — [`03_sag.py`](../src/calibration/03_sag.py) | [`ArmIK.__init__()`](../src/ik/solver.py:115) |
| `homography_calibration.json` | Step 6 — [`09_touch_calibration.py`](../src/calibration/09_touch_calibration.py) | [`VisionBridge`](../src/ik/vision_bridge.py:108), [`compute_grab_height()`](../src/config/arm.py), [`compute_wrist_correction()`](../src/config/arm.py) |
| `bin_calibration.json` | Step 10 — [`10_bin_calibration.py`](../src/calibration/10_bin_calibration.py) | [`load_transport_route_calibration()`](../src/config/arm.py:517), [`get_transport_route()`](../src/config/arm.py:558), [`prevalidate_transport_plan()`](../src/main.py:290), [`route_demo.py`](../src/simulation/route_demo.py) |
| [`src/config/arm.py`](../src/config/arm.py) | Manual Step 2b — write [`CLAW_OPEN_POS`](../src/config/arm.py:229) and [`CLAW_CLOSED_POS`](../src/config/arm.py:230) after Wizard setup | Runtime claw commands and [`02b_claw_grip_test.py`](../src/calibration/02b_claw_grip_test.py) |
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
| Rear route validates but clips bin or misses drop | Strict IK is valid, but physical X/Z route clearance needs tuning | Re-run Step 10 on hardware and adjust slowly with `step 0.1`, route tests, and explicit `SAVE` |
