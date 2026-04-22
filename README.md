# Autonomia — Autonomous Sorting Robot Arm

**Bachelor Project 2026 — 4-DOF robotic arm with vision-guided pick-and-place**

A complete autonomous sorting system that uses an OAK-D camera to detect coloured balls, computes inverse kinematics, and commands a 4-DOF robotic arm via serial to sort objects into bins.

---

## System Overview

```
OAK-D Camera ──► VisionBridge ──► State Machine ──► IK Solver ──► OpenRB-150 ──► Dynamixel Motors
  (RGB stream)    (homography)      (main.py)      (pi_kinematics)   (serial)       (5 axes)
```

| Component | Hardware | Software |
|---|---|---|
| Camera | Luxonis OAK Series 2 (IMX378) | `oak_camera.py` + `enhanced_detector.py` |
| Controller | Raspberry Pi 5 | `main.py` (Python 3) |
| Motor Controller | OpenRB-150 | `openrb_bridge.ino` (Arduino) |
| Motors | Dynamixel (5×) — XM430, XM540, XL430 | Steps 0–4095, centre = 2048 |

---

## Quick Start

### 1. Install dependencies

```bash
pip3 install -r src/requirements.txt
```

### 2. Run with simulated data (no hardware needed)

```bash
cd src/IK
python3 main.py
```

This opens the 3-D visualiser and runs through 2 fake detections (1 red, 1 blue) with the full state machine. Both `USE_REAL_SERIAL` and `USE_REAL_CAMERA` are `False` by default.

### 3. Run live detection only (OAK camera, no arm)

```bash
cd src
python3 vision/test_enhanced_detector.py
```

Press `q` to quit, `s` for statistics, `r` to reset.

---

## Project Structure

```
src/
├── IK/
│   ├── main.py                  ⭐ Master control loop (state machine)
│   ├── pi_kinematics.py         📐 4-DOF geometric IK solver
│   ├── vision_bridge.py         🔌 Camera → arm coordinate adapter
│   ├── config.py                ⚙️  Arm coordinates, bin positions, timing
│   ├── openrb_bridge.ino        🎛️  Arduino firmware for OpenRB-150
│   └── Simu/
│       ├── mock_serial.py       🧪 Fake serial for testing without hardware
│       ├── visualizer.py        📊 Live 3-D matplotlib arm visualiser
│       └── test_ik_virtual.py   🧪 IK solver standalone tests
│
└── vision/
    ├── enhanced_detector.py     ⭐ SimpleBallDetector — main detection engine
    ├── oak_camera.py            📷 OAK-D camera wrapper (depthai v3)
    ├── config.py                ⚙️  Camera resolution, ball thresholds
    ├── calibrate_camera.py      🎯 Homography calibration tool (click 4 corners)
    ├── color_histogram_classifier.py  🤖 SVM inference wrapper
    ├── train_color_classifier.py      🏋️  Train the SVM colour model
    ├── capture_training_data.py       📸 Capture OAK images for training
    ├── recalibrate_hsv.py             🔬 Analyse images → suggest new HSV ranges
    ├── hsv_tuner.py                   🎨 Interactive live HSV trackbar tuner
    ├── diagnose_detection.py          🩺 Live mask + click-to-read HSV
    ├── stream_debug.py                🔎 Extended stream debugger
    ├── test_enhanced_detector.py      🧪 Live ball detection test
    ├── test_record_stats.py           📈 Run test + generate report charts
    └── models/
        └── ball_color_classifier.pkl  Trained SVM model (64 KB)
```

---

## Hardware Setup

### Arm Dimensions (configure in `pi_kinematics.py`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| L1 | 25.5 cm | Shoulder → Elbow |
| L2 | 23.0 cm | Elbow → Wrist pivot |
| L3 | 16.5 cm | Wrist pivot → Claw tip |
| Shoulder Height | 33.0 cm | Shoulder joint above workspace |

### Bin Positions (configure in `src/IK/config.py`)

All coordinates are in **centimetres relative to the shoulder joint origin** (x = forward, y = left/right, z = up).

| Position | X (cm) | Y (cm) | Z (cm) |
|----------|--------|--------|--------|
| HOME | 20.0 | 0.0 | 30.0 |
| RED_BIN | 20.0 | 8.0 | 10.0 |
| BLUE_BIN | 20.0 | -8.0 | 10.0 |
| REJECT_BIN | 25.0 | 0.0 | 12.0 |

---

## 🔧 Calibration Guide (First-Time Setup)

Calibration must be completed before running the full system. Follow these phases in order — each step depends on the ones before it. Total time: ~45–70 minutes.

### Phase A — Arm Hardware (no camera needed)

| Step | Script / Tool | What It Does | Time |
|:---:|---|---|---|
| **0** | Dynamixel Wizard 2.0 | Set motor IDs, baudrate, and operating mode | ~10 min |
| **1** | Upload [`openrb_bridge.ino`](src/IK/openrb_bridge.ino) | Flash firmware to OpenRB-150 via Arduino IDE | ~2 min |
| **2** | [`calibrate_joints.py`](src/IK/calibrate_joints.py) | Verify motor signs, zeros, and directions | ~10–15 min |
| **2b** | [`calibrate_claw.py`](src/IK/calibrate_claw.py) | Tune claw open/close positions (m5) | ~2 min |
| **3** | [`calibrate_sag.py`](src/IK/calibrate_sag.py) | Measure gravity droop at 5 reaches, fit compensation model | ~10–20 min |

#### Step 0 — Dynamixel Motor Setup (USB2Dynamixel / U2D2)

Before assembling the arm, configure each motor individually using [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and a **U2D2** (or USB2Dynamixel) adapter.

Connect one motor at a time and set:

| Motor | Dynamixel ID | Joint | Model | Baudrate |
|:---:|:---:|---|---|:---:|
| m1 | **1** | Base Pan | XM430 | 115200 |
| m2 | **2** | Shoulder Tilt | XM540 | 115200 |
| m3 | **3** | Elbow Tilt | XM430 | 115200 |
| m4 | **4** | Wrist Tilt | XL430 | 115200 |
| m5 | **5** | Claw / Gripper | XL430 | 115200 |

**In Dynamixel Wizard 2.0:**
1. **Scan** for the motor (it ships with ID=1, baud=57600 by default)
2. Set **ID** to the value in the table above
3. Set **Baud Rate** to **115200** (index 1)
4. Confirm **Protocol Version** is **2.0**
5. Set **Operating Mode** to **Position Control (value: 3)**
6. Test by writing a **Goal Position** (range: 0–4095, center=2048) — the motor should move

> ⚠️ **Do this before assembly!** It's much easier to configure motors when they're loose on the bench. Once assembled, they share the same bus and you'd need to disconnect all but one to change IDs.

#### Step 1 — Flash Firmware
```bash
# Open src/IK/openrb_bridge.ino in Arduino IDE
# Select board: OpenRB-150, port: /dev/cu.usbmodem101
# Upload
```

#### Step 2 — Joint Calibration
Drives each motor one at a time so you can verify directions and zero positions.
```bash
cd src/IK && python3 calibrate_joints.py
```
Follow the on-screen prompts. If any motor moves the wrong way, update the sign/offset in `pi_kinematics.py`.

#### Step 2b — Claw Open/Close Calibration

Every claw is built slightly differently. This script helps you find the correct m5 positions that fully open and firmly grip a 50 mm ball without straining the motor.

```bash
cd src/IK && python3 calibrate_claw.py
```

**What happens:**
1. Moves the arm to a visible position
2. Lets you interactively adjust m5 (type values, `+`/`-` for ±50, `++`/`--` for ±200)
3. Tests jaw symmetry and runs 3 open/close cycles
4. Saves results to `claw_calibration.json` and prints config update instructions

Update these constants in [`config.py`](src/IK/config.py) if the defaults don't match:
- `CLAW_OPEN_POS` — fully open without motor strain (default: `2048`)
- `CLAW_CLOSED_POS` — grips a 50 mm ball without crushing (default: `1600`)

#### Step 3 — Sag (Droop) Calibration
The arm droops under gravity — further reaches = more droop. This script measures the droop and computes the correction automatically.
```bash
cd src/IK && python3 calibrate_sag.py       # default test height = 5 cm
cd src/IK && python3 calibrate_sag.py 8     # custom test height = 8 cm
```
**What happens:**
1. Moves the claw to 5 different reach distances (12–36 cm) with sag compensation OFF
2. You measure the actual claw height at each position with a ruler
3. Fits linear and quadratic models to the error
4. Saves results to `sag_calibration.json` — **automatically loaded** by `ArmIK` on next startup

> **Tip:** If the claw touches the desk at far reaches, use a higher test height (e.g., `python3 calibrate_sag.py 8`).

### Phase B — Vision Setup (camera needed, arm optional)

| Step | Script | What It Does | Time |
|:---:|---|---|---|
| **4** | [`hsv_tuner.py`](src/vision/hsv_tuner.py) | Interactively tune HSV colour ranges with live preview | ~5–15 min |
| **5** | [`recalibrate_hsv.py`](src/vision/recalibrate_hsv.py) | Statistically refine HSV ranges from training images | ~5 min |
| **6** | [`calibrate_homography.py`](src/IK/calibrate_homography.py) | Click 4 workspace corners → compute pixel-to-cm transform | ~5–10 min |
| **6b** | [`verify_workspace.py`](src/IK/verify_workspace.py) | Verify camera height and scan region coverage | ~3 min |

#### Step 4 — HSV Colour Tuning
Place red and blue balls in the workspace under your actual lighting conditions.
```bash
cd src && python3 vision/hsv_tuner.py
```
Adjust the trackbars until only the target colour is visible in the mask. Press `s` to save. Update the ranges in `enhanced_detector.py`.

#### Step 5 — HSV Refinement (Optional)
If you have training images captured via `capture_training_data.py`:
```bash
cd src && python3 vision/recalibrate_hsv.py training_data
```
Uses statistical analysis to suggest optimal HSV bounds.

#### Step 6 — Homography Calibration
Maps camera pixels to physical centimetres on the workspace.
```bash
cd src/IK && python3 calibrate_homography.py
```
**What happens:**
1. Shows the camera feed — click the 4 corners of your workspace
2. Enter the physical measurements (cm) of each corner from the arm's shoulder joint
3. Computes the perspective transform and saves to `homography_calibration.json`
4. Optionally verifies by detecting a ball and showing its cm coordinates

#### Step 6b — Camera Height & Scan Region Verify

Verifies camera height (for parallax accuracy) and checks that the camera can see balls across the entire arm workspace.

```bash
cd src/IK && python3 verify_workspace.py
```

**What happens:**
1. Prompts you to measure camera height — compares with `CAMERA_HEIGHT` in [`config.py`](src/IK/config.py)
2. Calculates worst-case parallax error for 50 mm balls
3. Tests 5 positions (centre + 4 workspace corners) — uses camera auto-detection if available, falls back to manual confirmation
4. Checks IK reachability for all test positions
5. Prints a pass/fail summary

> **Tip:** A 5 cm error in camera height causes ~3 mm parallax at workspace edges.

### Phase C — Integration Tuning (arm + camera together)

| Step | Script | What It Does | Time |
|:---:|---|---|---|
| **7** | [`calibrate_vision_offset.py`](src/IK/calibrate_vision_offset.py) | Fine-tune residual camera-to-shoulder offset (if needed) | ~5 min |
| **8** | [`test_pick.py`](src/IK/test_pick.py) | End-to-end pick-and-place verification | ~10 min |

#### Step 7 — Vision Offset Fine-Tune (Optional)

Since the homography already maps directly to the shoulder frame, `CAMERA_OFFSET_X` and `CAMERA_OFFSET_Y` in [`config.py`](src/IK/config.py) should both be `0.0`. Only run this step if the end-to-end test (Step 8) shows a consistent offset in one direction.

```bash
cd src/IK && python3 calibrate_vision_offset.py
```
Place a ball at 2–3 known physical positions, compare the detected coordinates with actual ones. If consistently off by more than 3 mm, update the offset values.

#### Step 8 — End-to-End Pick Test

The final verification — runs the full detect → move → grab → bin pipeline at 5 positions across the workspace.

```bash
cd src/IK && python3 test_pick.py
```

**What happens:**
1. Connects to both the arm (serial) and camera (if available)
2. For each of 5 test positions: detects ball → approaches → lowers → grabs → lifts → places in correct bin
3. After each pick, asks if it worked (pass/partial/fail)
4. Prints a scored summary with diagnostic hints
5. Saves detailed results to `pick_test_results.json`

Works without a camera too — enter ball coordinates manually when prompted.

**Diagnostic table:**

| Symptom | Likely Cause | Fix |
|---|---|---|
| Consistent X/Y offset | Residual camera offset | Adjust `CAMERA_OFFSET_X/Y` in [`config.py`](src/IK/config.py) or redo Step 7 |
| Off in different directions | Bad homography | Redo Step 6 |
| Claw too high / too low | Sag calibration off | Redo Step 3, or nudge `z_offset_multiplier` ±0.01 |
| Ball squirts out when grabbed | Claw positions wrong | Redo Step 2b |
| Arm can't reach detected balls | Scan region mismatch | Check Step 6b |
| Wrong bin | Colour detection error | Redo Steps 4–5 |

> **Target:** ≥ 80% clean picks across 5 positions = calibration complete.

### ✅ Calibration Complete

After all steps (0 through 8), the system is ready for autonomous operation:
```bash
cd src/IK && python3 main.py
```

> **Re-calibration:** You only need to redo specific steps when something changes:
> - Moved the camera → redo Steps 6, 6b, 7
> - Changed lighting → redo Steps 4, 5
> - Rebuilt/tightened the arm → redo Steps 2, 2b, 3
> - New claw or gripper → redo Step 2b
> - Everything → redo all steps 0–8

---

## Running the Full System

### Toggle hardware in `src/IK/main.py`

```python
USE_REAL_SERIAL = False     # True → real OpenRB-150 on SERIAL_PORT
USE_REAL_CAMERA = False     # True → real OAK-D camera + homography
SERIAL_PORT     = "/dev/cu.usbmodem101"
```

### State Machine Flow

```
HOME → SCANNING → [queue built] → APPROACHING (80%) → [correction image]
     → APPROACHING (100%) → GRABBING → SORTING → DROPPING → HOME
     → [next in queue] → ... → [queue empty] → SCANNING → ...
```

After each scan round, if the camera is live it rescans the workspace for any remaining objects (up to 3 rounds). In simulation mode it processes the fake queue once and exits.

---

## Homography Calibration

> **See [Calibration Guide — Step 6](#step-6--homography-calibration) above for the full procedure.**

---

## Vision Detection System

### SimpleBallDetector — 10-step pipeline

```
1.  Frame (OAK-D BGR)
2.  Resize → 0.75× scale for performance
3.  Lighting analysis → LOW / MEDIUM / HIGH (300–700 lux)
4.  CLAHE compensation (LAB L-channel, only at LOW)
5.  BGR → HSV + Grayscale
6.  Gaussian blur (5×5, grayscale only)
7.  HSV multi-range detection (6 red ranges, 2 blue ranges)
8.  Hough Circle Transform (geometric validation, every N frames)
9.  Ensemble merge (Union-Find clustering + confidence boost)
10. SVM colour verification (corrects label if ≥ 75% confident)
11. Kalman filter tracking (stable IDs across frames)
12. NMS + per-colour limit → return max N balls per colour
```

### Detection Results (OAK Series 2)

| Metric | Result |
|---|---|
| Red ball detection | ~98–100% |
| Blue ball detection | ~97–100% |
| False positive rate | < 2% |
| FPS | ~20–25 |

### Ball dimensions

```python
SimpleBallDetector.BALL_DIAMETER_MM = 50.0   # 50 mm physical diameter
```

---

## HSV Recalibration

> **See [Calibration Guide — Steps 4 & 5](#step-4--hsv-colour-tuning) above for the full procedure.**

---

## IK Solver

**File:** `src/IK/pi_kinematics.py`

Geometric 4-DOF inverse kinematics with:
- End-effector offset compensation (L3 wrist extension)
- Sag/droop compensation (`z_offset_multiplier = 0.04`, tune empirically)
- Two-step visual servoing: `calculate_partial_move(target, percentage=0.80)`

```python
from pi_kinematics import ArmIK

arm = ArmIK()
steps = arm.solve(x=20.0, y=5.0, z=0.0)
# → {"m1": 2048, "m2": 1820, "m3": 2201, "m4": 2075}

json_cmd = arm.solve_to_json(20.0, 5.0, 0.0)
# → '{"m1": 2048, "m2": 1820, "m3": 2201, "m4": 2075}'
```

Motor convention: Dynamixel XM430, XM540, XL430 — 0–4095 steps, centre = 2048 = 180°.

---

## Arduino Firmware

**File:** `src/IK/openrb_bridge.ino`

Upload to **OpenRB-150** via Arduino IDE (select board: OpenRB-150).

The firmware:
- Reads newline-terminated JSON from serial: `{"m1":2048,"m2":1820,"m3":2201,"m4":2075}`
- Moves all 5 Dynamixel motors to target positions with velocity control
- Replies `OK\n` when movement is complete
- Replies `ERR\n` on malformed input

Serial settings: **115200 baud, 8N1**.

---

## Arduino Upload Steps

1. Install the **Dynamixel2Arduino** library via Arduino IDE Library Manager
2. Open `src/IK/openrb_bridge.ino`
3. Board: **OpenRB-150** (install via Boards Manager → `ROBOTIS`)
4. Port: `/dev/cu.usbmodem101` (macOS) or `/dev/ttyACM0` (Linux) or `COMx` (Windows)
5. Upload → open Serial Monitor @ 115200 to verify boot message

---

## Simulation & Testing (no hardware)

### 3-D Visualiser

```bash
cd src/IK && python3 main.py
```

The `MockSerial` animates the arm through each motor command frame-by-frame in a matplotlib 3-D window. This lets you validate the full IK + state machine before touching hardware.

### IK Standalone Test

```bash
cd src/IK && python3 Simu/test_ik_virtual.py
```

Runs the IK solver through a grid of targets and displays the arm reaching each one.

### Vision Standalone Test

```bash
cd src && python3 vision/test_enhanced_detector.py
```

Runs detection on the live OAK-D feed with annotated overlay. No arm connection required.

---

## 🐛 Debugging & Diagnostics

When something goes wrong, use these diagnostic tools to identify the issue before re-calibrating.

### Quick Reference — Which Tool to Use

| Symptom | Tool | Command |
|---|---|---|
| No motors respond / some missing | [`diagnose_motors.py`](src/IK/diagnose_motors.py) | `cd src/IK && python3 diagnose_motors.py` |
| Motor LED blinking red / motor stuck | [`check_errors.py`](src/IK/check_errors.py) | `cd src/IK && python3 check_errors.py` |
| Balls not detected / wrong colour | [`diagnose_detection.py`](src/vision/diagnose_detection.py) | `cd src && python3 vision/diagnose_detection.py` |
| Need per-contour rejection details | [`stream_debug.py`](src/vision/stream_debug.py) | `cd src && python3 vision/stream_debug.py` |

### Motor Diagnostics

#### [`diagnose_motors.py`](src/IK/diagnose_motors.py)

Pings all 5 Dynamixel motors (IDs 1–5) at three baud rates (57600, 115200, 1000000) and prints a colour-coded connectivity report.

```bash
cd src/IK && python3 diagnose_motors.py
```

**What it reports per motor:**
- ✔ Found / ✘ Not found
- Model name (e.g., XM430-W350)
- Current position (0–4095)
- Baud rate detected at
- ⚠ Warning if motor baud differs from firmware's expected 115200

**Use when:**
- Motors don't move at all after power-on
- Only some motors respond (daisy-chain wiring issue)
- After initial Dynamixel setup (Step 0) to verify connectivity
- After reassembling the arm

#### [`check_errors.py`](src/IK/check_errors.py)

Reads the hardware error status register from all motors and decodes the error bit flags.

```bash
cd src/IK && python3 check_errors.py
```

**Error flags decoded:**

| Bit | Error | Common Cause |
|:---:|---|---|
| 0 | Input Voltage Error | Power supply issue (check 12V) |
| 2 | Overheating Error | Motor running too long under load |
| 3 | Motor Encoder Error | Internal motor fault |
| 4 | Electrical Shock Error | Wiring short / bad connection |
| 5 | Overload Error | Mechanical jam, arm collision, or excessive load |

> ⚠️ **Hardware errors require a 12V power cycle to clear.** Software commands alone won't reset them.

**Use when:**
- Motor LED is blinking red
- Motor suddenly stopped working after a collision or jam
- Motor feels limp (torque lost) but comms still work

### Vision Diagnostics

#### [`diagnose_detection.py`](src/vision/diagnose_detection.py)

Live 4-window HSV diagnostic viewer showing red/blue masks in real-time.

```bash
cd src && python3 vision/diagnose_detection.py
```

**Windows shown:**
1. **Original** — raw camera feed (click any pixel to print its H/S/V values)
2. **Red Mask** — red HSV threshold result
3. **Blue Mask** — blue HSV threshold result
4. **Overlay** — both masks on the original image

**Console output (every 30 frames):**
- Mask coverage percentage per colour
- Valid contour count
- Contour rejection breakdown: how many failed at each gate (area, radius, circularity, aspect ratio, solidity)

**Controls:** `c` = print current HSV ranges, `q` = quit

**Use when:**
- Balls not detected or detection is intermittent
- Need to check if HSV ranges cover actual ball colours under current lighting
- Before/after HSV recalibration to verify improvement

#### [`stream_debug.py`](src/vision/stream_debug.py)

Advanced 4-panel debug viewer with per-contour rejection reasons and confidence scores.

```bash
cd src && python3 vision/stream_debug.py
```

**Panels shown:**
1. **Top-Left:** Camera + accepted detections (●) + rejected contours (×)
2. **Top-Right:** Red HSV mask with contour outlines
3. **Bottom-Left:** Blue HSV mask with contour outlines
4. **Bottom-Right:** Stats — detection count, per-ball confidence bars, FPS

**Per-contour validation gates (with thresholds):**

| Gate | Threshold | Weight in Confidence |
|---|---|---|
| Area | ≥ π × min_radius² | — |
| Radius | [min_radius, max_radius] | — |
| Circularity | ≥ 0.60 | 40% |
| Aspect Ratio | ≥ 0.70 | 20% |
| Solidity | ≥ 0.75 | 20% |
| Colour Saturation | scored (glare-aware) | 20% |

**Use when:**
- Need to see *exactly* why specific contours are being rejected
- Tuning detection thresholds with live visual feedback
- More detailed than `diagnose_detection.py` — shows per-contour rejection reasons

---

## Troubleshooting

### `zsh: command not found: python`
Use `python3` on macOS:
```bash
python3 main.py
```

### `ModuleNotFoundError: No module named 'cv2'`
```bash
pip3 install -r src/requirements.txt
```

### Camera won't open
```bash
# Check USB connection and depthai device detection
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

### Balls not detected
1. Run `python3 src/vision/diagnose_detection.py` and left-click on a ball to read its H/S/V values
2. Compare with the ranges in `enhanced_detector.py` — if they don't overlap, recalibrate
3. Try `python3 src/vision/hsv_tuner.py` to tune ranges live with trackbars

### Arm goes to wrong position
1. Verify `WORKSPACE_PX` in `vision_bridge.py` matches your actual camera frame (run `calibrate_camera.py`)
2. Verify `WORKSPACE_CM` matches your physical workspace measurements
3. Check `shoulder_height` and `z_offset_multiplier` in `pi_kinematics.py` — tune these on the physical arm

### Serial: no `OK` response
- Confirm baud rate is 115200 on both sides
- Check `SERIAL_PORT` in `main.py` matches your device (use `ls /dev/tty*` on Mac/Linux)
- Confirm OpenRB-150 firmware is uploaded and boots correctly

### Motor won't move / LED blinking red

```bash
cd src/IK && python3 check_errors.py      # decode hardware error flags
cd src/IK && python3 diagnose_motors.py    # check connectivity
```

1. If `check_errors.py` shows **Overload Error** — the motor hit a mechanical limit or the arm collided. Power-cycle 12V, check for jams.
2. If `diagnose_motors.py` shows **0/5 motors found** — check USB cable, OpenRB-150 power, and that firmware is uploaded.
3. If some motors found but not all — check the daisy-chain cable between the last responding motor and the first missing one.

### Some motors respond, others don't

This usually means a **daisy-chain break**:
1. Run `python3 diagnose_motors.py` to see which motors respond
2. The break is between the **last responding motor** and the **first missing motor** in the chain
3. Check the cable between those two motors
4. Also check that all motors are set to **baud 115200** (run Dynamixel Wizard 2.0 per Step 0)

### Detection sees phantom balls / false positives

```bash
cd src && python3 vision/stream_debug.py
```

Look at the rejected contours (× markers). If many are passing validation:
1. Tighten HSV ranges (redo Step 4 — `hsv_tuner.py`)
2. Increase `min_radius` if small noise is passing the area gate
3. Check for reflective surfaces in the workspace that create colour patches

---

## Development History

| Attempt | Approach | Outcome |
|---|---|---|
| 1 | CNN (MobileNetV2 transfer learning) | ❌ Too little data, 200–500 ms/frame, overfit |
| 2 | Complex pipeline (Kalman + hand detection + motion) | ❌ Fragile, false negatives, hard to debug |
| 3 | HSV + Hough + SVM ensemble (current) | ✅ 98–100% detection, ~20–25 FPS |

**Key lesson:** For known, controlled objects under lab conditions, classical computer vision (calibrated HSV + geometry) outperforms ML in reliability, speed, and debuggability.

---

## License

MIT — see [LICENSE](LICENSE) for details.

---

**Team Autonomia — Bachelor 2026**
