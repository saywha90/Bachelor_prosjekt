# Autonomia — Autonomous Sorting Robot Arm

**Bachelor Project 2026 — 4-DOF robotic arm with vision-guided pick-and-place**

A complete autonomous sorting system that uses an OAK-D camera to detect coloured balls, computes inverse kinematics, and commands a 4-DOF robotic arm via serial to sort objects into bins.

---

## System Overview

```
OAK-D Camera ──► VisionBridge ──► State Machine ──► IK Solver ──► OpenRB-150 ──► Dynamixel Motors
  (RGB stream)    (homography)      (main.py)      (pi_kinematics)   (serial)       (4 axes)
```

| Component | Hardware | Software |
|---|---|---|
| Camera | Luxonis OAK Series 2 (IMX378) | `oak_camera.py` + `enhanced_detector.py` |
| Controller | Raspberry Pi 5 | `main.py` (Python 3) |
| Motor Controller | OpenRB-150 | `openrb_bridge.ino` (Arduino) |
| Motors | Dynamixel (4×) | Steps 0–4095, centre = 2048 |

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

| Link | Length | Description |
|---|---|---|
| L1 | 22.5 cm | Shoulder → Elbow |
| L2 | 20.5 cm | Elbow → Wrist pivot |
| L3 | 11.0 cm | Wrist → Claw tip |

### Bin Positions (configure in `src/IK/config.py`)

All coordinates are in **centimetres relative to the shoulder joint origin** (x = forward, y = left/right, z = up).

| Bin | x | y | z |
|---|---|---|---|
| RED_BIN | 20.0 | 15.0 | 10.0 |
| BLUE_BIN | 20.0 | −15.0 | 10.0 |
| REJECT_BIN | 10.0 | 15.0 | 10.0 |
| HOME | 10.0 | 0.0 | 15.0 |

---

## Running the Full System

### Toggle hardware in `src/IK/main.py`

```python
USE_REAL_SERIAL = False     # True → real OpenRB-150 on SERIAL_PORT
USE_REAL_CAMERA = False     # True → real OAK-D camera + homography
SERIAL_PORT     = "/dev/ttyACM0"
```

### State Machine Flow

```
HOME → SCANNING → [queue built] → APPROACHING (80%) → [correction image]
     → APPROACHING (100%) → GRABBING → SORTING → DROPPING → HOME
     → [next in queue] → ... → [queue empty] → SCANNING → ...
```

After each scan round, if the camera is live it rescans the workspace for any remaining objects (up to 3 rounds). In simulation mode it processes the fake queue once and exits.

---

## Homography Calibration (one-time setup)

The camera is mounted on a side pillar looking at a downward angle. Simple distance estimation fails because of the perspective. We use a **4-point perspective transform** to map pixels → cm accurately.

### Step 1 — Capture calibration points

```bash
cd src
python3 vision/calibrate_camera.py
```

- Left-click the **4 corners of your physical workspace** in the camera feed
- Order: **top-left → top-right → bottom-right → bottom-left**
- Press `c` to clear and retry, `q` to quit

The tool prints a ready-to-paste array:

```python
WORKSPACE_PX = np.float32([
    [ 102,  58],   # top-left
    [ 538,  62],   # top-right
    [ 576, 358],   # bottom-right
    [  64, 362],   # bottom-left
])
```

### Step 2 — Set real-world corners

Measure the 4 corners of your workspace in cm from the arm's shoulder:

```python
# In src/IK/vision_bridge.py
WORKSPACE_CM = np.float32([
    [35.0,  15.0],   # top-left      (far-left)
    [35.0, -15.0],   # top-right     (far-right)
    [10.0, -15.0],   # bottom-right  (near-right)
    [10.0,  15.0],   # bottom-left   (near-left)
])
```

### Step 3 — Enable the camera

```python
# In src/IK/main.py
USE_REAL_CAMERA = True
```

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

If you change balls or the lighting environment:

```bash
# 1. Capture new training images
python3 src/vision/capture_training_data.py --color red
python3 src/vision/capture_training_data.py --color blue

# 2. Analyse images → get suggested HSV ranges
python3 src/vision/recalibrate_hsv.py training_data

# 3. Paste the suggested ranges into enhanced_detector.py

# 4. Fine-tune live
python3 src/vision/diagnose_detection.py     # click to read H/S/V per pixel
python3 src/vision/hsv_tuner.py              # trackbar-based live tuner

# 5. Retrain the SVM (optional but recommended)
python3 src/vision/train_color_classifier.py --data_dir training_data
```

---

## IK Solver

**File:** `src/IK/pi_kinematics.py`

Geometric 4-DOF inverse kinematics with:
- End-effector offset compensation (L3 wrist extension)
- Sag/droop compensation (`z_offset_multiplier = 0.08`, tune empirically)
- Two-step visual servoing: `calculate_partial_move(target, percentage=0.80)`

```python
from pi_kinematics import ArmIK

arm = ArmIK()
steps = arm.solve(x=20.0, y=5.0, z=0.0)
# → {"m1": 2048, "m2": 1820, "m3": 2201, "m4": 2075}

json_cmd = arm.solve_to_json(20.0, 5.0, 0.0)
# → '{"m1": 2048, "m2": 1820, "m3": 2201, "m4": 2075}'
```

Motor convention: all Dynamixel XL/MX series, 0–4095 steps, centre = 2048 = 180°.

---

## Arduino Firmware

**File:** `src/IK/openrb_bridge.ino`

Upload to **OpenRB-150** via Arduino IDE (select board: OpenRB-150).

The firmware:
- Reads newline-terminated JSON from serial: `{"m1":2048,"m2":1820,"m3":2201,"m4":2075}`
- Moves all 4 Dynamixel motors to target positions with velocity control
- Replies `OK\n` when movement is complete
- Replies `ERR\n` on malformed input

Serial settings: **115200 baud, 8N1**.

---

## Arduino Upload Steps

1. Install the **Dynamixel2Arduino** library via Arduino IDE Library Manager
2. Open `src/IK/openrb_bridge.ino`
3. Board: **OpenRB-150** (install via Boards Manager → `ROBOTIS`)
4. Port: `/dev/ttyACM0` (Linux/Mac) or `COMx` (Windows)
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
