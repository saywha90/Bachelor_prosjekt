# Troubleshooting Guide

> **Autonomia — Autonomous Sorting Robot Arm**
> Bachelor Project 2026

This guide covers common error conditions, their root causes, and solutions.
Organised by subsystem: motors, camera, detection, calibration, and
general software issues.

---

## Quick Diagnostic Reference

| Symptom | Tool | Command |
|---------|------|---------|
| No motors respond / some missing | [`diagnose_motors.py`](../src/diagnostics/diagnose_motors.py) | `python src/diagnostics/diagnose_motors.py` |
| Motor LED blinking red / motor stuck | [`check_motor_errors.py`](../src/diagnostics/check_motor_errors.py) | `python src/diagnostics/check_motor_errors.py` |
| Balls not detected / wrong colour | [`diagnose_detection.py`](../src/diagnostics/diagnose_detection.py) | `python src/diagnostics/diagnose_detection.py` |
| Need per-contour rejection details | [`stream_debug.py`](../src/diagnostics/stream_debug.py) | `python src/diagnostics/stream_debug.py` |

---

## 1  Motor Errors

### 1.1  No motors respond (0/5 detected)

**Symptom:** `diagnose_motors.py` reports `0/5 motors detected` or the
main loop never receives `OK` from the firmware.

**Possible causes and fixes:**

| # | Cause | Fix |
|---|-------|-----|
| 1 | USB cable disconnected | Check USB-C between host and OpenRB-150 |
| 2 | 12 V power supply off | The OpenRB-150 needs external 12 V — USB alone is not enough for motors |
| 3 | Firmware not uploaded | Upload [`openrb_bridge.ino`](../firmware/openrb_bridge/openrb_bridge.ino) via Arduino IDE |
| 4 | Wrong serial port | Run `ls /dev/cu.usbmodem*` (macOS) or `ls /dev/ttyACM*` (Linux) to find the port. Update `SERIAL_PORT` in [`main.py`](../src/main.py:52) |
| 5 | Another program holds the port | Close Arduino IDE Serial Monitor or any other serial terminal |

### 1.2  Some motors respond, others don't

**Symptom:** `diagnose_motors.py` finds motors 1–3 but not 4–5 (or similar
partial result).

**Root cause:** Daisy-chain break. The cable between the last responding
motor and the first missing motor is disconnected or damaged.

**Fix:**

1. Identify which motors are found and which are missing.
2. Check the TTL cable between the **last found** motor and the **first
   missing** motor in the chain.
3. If the cable looks fine, verify the missing motor's ID and baud rate with
   Dynamixel Wizard 2.0 (connect it directly to the U2D2, bypassing the
   chain).

### 1.3  Motor LED blinking red

**Symptom:** One or more motor LEDs blink red. The motor may feel limp
(torque lost) but still responds to pings.

**Root cause:** A latched hardware error flag. Common error types:

| Bit | Error | Common Cause |
|:---:|-------|-------------|
| 0 | Input Voltage Error | Power supply issue — check 12 V |
| 2 | Overheating Error | Motor running too long under load — let it cool |
| 3 | Motor Encoder Error | Internal motor fault — may need replacement |
| 4 | Electrical Shock Error | Wiring short or bad connection |
| 5 | Overload Error | Mechanical jam, arm collision, or excessive load |

**Diagnosis:**

```bash
python src/diagnostics/check_motor_errors.py
```

The script reads the hardware error status register from all 5 motors and
decodes the bit flags. See [`parse_error_status()`](../src/diagnostics/check_motor_errors.py:35)
for the flag definitions.

**Fix:**

> ⚠️ **Hardware errors require a 12 V power cycle to clear.** Software
> commands alone cannot reset them.

1. Note which motors have errors and the error type.
2. **Power off** the 12 V supply.
3. If **Overload Error**: check for mechanical jams, collisions, or the arm
   hitting joint limits. The IK solver clamps to safe ranges
   ([`JOINT_LIMITS`](../src/ik/solver.py:78): m2/m3/m4 = [600, 3500])
   but extreme targets near the boundary can still cause transient overloads.
4. **Power on** the 12 V supply.
5. Re-run `check_motor_errors.py` to confirm all errors are cleared.

### 1.4  Motor overload during operation

**Symptom:** The arm suddenly goes limp mid-cycle. The main loop prints
`⚠️ Unexpected response` instead of `OK`.

**Root cause:** A motor exceeded its torque limit. Common during:
- Aggressive moves to extreme positions (shoulder fully extended + low Z)
- The HOME position being too "folded" (e.g., the old HOME of (10, 0, 15)
  caused extreme elbow folding at m3 = 273, triggering overload on the
  XM430)

**Fix:**
1. Power-cycle 12 V.
2. If the HOME position caused it, adjust [`HOME_POSITION`](../src/config/arm.py:28)
   to a less stressful pose. The current default of `(20.0, 0.0, 30.0)`
   keeps the shoulder upright to avoid overload.
3. Review [`JOINT_LIMITS`](../src/ik/solver.py:78) — tightening them prevents
   the solver from requesting positions that cause overload.

### 1.5  Serial communication: no `OK` response

**Symptom:** `ser.readline()` returns empty or `ERR`.

**Checklist:**

1. Baud rate mismatch — both host and firmware must use 115 200.
2. `SERIAL_PORT` in [`main.py`](../src/main.py:52) doesn't match the
   actual device. Use `ls /dev/tty*` to list available ports.
3. Firmware not uploaded or crashed — re-upload
   [`openrb_bridge.ino`](../firmware/openrb_bridge/openrb_bridge.ino).
4. The firmware replies `ERR\n` on malformed JSON — check the JSON command
   format. Expected: `{"m1":2048,"m2":1820,"m3":2201,"m4":2075,"m5":2048}\n`.

### 1.6  Smooth startup fails

**Symptom:** `smooth_startup()` prints `⚠️ Could not parse positions` and
falls back to a direct HOME command.

**Root cause:** The firmware's `read_pos` command returned unexpected data
(e.g., a boot message was still in the buffer).

**Fix:** This is a non-critical fallback — the arm still reaches HOME, just
without the gradual ramp. If it happens consistently, increase the boot
delay in [`main.py`](../src/main.py:435) (currently 3 s).

---

## 2  Camera Issues

### 2.1  OAK-D camera not found

**Symptom:** `OAKCamera.open()` returns `False` and prints
`❌ OAKCamera: Could not open camera`.

**Possible causes and fixes:**

| # | Cause | Fix |
|---|-------|-----|
| 1 | USB cable not connected | Check USB-C between camera and host |
| 2 | USB hub or adapter issue | OAK-D prefers USB 3.0 — try a direct connection |
| 3 | `depthai` not installed | `pip3 install depthai` |
| 4 | Device permissions (Linux) | Add udev rules for Luxonis devices |
| 5 | Another process is using the camera | Close other scripts that access the OAK-D |

**Quick test:**

```bash
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

If this returns an empty list, the camera is not detected at the USB level.

### 2.2  Camera image is dark on startup

**Symptom:** First few frames are nearly black (mean brightness ~32/255).

**Root cause:** The OAK-D's auto-exposure (AE) and auto-white-balance (AWB)
need time to converge, especially over USB 2.0.

**Fix:** The [`OAKCamera.open()`](../src/vision/camera.py:58) method already
discards the first 30 frames (`_AE_WARMUP_FRAMES = 30`). If images are
still dark, increase this value:

```python
# In src/vision/camera.py
_AE_WARMUP_FRAMES = 50   # increase if still dark after startup
```

### 2.3  Camera pipeline crashes after extended use

**Symptom:** `cam.read()` starts returning `(False, None)` after running
for a long time.

**Possible causes:**
- USB bandwidth issues (especially with USB 2.0 adapters)
- MyriadX VPU thermal throttling

**Fix:**
1. Use a USB 3.0 port or cable.
2. Ensure the camera has adequate ventilation.
3. The [`VisionBridge.scan_for_balls()`](../src/ik/vision_bridge.py:475)
   method handles `read()` failures gracefully — it simply skips that frame.

---

## 3  Detection Problems

### 3.1  Balls not detected

**Symptom:** `scan_for_balls()` returns an empty list even though balls
are visible in the workspace.

**Diagnosis:**

```bash
python src/diagnostics/diagnose_detection.py
```

This opens 4 windows: Original, Red Mask, Blue Mask, and Overlay. Left-click
on a ball to print its H/S/V pixel values.

**Common causes:**

| Cause | How to identify | Fix |
|-------|----------------|-----|
| HSV ranges don't cover ball colour | Click on the ball in the Original window; the printed H/S/V values are outside the configured ranges | Re-run Step 4 (`python src/calibration/04_hsv_tuner.py`) |
| Lighting too dim (< 300 lux) | `lighting_level` shows "low"; mask is patchy | Increase ambient lighting or lower S/V minimums in [`SimpleBallDetector`](../src/vision/detector.py:322) |
| Lighting too bright (> 700 lux) | Specular highlights wash out ball colour | Reduce direct lighting; the CLAHE compensation only helps at low light |
| Ball too small in frame | Ball radius < `min_radius` (10 px at native res) | Move the ball closer to the camera, or reduce `BALL_MIN_RADIUS` in [`config/vision.py`](../src/config/vision.py:23) |
| Ball too large in frame | Ball radius > `max_radius` (150 px at native res) | Move ball farther away, or increase `BALL_MAX_RADIUS` in [`config/vision.py`](../src/config/vision.py:24) |
| Morphological kernel closing gaps too aggressively | The 13×13 close kernel merges ball with background | Reduce kernel size in [`SimpleBallDetector.__init__()`](../src/vision/detector.py:346) |

### 3.2  False positives (phantom balls)

**Symptom:** The system detects balls where there are none, or detects
non-ball objects (red notebook, blue cable) as balls.

**Diagnosis:**

```bash
python src/diagnostics/stream_debug.py
```

This shows a 4-panel display with per-contour rejection reasons and
confidence scores. Look at the accepted detections (● markers) — are any
on non-ball objects?

**Fixes:**

1. **Tighten HSV ranges** — redo Step 4 with `python src/calibration/04_hsv_tuner.py`.
   Focus on raising the S (saturation) minimum to reject pale/desaturated
   objects.
2. **Increase `min_radius`** if small noise passes the area gate.
3. **Remove coloured objects** from the workspace — the detector cannot
   distinguish a red ball from a red notebook based on HSV alone (shape
   gates catch many non-spherical objects, but not all).
4. **Circularity threshold** — the default is ≥ 0.82 in
   [`_validate_contour()`](../src/vision/detector.py:802). Increase to
   0.85 if cube-shaped objects are passing.
5. **Raise confidence threshold** — increase
   [`BALL_CONFIDENCE_THRESHOLD`](../src/config/vision.py:25) from 0.50 to
   0.60 or higher.

### 3.3  Wrong colour classification

**Symptom:** A red ball is classified as blue, or vice versa.

**Root cause:** HSV hue ranges overlap or the SVM colour verifier makes
an incorrect correction.

**Diagnosis:**

1. Run `python src/diagnostics/diagnose_detection.py` and click on the
   misclassified ball. Check whether its H value falls in the red range
   (0–6 or 165–179) or the blue range (98–118).
2. If the H value is ambiguous (e.g., a purple-ish ball), the SVM may
   override the HSV label. The SVM correction threshold is ≥ 75 % confidence
   — see [`_verify_with_svm()`](../src/vision/detector.py:1049).

**Fix:**
1. Tighten HSV ranges so they don't overlap.
2. Retrain the SVM classifier with `python src/training/train_classifier.py`
   using freshly captured data (`python src/training/capture_data.py`).
3. As a last resort, increase the SVM threshold from 0.75 to 0.85 in
   [`_verify_with_svm()`](../src/vision/detector.py:1049).

### 3.4  Detection is intermittent (ball appears and disappears)

**Symptom:** Ball detection flickers — the ball is detected in some frames
but not others, causing the tracker to repeatedly create and destroy tracks.

**Root cause:** The ball is near a detection threshold boundary (e.g.,
confidence just above/below 50 %, or circularity near 0.82).

**Fix:**
1. The Kalman tracker in [`BallTracker`](../src/vision/detector.py:60)
   "coasts" for up to `max_disappeared` frames (default: 2). Increase this
   to 4–6 for less aggressive track dropout.
2. Lower the `confidence_threshold` slightly (e.g., 0.45 instead of 0.50).
3. Check if the ball has strong specular highlights that fragment the HSV
   mask. The hole-filling algorithm (flood fill) in
   [`_apply_hsv_ranges()`](../src/vision/detector.py:529) should handle
   this, but extreme glare may still cause issues.

---

## 4  Calibration Issues

### 4.1  Bad homography (detected positions are wrong)

**Symptom:** The vision bridge reports ball positions in cm that are
significantly different from their actual positions (> 2 cm error).

**Common causes:**

| Cause | Fix |
|-------|-----|
| Corner positions measured incorrectly (not from shoulder joint) | Re-measure all 4 corners from the **shoulder joint** (motor 2 pivot). CAMERA_OFFSET_X/Y should be 0.0. Ensure arm is at SCAN_POSE during calibration. |
| Clicked wrong pixel positions during calibration | Re-run `python src/calibration/06_homography.py` and click more carefully |
| Camera has moved since calibration | Re-run Step 6 |
| Using default hardcoded calibration instead of JSON | Check that `homography_calibration.json` exists in the `src/calibration/` directory |

### 4.2  Sag calibration drift

**Symptom:** The arm was accurate after sag calibration but gradually
became less accurate over days/weeks.

**Root cause:** Mechanical wear, loosened joints, or temperature changes
affecting motor compliance.

**Fix:**
1. Re-run Step 3: `python src/calibration/03_sag.py`.
2. If the quadratic model's R² is < 0.90, measure more carefully — ensure
   the ruler is perpendicular to the table and the claw is fully settled
   before reading.

### 4.3  HSV ranges don't work in new lighting

**Symptom:** Detection worked in the lab but fails in a different room or
at a different time of day.

**Root cause:** HSV ranges are sensitive to lighting colour temperature
and intensity. The adaptive lighting system handles 300–700 lux but cannot
compensate for extreme changes.

**Fix:**
1. Re-run Step 4: `python src/calibration/04_hsv_tuner.py` under the new
   lighting conditions.
2. Optionally run Step 5: `python src/calibration/05_hsv_refine.py` for
   statistical bounds.
3. The detector's [`get_adaptive_hsv_ranges()`](../src/vision/detector.py:447)
   already adjusts S/V thresholds by ± 10–20 units based on lighting level,
   but this may not be enough for major lighting changes.

### 4.4  Parallax errors at workspace edges

**Symptom:** Detection accuracy is fine at the centre but degrades at the
edges of the workspace.

**Root cause:** The homography assumes a flat Z = 0 plane. Balls that are
elevated (stacked, or on an uneven surface) or at the workspace edge where
the camera views at a steeper angle will have parallax error.

**Fix:**
1. Verify camera height matches [`CAMERA_HEIGHT`](../src/config/arm.py:53)
   — run Step 6b.
2. A 5 cm camera height error causes ~3 mm parallax at edges.
3. Keep the workspace surface flat and level.

---

## 4b  Wrist-mounted camera issues

### 4b.1  Balls detected at wrong coordinates

**Symptom:** The vision bridge reports ball positions that are offset from
their actual positions, even though the homography was recently calibrated.

**Root cause:** Most likely the arm wasn't at `SCAN_POSE` when vision ran.
The wrist-mounted camera's homography is only valid at the exact joint
configuration where it was calibrated.

**Fix:**
1. Check logs for the `verify_pose` warning — this indicates the arm was
   not at `SCAN_POSE` when scanning began.
2. If `SCAN_POSE` was changed without recalibrating homography, redo
   Step 06 (`python src/calibration/06_homography.py`).
3. Verify the arm consistently reaches `SCAN_POSE` before scanning by
   watching the state machine transitions.

### 4b.2  Claw appears in camera view during scanning

**Symptom:** The camera image shows part of the claw assembly, potentially
causing false detections or occluding the workspace.

**Root cause:** `SCAN_POSE` wrist angle (m4) is wrong; the camera is
looking partly at its own claw.

**Fix:** Re-tune Step 02c — adjust the m4 (wrist tilt) value in
`SCAN_POSE` so the camera looks past the claw. The claw should be fully
open during scanning.

### 4b.3  Camera doesn't see the whole workspace

**Symptom:** Balls placed at the edges of the workspace are not detected
because they fall outside the camera's field of view at `SCAN_POSE`.

**Root cause:** The camera height or angle at `SCAN_POSE` doesn't provide
sufficient coverage. The OAK-D S2 with 81° HFOV covers roughly a 50×30 cm
area at 30 cm height.

**Fix:**
1. Lift the arm higher (adjust m2 shoulder value in `SCAN_POSE`).
2. Tilt the wrist more (adjust m4 in `SCAN_POSE`).
3. Alternatively, shrink the workspace boundaries in the homography
   calibration (Step 06).

### 4b.4  Coordinate accuracy degraded after some operation

**Symptom:** Pick accuracy was good initially but has degraded over time
or after certain operations.

**Root cause:** Motor positions may have drifted from `SCAN_POSE` between
operations due to loose belts, slipped gears, or hardware errors.

**Fix:**
1. Re-run Step 02c verification — manually check that the arm reaches the
   expected `SCAN_POSE` joint positions.
2. Use `python src/diagnostics/diagnose_motors.py` to check for motor errors.
3. If positions have drifted, re-calibrate `SCAN_POSE` (Step 02c) and then
   re-run the homography calibration (Step 06).

---

## 5  Common Error Messages

### `zsh: command not found: python`

macOS doesn't include `python` — use `python3`:

```bash
python3 src/main.py
```

### `ModuleNotFoundError: No module named 'cv2'`

OpenCV is not installed:

```bash
pip3 install -r requirements.txt
```

### `ModuleNotFoundError: No module named 'depthai'`

The OAK-D SDK is not installed:

```bash
pip3 install depthai
```

### `serial.SerialException: could not open port`

The serial port is busy or doesn't exist:

1. Check the port name: `ls /dev/cu.usbmodem*` (macOS) or `ls /dev/ttyACM*` (Linux).
2. Close Arduino IDE Serial Monitor or any other program using the port.
3. Unplug and re-plug the USB cable.

### `ValueError: Target (x, y, z) is too close`

The IK solver cannot reach the target because it's inside the minimum reach
envelope (|L1 − L2| = 2.5 cm):

1. This usually means the homography mapped a ball to coordinates very close
   to the shoulder joint.
2. Re-run Step 6 to verify the homography.

### `[IK WARNING] ⚠️ Target ... is beyond max reach`

The target is outside the arm's maximum reach (L1 + L2 = 48.5 cm). The
solver automatically clamps to 99 % of max reach and preserves the
direction:

1. Check that the homography isn't mapping edge pixels to extreme
   coordinates.
2. Run Step 6b to verify the scan region stays within the arm's reachable
   area.

### `[IK WARNING] ⚠️ m2=600 is AT JOINT LIMIT`

A motor step value has been clamped to a joint safety limit. This prevents
overload errors but means the arm can't reach the exact target:

1. If this happens frequently, the workspace may extend beyond the arm's
   comfortable range. Reduce the homography's physical extent.
2. Review [`JOINT_LIMITS`](../src/ik/solver.py:78) — the defaults are
   conservative (m2/m3/m4: [600, 3500]).

### `[VISION] ❌ Vision bridge failed to open real camera`

The main loop aborts because the camera couldn't be opened:

1. Check USB connection.
2. Re-plug the OAK-D camera.
3. Ensure no other process is using the camera.
4. Check `depthai` version: `pip3 show depthai`.

### `[INIT] ⚠️ Did not receive 'OK:READY' from OpenRB-150!`

The firmware didn't send its boot message within the timeout:

1. Check that the firmware is uploaded (re-upload if unsure).
2. The correct port is selected (`SERIAL_PORT` in [`main.py`](../src/main.py:52)).
3. The baud rate matches (115 200 on both sides).
4. Increase the boot wait time in [`main.py`](../src/main.py:435)
   (currently `time.sleep(3)`).

---

## 6  General Debugging Tips

### Enable verbose IK output

The [`ArmIK.solve()`](../src/ik/solver.py:157) method prints detailed debug
output by default, including:
- Input target coordinates
- Wrist-relative Z (`z_ik`) after sag compensation
- Joint angles in radians and degrees
- Motor step values
- Joint limit warnings

### Check motor positions manually

Send a `read_pos` command from the main loop or use a serial terminal at
115 200 baud:

```json
{"cmd": "read_pos"}
```

The firmware responds with:

```json
{"m1": 2048, "m2": 1820, "m3": 2201, "m4": 2075, "m5": 2048}
```

### Force-clear motor errors

If `check_motor_errors.py` shows errors that persist after power cycling:

1. Disconnect ALL power (12 V and USB).
2. Wait 10 seconds.
3. Reconnect USB first, then 12 V.
4. The main loop sends `{"cmd": "clear_errors"}` on startup — see
   [`main()`](../src/main.py:458).

### Test IK without hardware

Run the 3-D visualiser in simulation mode:

```bash
python src/main.py
```

Without `--real-serial` and `--real-camera` flags (defaults),
the system uses [`MockSerial`](../src/simulation/mock_serial.py) and
canned fake detections. The matplotlib 3-D window shows the arm's
calculated positions.

### Test detection without the arm

```bash
python src/diagnostics/diagnose_detection.py
```

This opens the camera and shows live HSV masks without requiring any arm
connection. Click on any pixel to read its H/S/V values.

---

## Raspberry Pi 5 — Common Issues

### "Permission denied" on `/dev/ttyACM0`

**Symptom:** `PermissionError: [Errno 13] Permission denied: '/dev/ttyACM0'` when running `main.py`.

**Cause:** Your user is not in the `dialout` group.

**Fix:**

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

After reboot, verify with `groups` — it should list `dialout`.

---

### OAK-D S2 Not Detected on Pi

**Symptom:** `RuntimeError: No DepthAI device found!` or `depthai.Device.getAllAvailableDevices()` returns an empty list.

**Checklist:**

1. **Udev rules installed?**
   ```bash
   cat /etc/udev/rules.d/80-movidius.rules
   ```
   Should contain: `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`

   If missing, install them:
   ```bash
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

2. **USB power sufficient?** The Pi 5 limits USB current unless powered by the official 27 W USB-C PSU (or equivalent 5 V / 5 A supply). If the OAK-D resets or disconnects under load, ensure you're using a sufficiently rated 5 V supply.

3. **Try a different USB port** — use a USB 3.0 (blue) port for best bandwidth.

4. **Unplug and re-plug** the OAK-D after installing udev rules.

---

### Pi Runs Out of Memory During Detection

**Symptom:** `Killed` or `MemoryError` during the vision pipeline, especially with large frames or the ML classifier.

**Fixes:**

1. **Increase swap size:**
   ```bash
   sudo dphys-swapfile swapoff
   sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
   sudo dphys-swapfile setup
   sudo dphys-swapfile swapon
   ```

2. **Disable the desktop environment** if running headless:
   ```bash
   sudo raspi-config   # → System Options → Boot → Console
   sudo reboot
   ```

3. **Close other applications** — the vision + IK pipeline can use 2–3 GB on the 8 GB Pi 5.

---

> **Full setup guide:** [docs/pi-setup.md](pi-setup.md)
