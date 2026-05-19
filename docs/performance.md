# System Performance Metrics

> **Autonomia — Autonomous Sorting Robot Arm**
> Bachelor Project 2026

This document presents measured and planned performance metrics for the
complete ball-sorting system. Where formal measurements have been completed
the results are stated directly; metrics that still require controlled
evaluation are marked **TBD** with the measurement methodology defined so
they can be collected during final evaluation.

---

## 1  Detection Performance

Metrics from the [`SimpleBallDetector`](../src/vision/detector.py) ensemble
pipeline running on the OAK Series 2 camera (IMX378 sensor) at
640 × 400 processing resolution, scaled to 0.75× internally (480 × 300).

| Metric | Value | Source |
|--------|-------|--------|
| Red ball detection accuracy | ~98–100 % | README — live tests with OAK Series 2 |
| Blue ball detection accuracy | ~97–100 % | README — live tests with OAK Series 2 |
| False positive rate | < 2 % | README — verified via [`stream_debug.py`](../src/diagnostics/stream_debug.py) |
| Detection FPS (full pipeline) | ~20–25 FPS | README — measured on Raspberry Pi 5 |
| Colour classification accuracy | TBD | Run 50-ball test (25 red, 25 blue), record SVM-corrected label vs. ground truth. Target ≥ 95 %. |
| Confidence threshold | ≥ 50 % | Code — [`BALL_CONFIDENCE_THRESHOLD`](../src/config/vision.py:25) |
| Minimum confidence after gate filters | ≥ 90 % | Code — [`_validate_contour()`](../src/vision/detector.py:855) guarantees 90 % floor for any ball passing all shape gates |

### Detection Pipeline Breakdown

The 12-step pipeline in [`SimpleBallDetector.detect_balls()`](../src/vision/detector.py:1059) is:

| Step | Operation | Cost contribution |
|------|-----------|-------------------|
| 1 | Frame input (OAK-D BGR) | — |
| 2 | Resize to 0.75× | Minimal |
| 3 | Lighting analysis (LOW / MEDIUM / HIGH) | ~0.2 ms |
| 4 | CLAHE compensation (LAB L-channel, only at LOW light) | ~1 ms when active |
| 5 | BGR → HSV + Grayscale conversion | ~0.5 ms |
| 6 | Gaussian blur 5×5 on grayscale | ~0.3 ms |
| 7 | Multi-range HSV detection (2 red + 2 blue ranges) | ~2–4 ms |
| 8 | Hough Circle Transform (every frame) | ~3–5 ms |
| 9 | Ensemble merge (Union-Find clustering) | < 0.1 ms |
| 10 | SVM colour verification (≥ 75 % confidence override) | ~0.5 ms |
| 11 | Kalman filter tracking (stable IDs across frames) | < 0.1 ms |
| 12 | NMS + per-colour limit | < 0.1 ms |

---

## 2  Pick-and-Place Performance

| Metric | Value | Method |
|--------|-------|--------|
| Pick success rate (target) | ≥ 80 % | README — target across 5 workspace positions in [`08_pick_test.py`](../src/calibration/08_pick_test.py) |
| Pick success rate (measured) | TBD | Run [`python src/calibration/08_pick_test.py`](../src/calibration/08_pick_test.py) at the 5 standard positions (Centre, Near, Far, Left, Right). Score: pass / partial / fail per position. |
| End-to-end cycle time | TBD | Time from first ball detection to arm returning to HOME after drop, averaged over 20 consecutive cycles. Expected: 8–15 s depending on reach distance. |
| IK positioning accuracy (after sag calibration) | TBD | At 5 reach distances (12–36 cm), compare commanded Z with ruler-measured claw-tip height. Target: ± 3 mm after quadratic sag correction. |
| Claw grip reliability (50 mm balls) | TBD | 20 consecutive grab–lift–release cycles at centre position. Count clean grips vs. drops. |

### Cycle Phase Timing (estimated)

| Phase | Duration | Notes |
|-------|----------|-------|
| Scan (5 frames, pick-best) | ~0.3–0.5 s | [`scan_for_balls(num_frames=5)`](../src/ik/vision_bridge.py:475) |
| Approach — single direct move | ~1.5 s | [`MOVE_SETTLE_TIME`](../src/main.py:64) = 1.5 s |
| Grab (claw close + dwell) | ~0.8 s | [`GRAB_DWELL`](../src/config/arm.py:36) = 0.8 s |
| Lift to clearance height | ~1.5 s | CLEARANCE_HEIGHT = 15 cm |
| Return to SCAN_POSE | ~1.5 s | |
| Drop (claw open + dwell) | ~0.5 s | [`RELEASE_DWELL`](../src/config/arm.py:37) = 0.5 s |
| **Total (estimated)** | **~6–8 s** | Excludes measurement pause in current code |

---

## 3  Timing Instrumentation

The `CycleTimer` class in [`src/main.py`](../src/main.py) provides built-in timing instrumentation for every pick-and-place cycle. It records the wall-clock duration of each phase (scan, approach, grab, sort, drop) and prints a per-cycle summary after each successful cycle, plus a session-level summary on exit.

### What it measures

`CycleTimer` tracks five named phases that map directly to state machine transitions:

| Phase | Description | Expected Duration |
|-------|-------------|-------------------|
| scan | Camera capture + HSV detection | ~0.3s |
| approach | IK solve + motor move to target | ~1.8s |
| grab | Claw close + dwell + grip verify | ~0.5s |
| sort | Transit to drop zone | ~1.2s |
| drop | Claw open + release dwell | ~0.4s |
| **total** | **Full pick-sort cycle** | **~4.2s** |

### Per-cycle output

After each successful cycle (ball picked, sorted, and dropped), the timer prints:

```
⏱️  Cycle time: 4.20s (scan: 0.30s, approach: 1.80s, grab: 0.50s, sort: 1.20s, drop: 0.40s)
```

### Session summary

On program exit (graceful shutdown or `KeyboardInterrupt`), a session summary is printed with aggregate statistics across all completed cycles:

```
SESSION TIMING SUMMARY (5 cycles)
approach: avg 1.80s, grab: avg 0.50s, scan: avg 0.30s, sort: avg 1.20s, drop: avg 0.40s, total: avg 4.20s
```

### Thesis usage

This timing data is intended for the performance evaluation section of the thesis. It provides empirical cycle-time measurements that can be compared against the estimated durations in Section 2 above, and used to identify bottleneck phases for future optimisation.

---

## 4  System Latency

| Metric | Value | Method |
|--------|-------|--------|
| Camera-to-detection latency | TBD | Timestamp the `cam.read()` call and the return of `detect_balls()`. Average over 100 frames. Expected: 30–50 ms at 20–25 FPS. |
| Detection-to-arm-movement latency | TBD | Timestamp the IK solve + serial write + firmware ACK. Expected: 5–15 ms (JSON encode + serial round-trip at 115 200 baud). |
| Total latency (ball placement → arm starts moving) | TBD | Place a ball and measure wall-clock time until the first motor moves. Includes scan interval + detection + IK + serial. Expected: 0.5–1.5 s (dominated by scan frame capture). |
| Firmware ACK round-trip | TBD | Time from `ser.write()` to `ser.readline()` returning `OK\n`. Expected: < 10 ms. |

---

## 5  Reliability

| Metric | Value | Method |
|--------|-------|--------|
| Failure recovery time | TBD | Induce an overload error (block a motor), measure time from error detection to resumed operation after 12 V power cycle and restart. |
| Mean time between failures | TBD | Run continuous sorting for 60 min, record any motor errors, detection failures, or communication timeouts. |
| Consecutive successful sorts | TBD | Count the longest uninterrupted sequence of pass-rated picks in a multi-ball test run (≥ 20 balls). |
| Motor overload incidence | TBD | Track overload error flags (bit 5) via [`check_motor_errors.py`](../src/diagnostics/check_motor_errors.py) over 50 cycles. |

---

## 6  IK Solver Accuracy

The geometric 4-DOF IK solver in [`ArmIK`](../src/ik/solver.py:29) uses:

| Parameter | Value |
|-----------|-------|
| L1 (Shoulder → Elbow) | 25.5 cm |
| L2 (Elbow → Wrist) | 23.0 cm |
| L3 (Wrist → Claw tip) | 22.0 cm |
| Shoulder height | 35.0 cm |
| Max reach (L1 + L2) | 48.5 cm |
| Min reach (L1 − L2) | 2.5 cm |
| Sag compensation model | Linear (`z_offset_multiplier = 0.04`) or quadratic (from [`sag_calibration.json`](../src/ik/solver.py:118)) |
| Z minimum (floor clamp) | 6.0 cm |
| Joint limits | m2, m3, m4: [600, 3500] steps |

---

## 7  Test Protocol — Standardised 10-Ball Test

Use this protocol for reproducible system-level evaluation.

### Prerequisites

- Calibration complete (Steps 0–8)
- 5 red balls + 5 blue balls (50 mm diameter)
- Workspace cleared, bins in position
- System running: `python src/main.py --real-serial --real-camera`

### Procedure

1. **Warm-up** — let the camera auto-exposure stabilise for 30 s (this happens automatically via the AE warmup in [`OAKCamera.open()`](../src/vision/camera.py:58)).
2. **Place balls** — place all 10 balls in the workspace at varied positions spanning the reachable area (12–36 cm from shoulder, ±12 cm lateral).
3. **Start timer** — record wall-clock time at "GO".
4. **Observe** — the system scans, picks, and sorts autonomously. Do not intervene unless a motor error occurs.
5. **Score each pick** — for each ball, record:
   - Detection: was the ball detected? (Y/N)
   - Colour: was the colour classified correctly? (Y/N)
   - Pick: did the claw grip the ball cleanly? (pass / partial / fail)
   - Sort: was the ball placed in the correct bin? (Y/N)
6. **Stop timer** — record wall-clock time when all balls are sorted or the system reports "workspace is clear".
7. **Compute metrics:**
   - Detection rate = (balls detected) / 10
   - Colour accuracy = (correct colour labels) / (balls detected)
   - Pick success rate = (pass picks) / (attempted picks)
   - Sort accuracy = (correct bin placements) / (successful picks)
   - Total time = stop − start
   - Mean cycle time = total time / 10

### Pass Criteria

| Metric | Threshold |
|--------|-----------|
| Detection rate | ≥ 90 % (9/10) |
| Colour accuracy | ≥ 95 % |
| Pick success rate | ≥ 80 % |
| Sort accuracy | 100 % (given correct colour) |
| Mean cycle time | ≤ 15 s |

### Recording Template

```
Date:       ____-__-__
Operator:   ____________
Lighting:   __________ (LOW / MEDIUM / HIGH as reported by detector)

Ball  Colour  Detected  Colour-OK  Pick    Bin-OK  Notes
───── ─────── ───────── ────────── ─────── ─────── ──────
 1    Red     Y         Y          pass    Y
 2    Blue    Y         Y          pass    Y
 3    Red     Y         Y          partial -       claw too high
 4    ...
```

---

## 8  Known Limitations

- **No mid-approach visual correction** — the arm moves directly to the grab position in a single step; a mid-approach re-scan is not possible because the wrist-mounted camera is occluded by the claw during approach (see ADR-003).
- **Single-ball-at-a-time** — the system picks one ball per scan round and rescans. Batch processing (pick without rescanning) was considered but rejected to ensure fresh position data.
- **No depth-based height compensation** — Z is assumed 0 (table surface). Balls on uneven surfaces would cause Z errors.
- **Lighting sensitivity** — HSV ranges are calibrated for 300–700 lux. Outside this range, recalibration is required (Steps 4–5).
