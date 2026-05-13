# Changelog

All notable changes to this project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased] – 2026-05-11

### Fixed
- **Adaptive claw grip detection overhaul** — fixed false positives and false negatives:
  - Added position guard to `_feedback_confirms_grip()`: load/current checks now require the claw to be blocked away from the fully-closed position (prevents end-stop friction false positives)
  - Added zero-resistance safeguard: `load=0 AND current<5 mA` = no ball, regardless of position
  - Lowered `GRIP_CURRENT_LIMIT` from 200→50 mA for motor sensitivity on small balls
  - Extended `CLAW_CLOSED_POS` from 3300→3350 for tighter grip on small balls
  - Lowered `GRIP_MIN_BLOCKED_WITH_SENSOR` from 60→5 steps (sensor-assisted detection)
  - Increased `GRIP_TIMEOUT` from 3→15s to prevent premature timeout during incremental close
  - Increased secure close settle time to 3.0s so motor actually reaches target
- **SCAN_POSE mismatch** — synced `arm.py` SCAN_POSE values (m2, m4) to match `homography_calibration.json`
- **SCAN_POSE retry logic** — `verify_scan_pose_before_scan()` now retries up to 3 times

### Added
- **Claw grip diagnostic tool** (`src/calibration/02b_claw_grip_test.py`) — standalone Step 2b validation script to test grip detection on real hardware with detailed sensor readouts
- Two-tier position threshold system for grip detection: "minimally blocked" (≥5 steps with sensor confirm) and "strongly blocked" (≥30 steps position-only)
- 16 unit tests covering all grip detection scenarios including two-tier thresholds and zero-resistance rejection

### Changed
- Grip profile speed: `GRIP_PROFILE_VEL` 30→80, `GRIP_PROFILE_ACC` 10→20, `GRIP_EXTRA_CLOSE` 30→50 (faster claw close)
- Detection thresholds tuned for 50 mA current limit: `GRIP_LOAD_THRESHOLD` 50→15, `GRIP_LOAD_DETECT` 15→5, current contact threshold auto-calculated as 20 mA

## [Unreleased] – 2026-04-25

### Added
- **Pick-failed recovery**: `VERIFY_GRIP` state with position-based and load-based grip verification after each grab. On failure, the system opens the claw and immediately re-scans (up to 2 retries before skipping).
- **Timing instrumentation**: `CycleTimer` class logs duration of each phase (scan, approach, grab, sort, drop) per cycle, with session-level averages on exit.
- **Firmware `read_load` command**: reads Present Load from all Dynamixel motors for grip force measurement.
- Grip verification config constants: `GRIP_VERIFY_TOLERANCE`, `GRIP_LOAD_THRESHOLD`, `MAX_PICK_RETRIES`, `VERIFY_HEIGHT`.

## [Unreleased – previous]

### Added
- `argparse` CLI flags `--real-serial` / `--real-camera` replacing hardcoded `USE_REAL_*` booleans in `src/main.py`
- `scripts/manual_tests/` directory separating hardware/GUI demo scripts from the automated pytest suite
- `src/calibration/README.md` explaining manual Steps 00–01 and listing automated Steps 02–08
- Mermaid architecture diagram added to `docs/architecture.md`

### Changed
- Vision pipeline iteration history moved from `CHANGELOG.md` to `docs/vision-history.md`
- `tests/` now contains only `test_ik_solver.py` (28 unit tests, hardware-free)

---

## [0.9.0] — 2026-04-21

### Added
- Full 4-DOF pick-and-place pipeline (`src/main.py`)
- HSV-based ball detection with homography projection (`src/vision/detector.py`)
- Geometric IK solver with sag compensation (`src/ik/solver.py`)
- Visual servoing bridge with homography-based pixel-to-cm conversion (`src/ik/vision_bridge.py`)
- OAK-D camera integration (`src/vision/camera.py`)
- Simulation mode with mock serial and 2-D visualiser (`src/simulation/`)
- Numbered calibration pipeline: Steps 02–08 (`src/calibration/`)
- Architecture Decision Records: HSV vs CNN, 4-DOF geometry, fixed scan pose
- Comprehensive docs: architecture, calibration, performance, troubleshooting

### Metrics (as of 2026-04-21)
- Detection accuracy: 98–100 % under lab lighting
- Pipeline throughput: 20–25 FPS
- IK unit test suite: 28 tests, 100 % passing
