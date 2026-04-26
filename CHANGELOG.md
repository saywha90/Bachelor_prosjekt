# Changelog

All notable changes to this project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

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
