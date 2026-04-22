# Changelog

All notable changes to this project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

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
- Two-step visual servoing: coarse IK + fine pixel-error correction (`src/ik/vision_bridge.py`)
- OAK-D camera integration (`src/vision/camera.py`)
- Simulation mode with mock serial and 2-D visualiser (`src/simulation/`)
- Numbered calibration pipeline: Steps 02–08 (`src/calibration/`)
- Architecture Decision Records: HSV vs CNN, 4-DOF geometry, two-step servoing
- Comprehensive docs: architecture, calibration, performance, troubleshooting

### Metrics (as of 2026-04-21)
- Detection accuracy: 98–100 % under lab lighting
- Pipeline throughput: 20–25 FPS
- IK unit test suite: 28 tests, 100 % passing
