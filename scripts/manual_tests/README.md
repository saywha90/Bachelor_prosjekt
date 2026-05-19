# Manual Test Scripts

These scripts require physical hardware (OAK-D camera, Dynamixel servos) or produce
GUI output. They are **not** part of the automated pytest suite.

Run them individually from the project root:

```bash
python scripts/manual_tests/backend_check.py
python scripts/manual_tests/enhanced_detector_demo.py
python scripts/manual_tests/ik_virtual_demo.py
python scripts/manual_tests/oak_v3_demo.py
python scripts/manual_tests/record_stats.py
```

| Script | Purpose |
|---|---|
| `backend_check.py` | Matplotlib visualisation of IK backend geometry |
| `enhanced_detector_demo.py` | Live OAK-D ball-detection pipeline demo |
| `ik_virtual_demo.py` | CLI table of IK solutions across a grid of target positions |
| `oak_v3_demo.py` | Minimal OAK-D camera connectivity check |
| `record_stats.py` | Record detection statistics session + save PNG/JSON report |
