"""
config.py
=========
Shared configuration constants for the vision subsystem.

All vision scripts (test_enhanced_detector, stream_debug, capture_training_data,
etc.) import from this module.  Values are calibrated for the OAK Series 2
camera with IMX378 sensor.

Author: Bachelor Project 2026 – Autonomia
"""

# ── Camera settings ──────────────────────────────────────────────────
#   Processing resolution — lower than native 4K for real-time performance.
#   (640, 400) gives ~30 FPS through the full detection pipeline.
CAMERA_RESOLUTION = (640, 400)

#   Horizontal field-of-view for the IMX378 RGB sensor.
#   Used as fallback if EEPROM calibration is unavailable.
CAMERA_HFOV_DEG = 81.0

# ── Ball detection thresholds ────────────────────────────────────────
BALL_MIN_RADIUS = 10            # Minimum ball radius in pixels (at native res)
BALL_MAX_RADIUS = 150           # Maximum ball radius in pixels (at native res)
BALL_CONFIDENCE_THRESHOLD = 0.35  # Minimum ensemble confidence to accept
