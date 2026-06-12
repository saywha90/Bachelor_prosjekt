"""
vision.py
=========
Shared configuration constants for the vision subsystem.

All vision scripts (enhanced_detector_demo, stream_debug, capture_training_data,
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

# ── Main runtime camera exposure ───────────────────────────────────────
# Used by main.py via VisionBridge.apply_main_manual_exposure().  Keep this
# lower than calibration exposure when sunlight/bright floor glare washes out
# blue balls in the live sorting loop.
MAIN_DETECTION_MANUAL_EXPOSURE_US = 2000000
MAIN_DETECTION_MANUAL_ISO = 800
MAIN_DETECTION_MANUAL_WB_K = 4500
MAIN_DETECTION_POST_APPLY_DISCARD_FRAMES = 8

# ── Calibration camera exposure ────────────────────────────────────────
# Touch calibration intentionally uses a dim fixed exposure for the scan.
# Bright OAK-D images on the black calibration mat can wash out saturated
# red/orange/cyan balls, which makes HSV segmentation merge glare with colour.
# Set CALIBRATION_USE_DIM_MANUAL_EXPOSURE = False to fall back to the older
# empty-desk AE/AWB settle + lock workflow.
CALIBRATION_USE_DIM_MANUAL_EXPOSURE = True

# Conservative fixed scan exposure for bright lab/OAK-D calibration lighting.
# Tune these three values together if the preview is still washed out or too
# dark.
CALIBRATION_DIM_MANUAL_EXPOSURE_US = 2000000
CALIBRATION_DIM_MANUAL_ISO = 800
CALIBRATION_DIM_MANUAL_WB_K = 4500
CALIBRATION_DIM_POST_APPLY_DISCARD_FRAMES = 8

# Fallback settling used if manual runtime controls are unavailable or disabled.
# This prevents a hand/forearm in the foreground from changing exposure or white
# balance during ball auto-detection, but is less robust against overexposure.
CALIBRATION_EMPTY_DESK_SETTLE_FRAMES = 45
CALIBRATION_POST_LOCK_DISCARD_FRAMES = 5

# Backward-compatible aliases for scripts that imported the previous manual
# fallback constants directly.
CALIBRATION_MANUAL_EXPOSURE_US = CALIBRATION_DIM_MANUAL_EXPOSURE_US
CALIBRATION_MANUAL_ISO = CALIBRATION_DIM_MANUAL_ISO
CALIBRATION_MANUAL_WB_K = CALIBRATION_DIM_MANUAL_WB_K

# ── Ball detection thresholds ────────────────────────────────────────
BALL_MIN_RADIUS = 10            # Minimum ball radius in pixels (at native res)
BALL_MAX_RADIUS = 150           # Maximum ball radius in pixels (at native res)
BALL_CONFIDENCE_THRESHOLD = 0.50  # Minimum ensemble confidence to accept (was 0.35)
