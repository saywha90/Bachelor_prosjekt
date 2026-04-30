# Vision Pipeline — Iteration History

> This log tracks incremental fixes to the vision detection pipeline during development.
> For project-level release history see [`CHANGELOG.md`](../CHANGELOG.md) at the project root.

# Vision System Fixes

This document summarizes the fixes applied to the vision pipeline during the calibration and integration session. Each section describes the problem, root cause, and what was changed.

---

## Fix 1: Camera Calibration Tool — Dot Position Mismatch

**File:** `src/vision/calibrate_camera.py`

**Problem:** When clicking to mark calibration points, the visual dots appeared at wrong positions. Clicking top-left worked, but top-right and others were displaced or out of frame.

**Root cause:** The camera captures at 640×400 but the display window is 1280×800. Mouse callbacks recorded click coordinates in display space (0–1280, 0–800) but dots were drawn on the native 640×400 frame — so anything beyond pixel (640, 400) fell out of bounds.

**Fix:** Mouse coordinates in `mouse_callback()` are now scaled from display space to native camera space before being stored:

```python
nx = int(x * native_w / DISPLAY_W)   # 1280 → 640
ny = int(y * native_h / DISPLAY_H)   # 800  → 400
```

Display size constants centralized in `DISPLAY_W, DISPLAY_H`.

---

## Fix 2: Phantom/Ghost Ball Detections After Pickup

**Files:** `src/vision/detector.py` (formerly `enhanced_detector.py`), `src/ik/vision_bridge.py`

**Problem:** After the arm picked up a ball and rescanned, the detector reported "phantom" balls — balls no longer physically present — causing the arm to sort invisible objects.

**Root cause:**

- Kalman tracker `max_disappeared` was 6 — tracks survived 6 frames without a match, predicting phantom positions.
- No tracker reset between scan rounds — stale Kalman state persisted across scans.

**Fix:**

- Reduced `max_disappeared` from `6` to `2` in `SimpleBallDetector.__init__()` (line ~371).
- Added `reset_tracker()` method to `SimpleBallDetector` that clears all active Kalman tracks.
- `scan_for_balls()` in `vision_bridge.py` now calls `reset_tracker()` before each scan round.

---

## Fix 3: Non-Spherical Object Detection (Cubes, Pens)

**Files:** `src/vision/detector.py` (formerly `enhanced_detector.py`), `src/config/vision.py` (formerly `src/vision/config.py`)

**Problem:** Red cubes were detected as "RED ball" at ~94% confidence. Red pens were also falsely detected.

**Root cause:** Shape validation filters were too permissive — circularity threshold was 0.60 (a square has ~0.785), aspect ratio threshold was 0.70 (a pen might be 0.3), and confidence threshold was only 0.35.

**Fixes applied:**

| Filter | Before | After | Why |
|--------|--------|-------|-----|
| `min_circularity` | 0.60 | 0.82 | Square ≈ 0.785, circle ≈ 0.90+. 0.82 rejects squares |
| Corner detection | none | Reject ≤ 6 vertices | Squares have 4 vertices via `cv2.approxPolyDP`, circles have 8+ |
| `min_aspect` | 0.70 | 0.80 | Rejects elongated objects like pens (aspect 0.2–0.5) |
| Hough `param2` | 35 | 40 | Requires stronger circular evidence |
| `BALL_CONFIDENCE_THRESHOLD` | 0.35 | 0.50 | Filters out low-confidence false positives |
| Confidence bonus normalization | adjusted | matches new thresholds | Scores calibrated to new base thresholds |

---

## Fix 4: Live Camera View Added

**Files:** `src/ik/vision_bridge.py` (formerly `src/IK/vision_bridge.py`), `src/main.py` (formerly `src/IK/main.py`)

**Problem:** No way to see what the camera was detecting during simulation runs.

**Fix:** Added `show_frame()` method to `VisionBridge` that draws detection overlays (circles, color labels, cm coordinates) and displays via `cv2.imshow`. Called during every scan and correction phase. Uses `cv2.waitKey(1)` to stay non-blocking.

---

## Fix 5: Continuous Operation Mode

**File:** `src/main.py` (formerly `src/IK/main.py`)

**Problem:** System shut down after 3 empty scan rounds — not suitable for an autonomous sorting arm on a moving car.

**Fix:** Main loop runs indefinitely (`while True`). When no balls are found, waits 3 seconds while keeping camera view alive, then rescans. Only exits on `Ctrl+C` with graceful shutdown (HOME → close camera → close visualizer).

---

## Fix 6: Debug HUD — Static Scan Counter

**File:** `src/ik/vision_bridge.py` (formerly `src/IK/vision_bridge.py`)

**Problem:** The Debug HUD always showed "Scan #0" regardless of how many sorting cycles had run.

**Root cause:** `self._total_scans` was initialized in `__init__` and displayed in `_draw_debug_hud`, but was never incremented anywhere in the logic.

**Fix:** Added `self._total_scans += 1` to the start of the `scan_for_balls()` method.

---

## Fix 7: Phantom Grab Prevention (Abort Logic)

**File:** `src/main.py` (formerly `src/IK/main.py`)

**Problem:** If an object was manually removed after being queued but before the arm reached it, the arm would "grab" thin air.

**Root cause:** The state machine processed a static queue from the initial scan without a final verification step.

**Fix:** Modified `run_sorting_cycle()` to check the return value of `vision.refine_detection()`. If the object was lost during the approach, the arm cancelled the cycle and returned to `HOME` safely.

> **Note (2026-04-26):** The two-step 80 %→100 % approach and `refine_detection()` have since been removed entirely — the arm now moves directly to the grab position in a single step because the wrist-mounted camera is occluded during approach (see ADR-003).

---

## Fix 8: Hough False Positives (Bubblewrap/Clutter)

**File:** `src/vision/detector.py` (formerly `enhanced_detector.py`)

**Problem:** Desaturated or pale red objects (bubble wrap, cables) were being detected as balls.

**Root cause:** The Hough color validator had low saturation (`90`) and fill (`25%`) thresholds.

**Fix:** Tightened the Hough saturation gate to `120` and the required color fill ratio to `50%`.

---

## Fix 9: Touching/Overlapping Object Detection

**File:** `src/vision/detector.py` (formerly `enhanced_detector.py`)

**Problem:** Same-colored objects (ball touching a cube) merged into a single non-circular blob, hiding the ball from the HSV detector.

**Root cause:** The Hough detector (which can distinguish shapes via edge detection) was effectively disabled.

**Fix:** Re-enabled Hough on every frame (`_hough_interval = 1`) and balanced sensitivity (`param2 = 42`) to allow for shadowed or slightly occluded edges.

---

## Fix 10: Polygon and Texture Rejection (Cube/Carton Filter)

**File:** `src/vision/detector.py` (formerly `enhanced_detector.py`)

**Problem:** Cubic objects (milk cartons, blocks) or complex textures (cow's ear) were still being accepted as balls by geometric edge detectors.

**Root cause:** The system lacked a vertex-count "thinking" step to distinguish circles from polygons, and it lacked a bounds-check to see if a detected "circle" was just a tiny patch of a massive object (like a wall or milk carton roof).

**Fix:** Added a two-step local shape-verification protocol inside the Hough logic:
1. **Polygon Approximation (`approxPolyDP`)**: Mathematically counts vertices on every candidate color mask.
   - **Reject:** Objects with ≤ 6 vertices (Squares, Rectangles, Cubes).
2. **Intersection over Union (IoU)**: Superimposes a mathematical "ideal circle" over the actual red mask found in the camera view. 
   - **Reject:** If the real red color spills massively outside the circle (IoU < 45%). This eliminates the tops of the milk cartons, graphics, and random textured cow ears, as their actual paint doesn't follow a spherical boundary.

---

## Fix 11: Red Mask Leakage (Desk Background Noise)

**File:** `src/vision/detector.py`

**Problem:** The red mask was "leaking" into the beige/gray wood grain of the desk, causing large, irregular blobs that merged balls together and created massive false-positive shapes.

**Root cause:** The HSV saturation threshold for red was set too low (minimum 35–50). The neutral desk color had enough red/orange hue components to be picked up as "red" at that low saturation level, especially under the adaptive CLAHE lighting boost.

**Fix:** Tightened the red saturation gates to match the strictness of the blue detector:
- **Regular Red:** Minimum saturation increased from `50` to **`120`**.
- **Dark/Shadow Red:** Minimum saturation increased from `35` to **`80`**.
- **Value (Brightness):** Increased minimum V from `10` to **`20`** to ignore deep shadows.

This forces the detector to ignore the desaturated background colors of the desk and only identify the vibrant, highly-saturated red balls.
