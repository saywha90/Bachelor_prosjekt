# ADR-001: Classical HSV Vision over CNN-based Detection

## Status

Accepted

## Context

The system needs to detect red and blue balls on a workspace in real-time to guide a robotic arm for pick-and-place sorting. The detection must be fast enough for a continuous scan → pick → rescan loop (target: ≥ 20 FPS at 640 × 400 resolution), reliable enough for autonomous operation (target: ≥ 98% detection rate), and produce accurate centre coordinates for the homography-based coordinate transform that feeds the IK solver.

Two prior approaches were attempted and abandoned:

1. **CNN (MobileNetV2 transfer learning):** Produced 200–500 ms per frame on the Raspberry Pi, overfit on the limited training dataset, and required extensive labelled data for each new ball colour or lighting condition.
2. **Complex classical pipeline (Kalman + hand detection + motion segmentation):** Fragile, produced frequent false negatives, and was difficult to debug due to the number of interacting heuristics.

## Decision

Use an **ensemble of classical computer vision methods** as implemented in [`SimpleBallDetector`](../../src/vision/detector.py):

- **Multi-range HSV colour thresholding** (2 red ranges covering hue wraparound, 2 blue ranges) with morphological cleanup (open 3×3, close 13×13, flood-fill hole filling) and contour validation (circularity ≥ 0.82, aspect ratio ≥ 0.80, solidity ≥ 0.75, vertex count > 6)
- **Hough Circle Transform** for geometric validation, with IoU-based shape verification against an ideal circle (IoU ≥ 0.45) and polygon rejection (≤ 6 vertices → reject)
- **Union-Find ensemble merge** that clusters overlapping detections from both methods and boosts confidence by 8% for multi-method agreement
- **SVM colour verification** (secondary) using a histogram-feature classifier trained on captured ball images ([`ball_color_classifier.pkl`](../../src/vision/models/ball_color_classifier.pkl)) that corrects the colour label when ≥ 75% confident
- **Kalman-filter ball tracking** for stable IDs across frames (constant-velocity model, greedy cost-matrix matching by Euclidean distance + colour)

The pipeline also includes **adaptive lighting compensation**: lighting is classified as low / medium / high based on mean brightness, and CLAHE is applied to the LAB L-channel at low light. HSV ranges are widened or tightened dynamically based on the lighting level.

## Rationale

| Criterion | HSV + Hough + SVM Ensemble | CNN / Deep Learning |
|---|---|---|
| **Latency** | ~40–50 ms/frame (20–25 FPS) at 0.75× scale on Raspberry Pi | 200–500 ms/frame for MobileNetV2 on the same hardware |
| **Training data** | No training data required for HSV/Hough; small labelled set sufficient for the SVM colour verifier | Hundreds to thousands of labelled images needed per class |
| **Determinism** | Fully deterministic — same frame always produces the same result | Non-deterministic (floating-point rounding, batch normalisation) |
| **Explainability** | Each rejection has a specific reason (area too small, circularity too low, wrong hue range) visible in diagnostic tools | Black-box confidence score; difficult to diagnose why a detection was missed |
| **Calibration** | HSV ranges can be tuned interactively with live trackbars (`04_hsv_tuner.py`) in minutes | Requires re-training with new data, plus hyperparameter search |
| **Controlled environment** | Lab lighting is 300–700 lux; ball colours are known and distinct (red, blue); backgrounds are controlled | CNN excels in uncontrolled environments with many object classes — overkill here |

The ensemble approach combines the complementary strengths of HSV (colour-specific, fast) and Hough (geometry-specific, colour-agnostic). The SVM adds a learned second opinion that catches cases where HSV alone might misclassify due to lighting artefacts.

## Alternatives Considered

### YOLO / SSD Object Detection
- **Evaluated:** YOLOv5-nano was tested informally during early prototyping.
- **Rejected because:** Required GPU or significant inference time on Raspberry Pi (even with ONNX/OpenVINO optimisation); needed labelled training data; the problem of detecting 2 known-colour spheres in a controlled environment does not benefit from a general object detector.

### Pure HSV Without Ensemble
- **Evaluated:** HSV-only detection was the first working prototype.
- **Rejected because:** Susceptible to false positives from similarly-coloured background objects (e.g., red USB cables, blue tape). Adding Hough as a geometric cross-check and the SVM as a colour cross-check reduced false positive rate to < 2%.

### Depth-Based Detection (OAK-D Stereo)
- **Evaluated:** The OAK-D supports stereo depth, which could detect spherical objects independent of colour.
- **Rejected because:** The stereo depth at close range (10–30 cm) has low accuracy for 50 mm balls; colour sorting still requires colour classification; the RGB pipeline alone proved sufficient.

## Consequences

### Positive

- **20–25 FPS** on Raspberry Pi 5 — fast enough for continuous scanning with responsive live preview
- **98–100% detection rate** for both red and blue balls under lab conditions
- **< 2% false positive rate** thanks to multi-stage filtering (shape gates → ensemble → SVM → NMS → per-colour limit)
- **Full diagnostics:** `04_hsv_tuner.py`, `diagnose_detection.py`, and `stream_debug.py` provide complete visibility into every step of the detection pipeline
- **No GPU required** — the entire pipeline runs on CPU

### Negative

- **Requires manual HSV calibration** when lighting conditions change significantly — mitigated by the adaptive lighting system and the `04_hsv_tuner.py` interactive tool
- **Sensitive to lighting changes** outside the 300–700 lux range the system was calibrated for — the CLAHE compensation helps at low light, but extreme conditions (direct sunlight, very dark rooms) may require recalibration
- **Limited to known colours** — adding a new ball colour requires defining new HSV ranges and retraining the SVM classifier
- **Specific to spherical objects** — the circularity, aspect ratio, and Hough Circle gates are tuned for balls; sorting cubes or irregular objects would require reworking the validation pipeline
