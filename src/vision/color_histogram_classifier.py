"""
Color Histogram Ball Classifier — Inference
============================================

Lett inference-wrapper for den trente HSV-histogram + SVM-modellen.
Krever kun opencv-python, numpy og scikit-learn (ingen TensorFlow).

Eksempel:
    clf = ColorHistogramClassifier("models/ball_color_classifier.pkl")
    color, conf = clf.predict(ball_roi_bgr)    # → ("red", 0.97)

Author: Bachelor Project 2026 - Autonomia
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import joblib
    JOBLIB_AVAILABLE = True
except ImportError:
    JOBLIB_AVAILABLE = False

# ─── Feature extraction (duplisert fra train_color_classifier.py) ────────────
# Holdt separat for å unngå å importere hele train-scriptet ved inferens.

H_BINS = 36
S_BINS = 32
V_BINS = 32
TOTAL_FEATURES = H_BINS + S_BINS + V_BINS  # 100


def _extract_hsv_histogram(img_bgr: np.ndarray) -> np.ndarray:
    """Trekker ut normalisert HSV-histogram. Se train_color_classifier.py."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h_c, s_c, v_c = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    mask = (s_c >= 40) & (v_c >= 30) & (v_c <= 250)
    if mask.sum() < 30:
        mask = np.ones_like(mask, dtype=bool)

    def _hist(channel, bins, lo, hi):
        arr = np.histogram(channel[mask], bins=bins, range=(lo, hi))[0].astype(np.float32)
        s = arr.sum()
        return arr / s if s > 0 else arr

    return np.concatenate([
        _hist(h_c, H_BINS, 0, 180),
        _hist(s_c, S_BINS, 0, 256),
        _hist(v_c, V_BINS, 0, 256),
    ])


# ─── Classifier ─────────────────────────────────────────────────────────────

class ColorHistogramClassifier:
    """
    Enkel ballfargetklassifiserer basert på HSV-histogrammer og SVM.

    Fordeler vs MobileNetV2-tilnærming:
    - 32 KB modell (vs ~10 MB .h5)
    - < 1 ms inferenstid på Raspberry Pi
    - Fungerer uten TensorFlow
    - Trenes effektivt med få bilder (50–200)
    - 95%+ nøyaktighet på webcam-croppede bilder
    """

    def __init__(self, model_path: str = "models/ball_color_classifier.pkl"):
        """
        Laster trent SVM-modell.

        Args:
            model_path: Sti til .pkl-fil fra train_color_classifier.py
        """
        if not JOBLIB_AVAILABLE:
            raise ImportError("joblib ikke installert. Kjør: pip install scikit-learn joblib")

        path = Path(model_path)
        if not path.exists():
            raise FileNotFoundError(
                f"Modell ikke funnet: {model_path}\n"
                f"Tren med: python src/vision/train_color_classifier.py"
            )

        data = joblib.load(str(path))
        self._pipeline = data["pipeline"]
        self._class_names: list[str] = data["class_names"]

    @property
    def class_names(self) -> list[str]:
        return self._class_names

    def predict(self, ball_roi_bgr: np.ndarray) -> Tuple[str, float]:
        """
        Klassifiserer en ball-ROI.

        Args:
            ball_roi_bgr: Cropped ball-bilde i BGR-format (vilkårlig størrelse)

        Returns:
            (farge_navn, konfidensverdi)  — f.eks. ("red", 0.97)
            Returnerer ("unknown", 0.0) ved feil.
        """
        try:
            features = _extract_hsv_histogram(ball_roi_bgr).reshape(1, -1)
            proba = self._pipeline.predict_proba(features)[0]
            label_idx = int(np.argmax(proba))
            return self._class_names[label_idx], float(proba[label_idx])
        except Exception:
            return "unknown", 0.0

    def predict_batch(self, rois: list[np.ndarray]) -> list[Tuple[str, float]]:
        """Klassifiserer en liste med ball-ROI-er på én gang."""
        return [self.predict(roi) for roi in rois]
