"""
HSV-histogram + SVM Ball Color Classifier
==========================================

Enkel og effektiv klassifiserer som bruker fargehistogrammer til å skille
røde og blå baller. Fungerer utmerket med lite treningsdata (50-200 bilder)
og er rask nok for Raspberry Pi (< 1ms per klassifisering).

Teknisk tilnærming:
- Trekker ut HSV-histogrammer (32 bins per kanal = 96 features)
- Normaliserer histogrammer
- Trener SVM med RBF-kjerne + kryssvalidering

Bruk:
    python train_color_classifier.py --data_dir ./training_data

Lagrer:
    models/ball_color_classifier.pkl

Author: Bachelor Project 2026 - Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from sklearn.svm import SVC
    from sklearn.preprocessing import StandardScaler
    from sklearn.model_selection import StratifiedKFold, cross_val_score
    from sklearn.pipeline import Pipeline
    from sklearn.metrics import classification_report, confusion_matrix
    import joblib
except ImportError:
    print("FEIL: scikit-learn ikke installert. Kjør: pip install scikit-learn joblib")
    sys.exit(1)

# ─── Histogram feature extraction ──────────────────────────────────────────

H_BINS = 36   # 0–179 → 5 grad per bin
S_BINS = 32   # saturation
V_BINS = 32   # value/brightness

TOTAL_FEATURES = H_BINS + S_BINS + V_BINS   # 100 features totalt


def extract_hsv_histogram(img_bgr: np.ndarray) -> np.ndarray:
    """
    Trekker ut et normalisert HSV-histogram fra et ball-bilde.

    Piksler med lav metning (S<40) og meget lys/mørk (V<30 eller V>250)
    filtreres bort — dette fjerner svart/hvit bakgrunn.

    Args:
        img_bgr: Ball-bilde i BGR-format (typisk ~100x115 px)

    Returns:
        1-D feature-vektor med lengde TOTAL_FEATURES (100)
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h_chan, s_chan, v_chan = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    # Bakgrunnsmaske: behold bare fargede (metningsrike) piksler
    mask = (s_chan >= 40) & (v_chan >= 30) & (v_chan <= 250)

    # Bruk maske — fall tilbake til alle piksler hvis for lite igjen
    if mask.sum() < 30:
        mask = np.ones_like(mask, dtype=bool)

    h_hist = np.histogram(h_chan[mask], bins=H_BINS, range=(0, 180))[0].astype(np.float32)
    s_hist = np.histogram(s_chan[mask], bins=S_BINS, range=(0, 256))[0].astype(np.float32)
    v_hist = np.histogram(v_chan[mask], bins=V_BINS, range=(0, 256))[0].astype(np.float32)

    # L1-normaliser (summen = 1) for belysningsinvarians
    def _l1(arr):
        s = arr.sum()
        return arr / s if s > 0 else arr

    return np.concatenate([_l1(h_hist), _l1(s_hist), _l1(v_hist)])


# ─── Dataset loading ────────────────────────────────────────────────────────

def load_dataset(data_dir: str):
    """Laster alle bilder fra data_dir/<klasse>/ og trekker ut histogrammer."""
    data_path = Path(data_dir)
    class_dirs = sorted([d for d in data_path.iterdir() if d.is_dir()])

    if not class_dirs:
        raise ValueError(f"Ingen klasse-mapper funnet i {data_dir}")

    X, y, classes = [], [], []
    class_names = [d.name for d in class_dirs]

    for label_idx, class_dir in enumerate(class_dirs):
        files = list(class_dir.glob("*.jpg")) + list(class_dir.glob("*.png"))
        print(f"  {class_dir.name}: {len(files)} bilder")
        for f in files:
            img = cv2.imread(str(f))
            if img is None:
                print(f"    ADVARSEL: Kan ikke lese {f.name}")
                continue
            X.append(extract_hsv_histogram(img))
            y.append(label_idx)

    return np.array(X, dtype=np.float32), np.array(y, dtype=np.int32), class_names


# ─── Training ───────────────────────────────────────────────────────────────

def train(data_dir: str, output_path: str = "models/ball_color_classifier.pkl"):
    """Trener SVM-klassifiserer og lagrer modell + klassenames."""
    print("=" * 60)
    print("BALL COLOR CLASSIFIER — HSV Histogram + SVM")
    print("=" * 60)

    print(f"\n1. Laster bilder fra {data_dir}/ ...")
    X, y, class_names = load_dataset(data_dir)
    print(f"\n  Totalt: {len(X)} bilder, {len(class_names)} klasser: {class_names}")
    print(f"  Feature-lengde: {X.shape[1]}")

    # Pipeline: StandardScaler → SVM (RBF)
    pipeline = Pipeline([
        ("scaler", StandardScaler()),
        ("svm", SVC(
            kernel="rbf",
            C=10.0,
            gamma="scale",
            probability=True,
            class_weight="balanced",
            random_state=42,
        )),
    ])

    # StratifiedKFold kryssvalidering (5-fold)
    print("\n2. Kryssvalidering (5-fold) ...")
    skf = StratifiedKFold(n_splits=min(5, min(np.bincount(y))), shuffle=True, random_state=42)
    cv_scores = cross_val_score(pipeline, X, y, cv=skf, scoring="accuracy")
    print(f"\n  CV nøyaktighet: {cv_scores.mean()*100:.1f}% ± {cv_scores.std()*100:.1f}%")
    print(f"  Individuelle fold: {[f'{s*100:.1f}%' for s in cv_scores]}")

    # Tren på alt data (best modell)
    print("\n3. Trener på alle bilder ...")
    pipeline.fit(X, y)

    # Konfusjonsmatrise på treningsdata (overkill-sjekk)
    y_pred = pipeline.predict(X)
    print("\n4. Klassifiseringsrapport (treningsdata):")
    print(classification_report(y, y_pred, target_names=class_names))

    cm = confusion_matrix(y, y_pred)
    print("  Konfusjonsmatrise:")
    for i, row in enumerate(cm):
        print(f"    Faktisk {class_names[i]:4s}: {row}")

    # Lagre modell med klasse-navn
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump({"pipeline": pipeline, "class_names": class_names}, str(output))
    print(f"\n✓ Modell lagret: {output}")
    print(f"  Størrelse: {output.stat().st_size / 1024:.0f} KB")

    print("\n" + "=" * 60)
    print("FERDIG")
    print("=" * 60)
    print("\nBruk i detector.py:")
    print("  from vision.classifier import ColorHistogramClassifier")
    print("  clf = ColorHistogramClassifier('models/ball_color_classifier.pkl')")
    print("  color, confidence = clf.predict(ball_roi_bgr)")


# ─── Main ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tren HSV-histogram SVM ballklassifiserer")
    parser.add_argument("--data_dir",  default="./training_data",
                        help="Mappe med klasse-undermapper (default: ./training_data)")
    parser.add_argument("--output",    default="src/vision/models/ball_color_classifier.pkl",
                        help="Output-sti for pkl-modell (default: src/vision/models/)")
    args = parser.parse_args()

    train(args.data_dir, args.output)
