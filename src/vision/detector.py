"""
Simple Ball Detector
====================

Ensemble-system for pålitelig deteksjon av røde og blå baller med
Luxonis OAK Series 2 kamera.

Deteksjonsrørledning:
  1. Multi-range HSV color detection (5 red ranges, 2 blue ranges)
  2. Hough Circle Transform (geometrisk validering, aktiveres hvert N-te frame)
  3. Ensemble voting (slår sammen begge metoder)
  4. Adaptiv lyskompensasjon (CLAHE ved lavt lys)
  5. SVM-fargeverifisering (sekundær — korrigerer feil fargelabel)
  6. Kalman-filter ball-tracker (stabile ID-er på tvers av frames)

HSV-ranges er kalibrert live for OAK IMX378-sensor.

Author: Bachelor Project 2026 - Autonomia
"""

import logging
import math
from collections import defaultdict

import cv2
import numpy as np
from typing import Any, List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)

DETECTOR_BUILD_ID = "blue-hough-fallback-per-color-nms-2026-05-26"

BLUE_HUE_MIN = 78
BLUE_HUE_MAX = 135

__all__ = ["SimpleBallDetector", "BallColor", "DetectedBall", "DETECTOR_BUILD_ID"]


class BallColor(Enum):
    """Enum for ballfarger."""
    RED = "red"
    BLUE = "blue"
    UNKNOWN = "unknown"


@dataclass
class DetectedBall:
    """Dataklasse for en detektert ball."""
    color: BallColor
    center: Tuple[int, int]
    radius: float
    confidence: float
    detection_method: str  # "hsv", "hough", "ensemble"
    distance_cm: Optional[float] = None
    shape_confidence: float = 0.0   # Form-score (sirkulæritet, areal, aspekt) 0-1
    color_confidence: float = 0.0   # Farge-score (metning i HSV) 0-1
    track_id: int = 0               # Persistent tracker-ID (> 0 = aktivt sporet)


class BallTracker:
    """
    Centroid-based multi-objekt tracker for stabile ball-ID-er på tvers av frames.

    Kobler nye deteksjoner til eksisterende objekter via euklidsk avstand og
    farge-matching. Objekter som ikke detekteres i mer enn max_disappeared frames
    de-registreres automatisk. ID-er tildeles sekvensielt fra 1 og gjenbrukes
    aldri innenfor én sesjon.

    Tidskompleksitet: O(n²) — tilstrekkelig for ≤ 6 samtidige baller.
    """

    def __init__(self, max_disappeared: int = 8, max_distance: float = 120.0) -> None:
        """
        Args:
            max_disappeared: Maks antall frames et objekt kan være borte
                             før det fjernes fra registeret.
            max_distance:    Maks pikselavstand for å matche to deteksjoner
                             som samme objekt mellom frames.
        """
        self._next_id: int = 1
        self._objects: Dict[int, DetectedBall] = {}
        self._disappeared: Dict[int, int] = {}
        self._kalman:    Dict[int, cv2.KalmanFilter] = {}  # Kalman-filter per tracked ball
        self._radius_sm: Dict[int, float] = {}             # EMA-gléttet radius
        self.max_disappeared = max_disappeared
        self.max_distance    = max_distance
        self._ALPHA_RADIUS   = 0.15  # Kalman-filter tar seg av posisjon; radius EMA-glattet

    def _make_kalman(self, cx: float, cy: float) -> cv2.KalmanFilter:
        """Lag et nytt Kalman-filter initialisert ved posisjon (cx, cy).

        Tilstandsvektor [x, y, vx, vy] — konstant-hastighetsmodell.
        Mål [x, y] — moments-tyngdepunkt fra _validate_contour.

        Støyparametre (ved halvskala 320×200):
          Q_pos = 1.0   → posisjon kan avvike ~1px/frame fra modell
          Q_vel = 9.0   → hastighet kan endre seg ~3px/frame²
          R     = 25.0  → moments-sentroid ±5px jitter → varianse 25
        Disse gir Kalmangain K ≈ 0.17 for stasjonær ball:
        utgangsposisjon ≈ 17% måling + 83% prediksjon → ≈ ±0.85px støy.
        """
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.float32([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        kf.measurementMatrix = np.float32([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])
        # Q_pos=1.0 → posisjon kan avvike ±1px/frame fra modell (var 0.02 → for tregt)
        # Q_vel=4.0 → hastighet kan endre ±2px/frame² (dekker brå stopp/start)
        # R=25.0  → measurements-SD ≈ ±5px for moments-sentroid (var 50 → ignorerte målinger)
        kf.processNoiseCov    = np.diag([1.0, 1.0, 4.0, 4.0]).astype(np.float32)
        kf.measurementNoiseCov = np.diag([25.0, 25.0]).astype(np.float32)
        kf.errorCovPost        = np.eye(4, dtype=np.float32) * 100.0
        kf.statePost           = np.float32([[cx], [cy], [0.0], [0.0]])
        return kf

    def update(self, detections: List[DetectedBall]) -> Dict[int, DetectedBall]:
        """
        Oppdater tracker med deteksjoner fra aktuell frame.

        Per frame:
          1. predict()  — fremskriver alle Kalman-filtre én tidssteg
          2. Grådig kostnadsmatching mot PREDIKERTE (ikke lagrede) posisjoner
          3. correct()  — korrigerer matchede filtre med faktisk måling
          4. Umatchede tracks: commit prediksjon som ny tilstand ("coasting")

        Returns:
            Dict som mapper stabil track_id → DetectedBall for alle aktive objekter.
        """
        # ── Steg 1: Forutsi alle eksisterende objekters neste posisjon ────
        predicted: Dict[int, Tuple[float, float]] = {}
        for oid, kf in self._kalman.items():
            sp = kf.predict()
            predicted[oid] = (float(sp[0, 0]), float(sp[1, 0]))

        # ── Ingen deteksjoner: fremskriv via prediksjon, øk forsvunnet ────
        if not detections:
            for oid in list(self._disappeared):
                self._disappeared[oid] += 1
                if self._disappeared[oid] > self.max_disappeared:
                    self._objects.pop(oid, None)
                    self._disappeared.pop(oid, None)
                    self._kalman.pop(oid, None)
                    self._radius_sm.pop(oid, None)
                else:
                    self._coast_track(oid, predicted)
            return dict(self._objects)

        # ── Ingen eksisterende objekter: registrer alle nye direkte ───────
        if not self._objects:
            for det in detections:
                kf = self._make_kalman(float(det.center[0]), float(det.center[1]))
                det.track_id                     = self._next_id
                self._objects[self._next_id]     = det
                self._disappeared[self._next_id] = 0
                self._kalman[self._next_id]      = kf
                self._radius_sm[self._next_id]   = det.radius
                self._next_id += 1
            return dict(self._objects)

        # ── Bygg kostnadsmatrise mot PREDIKERTE posisjoner ────────────────
        existing_ids = list(self._objects.keys())
        n_e, n_d     = len(existing_ids), len(detections)

        cost = np.full((n_e, n_d), fill_value=1e9, dtype=np.float32)
        for i, oid in enumerate(existing_ids):
            if oid not in predicted:
                continue
            px, py    = predicted[oid]
            ref_color = self._objects[oid].color
            for j, nb in enumerate(detections):
                if ref_color != nb.color:
                    continue
                dx = px - nb.center[0]
                dy = py - nb.center[1]
                cost[i, j] = math.hypot(dx, dy)

        # ── Grådig matching (laveste kostnad vinner) ─────────────────────
        pairs = sorted(
            [(float(cost[i, j]), i, j) for i in range(n_e) for j in range(n_d)],
            key=lambda t: t[0],
        )
        matched_e: set = set()
        matched_d: set = set()
        matches: List[Tuple[int, int]] = []
        for c, i, j in pairs:
            if i in matched_e or j in matched_d:
                continue
            if c >= self.max_distance:
                break
            matches.append((i, j))
            matched_e.add(i)
            matched_d.add(j)

        # ── Kalman correct() for matchede — produserer glatt statePost ────
        for i, j in matches:
            oid = existing_ids[i]
            det = detections[j]
            kf  = self._kalman[oid]

            measurement = np.float32([[det.center[0]], [det.center[1]]])
            kf.correct(measurement)

            det.center = (
                int(round(float(kf.statePost[0, 0]))),
                int(round(float(kf.statePost[1, 0]))),
            )
            prev_r               = self._radius_sm.get(oid, det.radius)
            self._radius_sm[oid] = prev_r + self._ALPHA_RADIUS * (det.radius - prev_r)
            det.radius           = self._radius_sm[oid]

            det.track_id          = oid
            self._objects[oid]    = det
            self._disappeared[oid] = 0

        # ── Umatchede eksisterende: fremskriv via prediksjon ("coasting") ─
        for i, oid in enumerate(existing_ids):
            if i not in matched_e:
                self._disappeared[oid] += 1
                if self._disappeared[oid] > self.max_disappeared:
                    self._objects.pop(oid, None)
                    self._disappeared.pop(oid, None)
                    self._kalman.pop(oid, None)
                    self._radius_sm.pop(oid, None)
                else:
                    self._coast_track(oid, predicted)

        # ── Registrer nye, umatchede deteksjoner ─────────────────────────
        for j in range(n_d):
            if j not in matched_d:
                det = detections[j]
                kf  = self._make_kalman(float(det.center[0]), float(det.center[1]))
                det.track_id                     = self._next_id
                self._objects[self._next_id]     = det
                self._disappeared[self._next_id] = 0
                self._kalman[self._next_id]      = kf
                self._radius_sm[self._next_id]   = det.radius
                self._next_id += 1

        return dict(self._objects)

    def _coast_track(self, oid: int, predicted: Dict[int, Tuple[float, float]]) -> None:
        """Advance a track with no detection (Kalman predict-only).

        Commits the Kalman prediction as the new state and updates the
        tracked object's center to the predicted position.
        """
        kf = self._kalman.get(oid)
        if kf is not None:
            kf.statePost    = kf.statePre.copy()
            kf.errorCovPost = kf.errorCovPre.copy()
        if oid in predicted and oid in self._objects:
            px, py = predicted[oid]
            self._objects[oid].center = (int(round(px)), int(round(py)))

    def reset(self) -> None:
        """Nullstill all tracking-tilstand og ID-teller."""
        self._next_id = 1
        self._objects.clear()
        self._disappeared.clear()
        self._kalman.clear()
        self._radius_sm.clear()


class SimpleBallDetector:
    """
    Ball detector med ensemble approach og adaptiv lyshåndtering.

    Rørledning per frame:
      1. Lysanalyse — klassifiser som low / medium / high
      2. CLAHE-kompensasjon ved lavt lys
      3. Multi-range HSV deteksjon (5 red, 2 blue ranges)
      4. Hough Circle Transform (geometrisk validering, cachet)
      5. Ensemble merge + NMS
      6. SVM fargeverifisering (sekundær)
      7. Persistent ball tracking (stabile ID-er)
      8. Returner maks N baller per farge
    """
    
    # Kjent balldiameter i mm (brukes til avstandsberegning)
    BALL_DIAMETER_MM = 50.0

    # Processing scale for detect_balls() — kernel sizes in __init__ are tuned for this value
    DETECTION_SCALE = 0.75

    # ─── Initialization ──────────────────────────────────────────────────

    def __init__(self,
                 min_radius: int = 10,
                 max_radius: int = 150,
                 confidence_threshold: float = 0.50,
                 enable_adaptive_lighting: bool = True,
                 focal_length_px: Optional[float] = None,
                 max_balls_per_color: int = 1):
        """
        Initialiserer forenklet detector.
        
        Args:
            min_radius: Minimum ball radius i piksler
            max_radius: Maximum ball radius i piksler
            confidence_threshold: Minimum confidence for å godkjenne deteksjon
            enable_adaptive_lighting: Aktiver adaptiv lyshåndtering for 300-700 lux
            focal_length_px: Kameraets brennvidde i piksler (kalibreres automatisk
                             første gang en ball detekteres på kjent avstand, eller
                             settes manuelt). Typisk 800-1200 px for webcam 1280x720.
            max_balls_per_color: Maks antall baller per farge å returnere per frame.
                                 Sett til 1 når du bare har én rød og én blå ball.
        """
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.max_balls_per_color = max_balls_per_color
        self.confidence_threshold = confidence_threshold
        self.enable_adaptive_lighting = enable_adaptive_lighting
        # Brennvidde: f = (radius_px * 2 * known_dist_mm) / BALL_DIAMETER_MM
        # Standard estimat for 1280x720 webcam uten kalibrering: ~900 px
        self.focal_length_px = focal_length_px if focal_length_px is not None else 900.0
        
        # Multi-range HSV thresholds for RED
        # ✅ KALIBRERT live med diagnose_detection.py — egne målinger på faktiske baller
        # Målte piksler: H=178-179, S=146-255, V=171-255
        # Ballen er LYS og mettet — IKKE mørk som tidligere antatt.
        # S_min senket fra 80→50 for å fange matte røde baller under varierende lys.
        self.red_ranges = [
            # Rød høy side (H wraparound nær 180) — primær range (S lowered 120→60)
            (np.array([160,  60,  40]), np.array([179, 255, 255])),
            # Rød lav side (H wraparound fra 0) — extended H 10→20, S lowered 120→60
            (np.array([0,    60,  40]), np.array([20,  255, 255])),
            # Mørk rød (skygge / lavt lys) — lav-side hue, S raised 40→55 to reject grey ground
            (np.array([0,    55,  20]), np.array([20,  255, 255])),
            # Mørk rød (skygge / lavt lys) — høy-side hue, S raised 40→55 to reject grey ground
            (np.array([160,  55,  20]), np.array([179, 255, 255])),
            # Orange-shifted red under bright presentation lights — S raised 30→55 to reject grey ground
            (np.array([0,    55,  50]), np.array([25,  255, 255])),
        ]

        # Multi-range HSV thresholds for BLUE
        # Dekker både mørk marineblå og lysere/cyan-ish blå baller uten å gjøre
        # rød/bakgrunn mer permissiv: de lavere S-gulvene brukes kun i blå-masken,
        # og _validate_contour() krever fortsatt blå hue-dominans for lav-S konturer.
        # Typiske målinger:
        #   Low-light cyan:   H≈78-100,  S≈45-255, V≈25-145
        #   Lys/cyan-ish blå: H≈85-110,  S≈55-255, V≈60-255
        #   Standard blå:     H≈100-135, S≈90-255, V≈40-255
        #   Mørk navy:        H≈95-135,  S≈70-255, V≈20-190
        self.blue_ranges = [
            # Low-light cyan/teal blue under dim manual exposure.
            # V ceiling raised 145→255: bright cyan highlight pixels (H≈78-90,
            # V>145) were falling between range 1 (V≤145) and range 2 (H≥85),
            # leaving only a dim crescent that failed roundness gates.
            # S floor lowered 45→40 to catch slightly desaturated cyan body.
            (np.array([ 78,  40,  20]), np.array([100, 255, 255])),
            # Lys/cyan-ish blå — H floor lowered 85→78 to cover full cyan range
            # at moderate-to-high saturation.  Shape gates still reject fabric.
            (np.array([ 78,  50,  50]), np.array([115, 255, 255])),
            # Blå — primær range (standard/mettet blå)
            (np.array([100,  90,  40]), np.array([135, 255, 255])),
            # Mørk marineblå (navy) — lavere S og V, bredere H
            (np.array([ 95,  70,  20]), np.array([135, 255, 190])),
        ]
        
        # Morphological kernels for noise reduction.
        # Closing-kernel er 11x11 for å koble fragmenter i ball-masken —
        # ballen gir en flekkete HSV-maske pga. lys/skygge variasjon.
        self.morph_kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # 13×13 ved halvskala = 26×26 ved full oppløsning.
        # Lukker gap opptil ~6px ved halvskala (spekulaere highlights i ball-masken)
        # uten å absorbere bakgrunnsfarger som er > 6px fra ballkanten.
        self.morph_kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))

        # CLAHE opprettes én gang (ikke per frame) for ytelse
        self.clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

        # SVM color classifier — DISABLED (always returns input unchanged).
        # Loading is skipped to avoid unnecessary I/O; see _verify_with_svm().
        self._svm_classifier = None

        # Statistics
        self.stats = {
            'total_hsv_detections': 0,
            'total_hough_detections': 0,
            'ensemble_detections': 0,
            'lighting_level': 'unknown'
        }
        self._frame_counter  = 0
        self._tracker = BallTracker(
            max_disappeared=2, max_distance=100.0
        )
    
    # ─── Lighting analysis ─────────────────────────────────────────────

    def analyze_lighting(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Analyserer lysforholdene i bildet for adaptiv justering.
        Estimerer lux-nivå basert på gjennomsnittlig lysstyrke (300-700 lux range).
        
        Args:
            frame: Input frame i BGR
            
        Returns:
            Dictionary med lysanalyse
        """
        # Konverter til grayscale for lysanalyse
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Beregn lysstatistikk
        mean_brightness = np.mean(gray)
        std_brightness = np.std(gray)
        
        # Grov lux-estimering basert på mean brightness (ikke kalibrert for OAK).
        # Brukes kun til å klassifisere lysnivået som low/medium/high.
        estimated_lux = 300 + (mean_brightness - 80) * 4.0
        estimated_lux = np.clip(estimated_lux, 300, 700)
        
        # Klassifiser lysnivå
        if mean_brightness < 100:
            level = "low"  # 300-400 lux
            needs_boost = True
        elif mean_brightness < 140:
            level = "medium"  # 400-550 lux
            needs_boost = False
        else:
            level = "high"  # 550-700 lux
            needs_boost = False
        
        return {
            'mean_brightness': mean_brightness,
            'std_brightness': std_brightness,
            'estimated_lux': estimated_lux,
            'level': level,
            'needs_boost': needs_boost
        }
    
    def apply_lighting_compensation(self, frame: np.ndarray, lighting_info: Dict) -> np.ndarray:
        """
        Appliserer adaptiv lyskompensasjon basert på detekterte lysforhold.
        
        CLAHE kjøres ALLTID når adaptive lighting er aktivert — selv medium/high
        lys har ujevne skygger (vignetting, lokale skygger) som gjør at baller
        i hjørner/kanter forsvinner uten lokal kontrastforsterkning.
        """
        if not self.enable_adaptive_lighting:
            return frame
        
        # Alltid bruk CLAHE — ujevnt lys gir lokale mørke soner selv i medium/high
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a, b = cv2.split(lab)
        l_channel = self.clahe.apply(l_channel)
        enhanced = cv2.merge([l_channel, a, b])
        return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
    
    # ─── Detection pipeline ───────────────────────────────────────────

    def _apply_hsv_ranges(
        self,
        hsv: np.ndarray,
        ranges: List[Tuple[np.ndarray, np.ndarray]],
        color: BallColor,
    ) -> List[DetectedBall]:
        """
        Bygger kombinert HSV-maske fra alle ranges og returnerer validerte baller.

        Args:
            hsv: HSV frame
            ranges: Liste med (lower, upper) numpy-arrays
            color: Forventet ballfarge

        Returns:
            Liste med validerte DetectedBall
        """
        combined = self._build_color_mask(hsv, ranges, color)

        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN,  self.morph_kernel_small)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self.morph_kernel_large)

        # Fyll innvendige hull (spekulær highlight) via flood fill fra hjørne.
        # Guard: hopp over hvis (0,0) allerede er hvit — da ville fill spre seg
        # innom masken og bitwise_not ville dekke hele bildet med hvitt.
        if combined[0, 0] == 0:
            _flood   = combined.copy()
            _ff_mask = np.zeros((combined.shape[0] + 2, combined.shape[1] + 2), dtype=np.uint8)
            cv2.floodFill(_flood, _ff_mask, (0, 0), 255)
            combined = cv2.bitwise_or(combined, cv2.bitwise_not(_flood))

        # Temporal mask er fjernet — falsk-positiv-filtrering håndteres av
        # sirkulæritet/aspekt/soliditet/farge-gates.

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        balls = []
        for contour in contours:
            ball = self._validate_contour(contour, color, "hsv", hsv)
            if ball:
                balls.append(ball)
        return balls

    def _build_color_mask(
        self,
        hsv: np.ndarray,
        ranges: List[Tuple[np.ndarray, np.ndarray]],
        color: BallColor,
    ) -> np.ndarray:
        """Build a HSV mask for one colour, with seeded red/orange glare repair.

        Red/orange calibration balls often contain three different pixel classes
        in the same physical ball: saturated red seed pixels, orange/yellow-shifted
        body pixels caused by presentation lights, and low-saturation bright
        specular highlights.  A plain global threshold sees only disconnected red
        fragments, so the contour is unstable and temporal calibration later sees
        the ball in only a few frames.

        To avoid reintroducing background false positives, the permissive
        orange/glare pixels are accepted only when they are spatially connected to
        the strict red seed mask.  Separate orange objects or bright desk patches
        without red support are still rejected before contour validation.
        """
        strict = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            strict = cv2.bitwise_or(strict, cv2.inRange(hsv, lower, upper))

        if color == BallColor.BLUE:
            return self._repair_blue_mask(hsv, strict)

        if color != BallColor.RED:
            return strict

        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        # First recover orange-shifted red body pixels.  H=26..35 is common on
        # red/orange balls under glare, but is not globally safe enough to add to
        # red_ranges.  Keep only components that touch the strict red seed.
        orange_hue = ((hue <= 35) | (hue >= 150))
        orange_body = np.zeros_like(strict)
        orange_body[(orange_hue & (sat >= 35) & (val >= 35))] = 255
        repaired = self._keep_components_touching_seed(orange_body, strict)

        # Some real calibration balls are orange-shifted across the entire
        # visible body in OAK frames, leaving little or no H<=25 / H>=160 strict
        # red seed.  The previous seed-only repair therefore saw only unstable
        # glare fragments, so temporal calibration reported low-support misses.
        # Promote standalone orange components only when the component itself is
        # ball-like; irregular orange desk patches still fail shape validation.
        standalone_orange = self._keep_ball_like_orange_components(orange_body, strict)
        repaired = cv2.bitwise_or(repaired, standalone_orange)

        # Then fill specular-highlight gaps, but only close to the already
        # red/orange-supported component.  This repairs split crescents without
        # letting bright table/background regions become standalone detections.
        support = cv2.dilate(repaired, self.morph_kernel_large, iterations=1)
        glare = np.zeros_like(strict)
        glare[((sat <= 90) & (val >= 165))] = 255
        repaired = cv2.bitwise_or(repaired, cv2.bitwise_and(glare, support))

        return repaired

    def _repair_blue_mask(self, hsv: np.ndarray, strict: np.ndarray) -> np.ndarray:
        """Recover shadowed blue/cyan ball pixels without accepting fabric blobs.

        The cyan ball can be geometrically round in the camera image while its
        lower half falls below the normal blue S/V floor under dim manual
        exposure.  If only the saturated upper crescent is kept, the later
        roundness gates correctly reject it as "not a ball".  Repair only pixels
        that are blue/cyan in hue and spatially close to an existing strict-blue
        seed, then let the normal contour shape gates decide whether the result
        is actually round.
        """
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        blue_hue = (hue >= BLUE_HUE_MIN) & (hue <= BLUE_HUE_MAX)

        # The low-light repair path must be permissive because the lower half of
        # the cyan ball can fall to S≈20-35 and V≈20-40.  In medium/high sunlight,
        # however, grey floor pixels and specular patches often have random blue
        # hue with S≈15-35; promoting those pixels makes huge false blue regions
        # that can out-rank the real ball.  Gate repair strength by scene value so
        # the permissive path is used only when the frame is actually dim.
        mean_val = float(np.mean(val))
        dim_scene = mean_val < 75.0

        # Dim blue body/shadow: hue remains blue/cyan, but S/V can drop below
        # the regular HSV ranges.  Restrict it to a local dilation of strict
        # blue so black mat texture cannot become a standalone detection.
        # In normal/sunny scenes use a much tighter support and a higher S floor;
        # otherwise low-saturation floor/glare pixels around blue seed regions are
        # swallowed into the mask.
        local_support = cv2.dilate(
            strict,
            self.morph_kernel_large,
            iterations=3 if dim_scene else 1,
        )
        dim_body = np.zeros_like(strict)
        # S floor lowered 24→16 and V floor 18→12 so the dark underside of a
        # cyan ball on a black mat is included (it still needs strict-seed
        # proximity via local_support, so mat texture is not promoted).
        if dim_scene:
            dim_body[(blue_hue & (sat >= 16) & (val >= 12))] = 255
        else:
            dim_body[(blue_hue & (sat >= 32) & (val >= 20))] = 255
        repaired = cv2.bitwise_or(strict, cv2.bitwise_and(dim_body, local_support))

        # Blue balls also show cyan/white specular highlights.  Add only highlight
        # pixels inside the repaired local support so holes do not split/fold the
        # contour, while unrelated bright tape/desk areas remain excluded.
        highlight_support = cv2.dilate(repaired, self.morph_kernel_large, iterations=1)
        highlight = np.zeros_like(strict)
        if dim_scene:
            highlight[(blue_hue & (sat <= 60) & (val >= 90))] = 255
        else:
            highlight[(blue_hue & (sat <= 60) & (val >= 125))] = 255
        repaired = cv2.bitwise_or(repaired, cv2.bitwise_and(highlight, highlight_support))

        # Pure specular highlights (near-white glare spot at the top of the ball)
        # have undefined/random hue because S≈0.  The blue_hue gate above misses
        # them, leaving a hole that turns the contour into a crescent which then
        # fails circularity / circle_fill gates.  Recover any very-low-S high-V
        # pixel that is spatially adjacent to existing blue pixels, regardless of
        # hue.  The tight dilation radius prevents unrelated bright desk/wall
        # patches from being absorbed.
        specular_support = cv2.dilate(
            repaired,
            self.morph_kernel_large,
            iterations=2 if dim_scene else 1,
        )
        specular = np.zeros_like(strict)
        if dim_scene:
            specular[((sat <= 45) & (val >= 100))] = 255
        else:
            # Sunny frames contain large low-S/high-V floor and paper regions.
            # Recover only true bright specular holes close to existing blue.
            specular[((sat <= 35) & (val >= 170))] = 255
        repaired = cv2.bitwise_or(repaired, cv2.bitwise_and(specular, specular_support))

        # Bridge specular-highlight holes and shadow crescents into a more
        # filled disc before the standard MORPH_CLOSE in _apply_hsv_ranges.
        # 21×21 at the 0.75× detection scale closes gaps up to ~10 px,
        # which covers typical cyan-ball specular holes and dim-shadow arcs.
        _bridge = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (21, 21) if dim_scene else (11, 11),
        )
        repaired = cv2.morphologyEx(repaired, cv2.MORPH_CLOSE, _bridge)

        return repaired

    def _keep_ball_like_orange_components(self, mask: np.ndarray, seed: np.ndarray) -> np.ndarray:
        """Keep standalone orange/red components that are already ball-shaped.

        Seeded repair remains permissive because strict red support proves the
        component belongs to a red ball.  Unseeded orange components must pass a
        local circle/solidity/extent gate before they are allowed into the mask;
        final contour validation then applies the normal confidence and colour
        gates.  This recovers glare-shifted orange balls without globally making
        orange table/background pixels valid red detections.
        """
        num_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels <= 1:
            return np.zeros_like(mask)

        kept = np.zeros_like(mask)
        min_area = math.pi * (self.min_radius ** 2)
        for label in range(1, num_labels):
            component_pixels = labels == label
            if np.any(seed[component_pixels] > 0):
                continue

            area_px = int(stats[label, cv2.CC_STAT_AREA])
            if area_px < min_area:
                continue

            comp = (component_pixels.astype(np.uint8) * 255)
            contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(contour)
            if contour_area < min_area:
                continue

            (enc_x, enc_y), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius or radius > self.max_radius:
                continue
            circle_area = math.pi * (radius ** 2)
            circle_fill = contour_area / circle_area if circle_area > 0 else 0.0
            if circle_fill < 0.74:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = (4 * np.pi * contour_area) / (perimeter ** 2)
            if circularity < 0.72:
                continue

            bx, by, bw, bh = cv2.boundingRect(contour)
            aspect_ratio = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0.0
            if aspect_ratio < 0.78:
                continue

            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = (contour_area / hull_area) if hull_area > 0 else 0.0
            if solidity < 0.82:
                continue

            ellipse_aspect = 1.0
            if len(contour) >= 5:
                try:
                    (_ecx, _ecy), (axis_a, axis_b), _angle = cv2.fitEllipse(contour)
                    major_axis = max(axis_a, axis_b)
                    minor_axis = min(axis_a, axis_b)
                    ellipse_aspect = minor_axis / major_axis if major_axis > 0 else 0.0
                except cv2.error:
                    ellipse_aspect = 0.0
            if ellipse_aspect < 0.82:
                continue

            extent = contour_area / float(bw * bh) if bw > 0 and bh > 0 else 0.0
            if extent < 0.60:
                continue

            # Reject polygonal orange objects; a filled circle keeps many more
            # vertices after approximation than an object edge or rectangular patch.
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            if len(approx) <= 7:
                continue

            kept[component_pixels] = 255

        return kept

    @staticmethod
    def _keep_components_touching_seed(mask: np.ndarray, seed: np.ndarray) -> np.ndarray:
        """Return connected components from mask that overlap seed pixels."""
        num_labels, labels = cv2.connectedComponents(mask, connectivity=8)
        if num_labels <= 1:
            return np.zeros_like(mask)

        touching_labels = np.unique(labels[seed > 0])
        touching_labels = touching_labels[touching_labels != 0]
        if touching_labels.size == 0:
            return np.zeros_like(mask)

        kept = np.isin(labels, touching_labels)
        return (kept.astype(np.uint8) * 255)

    def detect_with_hsv_multirange(
        self,
        hsv: np.ndarray,
        lighting_info: Optional[Dict] = None,
    ) -> Tuple[List[DetectedBall], List[DetectedBall]]:
        """
        Detekterer baller med multi-range HSV (med adaptiv justering).

        Args:
            hsv: Frame i HSV color space
            lighting_info: Optional lysanalyse for adaptiv justering

        Returns:
            Tuple med (red_balls, blue_balls)
        """
        red_balls  = self._apply_hsv_ranges(hsv, self.red_ranges,  BallColor.RED)
        blue_balls = self._apply_hsv_ranges(hsv, self.blue_ranges, BallColor.BLUE)

        self.stats['total_hsv_detections'] += len(red_balls) + len(blue_balls)
        return red_balls, blue_balls
    
    def detect_with_hough(self, gray: np.ndarray, hsv: np.ndarray) -> List[DetectedBall]:
        """
        Detekterer baller med Hough Circle Transform.
        Dette er geometrisk deteksjon - finner sirkulære objekter uavhengig av farge.
        
        Args:
            gray: Grayscale frame
            hsv: HSV frame (for fargebestemmelse)
            
        Returns:
            Liste med detekterte baller
        """
        # Hough Circle Transform
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1,
            # minDist: minimum avstand mellom senterene til detekterte sirkler.
            # max(80, w//10) ga 80px ved 320px bredde = 25% av bildet — alt for restriktivt.
            # 3× min_radius er tilstrekkelig for å gjøre to overlappende deteksjoner av
            # samme ball, men permissivt nok for to baller side om side.
            minDist=max(self.min_radius * 3, 20),
            param1=50,
            param2=35,   # 35: balanced — catches matte balls but rejects desk texture/shadows
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        detected_balls = []
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            for (x, y, radius) in circles:
                # Bestem farge ved å sjekke HSV-verdier i sentrum
                color = self._determine_color_from_hsv(hsv, (x, y), radius)

                if color == BallColor.UNKNOWN:
                    fallback_ball = self._recover_blue_from_hough_roi(hsv, (x, y), radius)
                    if fallback_ball is not None:
                        detected_balls.append(fallback_ball)
                        continue
                 
                # Kun godkjenn hvis fargen er identifisert som rød eller blå
                if color != BallColor.UNKNOWN:
                    # Beregn confidence basert på edge strength
                    roi = gray[max(0, y-radius):min(gray.shape[0], y+radius),
                              max(0, x-radius):min(gray.shape[1], x+radius)]
                    
                    if roi.size > 0:
                        # Hough er en geometrisk detektor — shape_confidence er definisjonsmessig høy.
                        # Beregn farge-score fra HSV-metning i senterregion, ekskluder gjennskinn.
                        shape_conf = 0.88   # Geometrisk validert sirkel

                        sr    = max(1, int(radius * 0.6))
                        roi_s = hsv[max(0, y-sr):min(hsv.shape[0], y+sr),
                                    max(0, x-sr):min(hsv.shape[1], x+sr), 1]
                        roi_v = hsv[max(0, y-sr):min(hsv.shape[0], y+sr),
                                    max(0, x-sr):min(hsv.shape[1], x+sr), 2]
                        if roi_s.size > 0:
                            not_glare = ~((roi_s < 30) & (roi_v > 210))
                            color_conf = (float(np.mean(roi_s[not_glare])) / 255.0
                                          if np.any(not_glare) else
                                          float(np.mean(roi_s)) / 255.0)
                        else:
                            color_conf = 0.0

                        # ---- Lokal form-verifisering ----
                        # Hindrer Hough i å godkjenne firkanter/kuber ("se objekt, er det firkantet -> ignorer")
                        vr = int(radius * 1.5)
                        local_hsv = hsv[max(0, y-vr):min(hsv.shape[0], y+vr),
                                        max(0, x-vr):min(hsv.shape[1], x+vr)]
                        if local_hsv.size > 0:
                            ranges = self.red_ranges if color == BallColor.RED else self.blue_ranges
                            local_mask = self._build_color_mask(local_hsv, ranges, color)
                            
                            local_contours, _ = cv2.findContours(local_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            if local_contours:
                                largest = max(local_contours, key=cv2.contourArea)
                                local_area = cv2.contourArea(largest)
                                if local_area < math.pi * (self.min_radius ** 2):
                                    continue
                                (_lcx, _lcy), local_radius = cv2.minEnclosingCircle(largest)
                                if local_radius < self.min_radius or local_radius > self.max_radius:
                                    continue
                                perimeter = cv2.arcLength(largest, True)
                                if perimeter > 0:
                                    local_circularity = (4 * np.pi * local_area) / (perimeter ** 2)
                                    if local_circularity < 0.70:
                                        continue

                                    local_circle_area = math.pi * (local_radius ** 2)
                                    local_circle_fill = local_area / local_circle_area if local_circle_area > 0 else 0.0
                                    if local_circle_fill < 0.70:
                                        continue

                                    lx, ly, lw, lh = cv2.boundingRect(largest)
                                    local_aspect = min(lw, lh) / max(lw, lh) if max(lw, lh) > 0 else 0.0
                                    if local_aspect < 0.78:
                                        continue

                                    local_hull = cv2.convexHull(largest)
                                    local_hull_area = cv2.contourArea(local_hull)
                                    local_solidity = (local_area / local_hull_area) if local_hull_area > 0 else 0.0
                                    if local_solidity < 0.80:
                                        continue

                                    local_ellipse_aspect = 1.0
                                    if len(largest) >= 5:
                                        try:
                                            (_ex, _ey), (axis_a, axis_b), _ang = cv2.fitEllipse(largest)
                                            major_axis = max(axis_a, axis_b)
                                            minor_axis = min(axis_a, axis_b)
                                            local_ellipse_aspect = minor_axis / major_axis if major_axis > 0 else 0.0
                                        except cv2.error:
                                            local_ellipse_aspect = 0.0
                                    if local_ellipse_aspect < 0.80:
                                        continue

                                    epsilon = 0.02 * perimeter
                                    approx = cv2.approxPolyDP(largest, epsilon, True)
                                    # En ren kube, hjørne, eller melkekartong vil ha <= 6 hjørner.
                                    # En sirkel (ball) har typisk 8-16 hjørner etter poly-approx.
                                    if len(approx) <= 6:
                                        continue  # Avvis den direkte

                            # 2. IoU sjekk mot ideell sirkel for å avvise tekstur/svære objekter
                            ideal_circle = np.zeros_like(local_mask)
                            cy_loc = y - max(0, y-vr)
                            cx_loc = x - max(0, x-vr)
                            cv2.circle(ideal_circle, (cx_loc, cy_loc), int(radius), 255, -1)
                            
                            intersection = cv2.bitwise_and(local_mask, ideal_circle)
                            union = cv2.bitwise_or(local_mask, ideal_circle)
                            
                            area_i = np.count_nonzero(intersection)
                            area_u = np.count_nonzero(union)
                            
                            iou = area_i / area_u if area_u > 0 else 0
                            # Krev at fargen ligner en sirkel. IoU=1 for perfekt ball, ~0.6 for okkludert ball.
                            # Melkekartongtak eller kuører spenner vilt og gir IoU < 0.4.
                            if iou < 0.45:
                                continue  # Avvis tekstur

                        # Konsistent formel med _validate_contour: 80% form + 20% farge + sqrt
                        raw        = (shape_conf * 0.80) + (color_conf * 0.20)
                        confidence = float(np.sqrt(raw))

                        if confidence > self.confidence_threshold:
                            ball = DetectedBall(
                                color=color,
                                center=(x, y),
                                radius=float(radius),
                                confidence=confidence,
                                detection_method="hough",
                                shape_confidence=float(shape_conf),
                                color_confidence=float(color_conf),
                            )
                            detected_balls.append(ball)
        
        self.stats['total_hough_detections'] += len(detected_balls)
        return detected_balls
    
    # ─── Internal helpers ─────────────────────────────────────────────

    def _determine_color_from_hsv(self, hsv: np.ndarray, center: Tuple[int, int], radius: int) -> BallColor:
        """
        Bestemmer ballens farge ved å sample HSV-verdier.
        
        Args:
            hsv: HSV frame
            center: Ball center (x, y)
            radius: Ball radius
            
        Returns:
            BallColor enum
        """
        x, y = center
        
        # Sample region rundt sentrum (60% av radius)
        sample_radius = int(radius * 0.6)
        roi = hsv[max(0, y-sample_radius):min(hsv.shape[0], y+sample_radius),
                  max(0, x-sample_radius):min(hsv.shape[1], x+sample_radius)]
        
        if roi.size == 0:
            return BallColor.UNKNOWN
        
        # Klassifiser ved piksel-telling per hue-range (håndterer rød wraparound korrekt).
        # mean_hue fungerer IKKE for rød: piksler ved hue=2 og hue=178 gir mean≈90 → feil BLÅTT.
        hue_ch = roi[:, :, 0]
        sat_ch = roi[:, :, 1]
        val_ch = roi[:, :, 2]
        
        # sat/val-sjekk for å avvise feil fra Hough (f.eks. rød kube eller bobleplast).
        # S≥8 og V≥8 tillater matte farger under varierende lys.
        valid_mask = (sat_ch >= 8) & (val_ch >= 8)
        valid_pixels = int(np.sum(valid_mask))

        # Krev at minst 15% av ROI-pikslene er klare farger.
        # Matte røde baller med store spekulære highlights kan ha lav metning.
        if valid_pixels < roi.shape[0] * roi.shape[1] * 0.15:
            return BallColor.UNKNOWN
        
        # Rød/orange: Hue 0-35 ELLER 145-179 (wraparound).  The 26-35 extension
        # matches the seeded red/orange repair mask used for glare-heavy balls.
        red_mask = valid_mask & ((hue_ch <= 35) | (hue_ch >= 145))
        red_pixels = int(np.sum(red_mask))
        
        # Blå/cyan: Hue 78-135 — konsistent med blå HSV-maskene, including
        # cyan-shifted balls under dim manual exposure.
        blue_mask = valid_mask & (hue_ch >= BLUE_HUE_MIN) & (hue_ch <= BLUE_HUE_MAX)
        blue_pixels = int(np.sum(blue_mask))
        
        # Bestemmelse: klar majoritet av gyldige piksler (30%).
        # Matte røde baller med spekulære highlights trenger lavere bar.
        if red_pixels > blue_pixels and red_pixels >= valid_pixels * 0.30:
            return BallColor.RED
        if blue_pixels > red_pixels and blue_pixels >= valid_pixels * 0.30:
            return BallColor.BLUE
        
        return BallColor.UNKNOWN

    def _recover_blue_from_hough_roi(
        self,
        hsv: np.ndarray,
        center: Tuple[int, int],
        radius: int,
    ) -> Optional[DetectedBall]:
        """Recover a circular blue ball when the HSV contour mask is too sparse.

        The live scene can show a clearly round blue ball on a dark carpet while
        the HSV contour path still rejects it because highlights/shadows fragment
        the mask.  Hough supplies reliable circle geometry; this fallback accepts
        it only when the circular ROI is clearly blue-dominant.
        """
        x, y = center
        h, w = hsv.shape[:2]
        x0, x1 = max(0, x - radius), min(w, x + radius + 1)
        y0, y1 = max(0, y - radius), min(h, y + radius + 1)
        if x0 >= x1 or y0 >= y1:
            return None

        yy, xx = np.ogrid[y0:y1, x0:x1]
        circle = ((xx - x) ** 2 + (yy - y) ** 2) <= (radius * 0.85) ** 2
        if not np.any(circle):
            return None

        roi_hue = hsv[y0:y1, x0:x1, 0][circle]
        roi_sat = hsv[y0:y1, x0:x1, 1][circle]
        roi_val = hsv[y0:y1, x0:x1, 2][circle]
        valid = (roi_sat >= 8) & (roi_val >= 8)
        if int(np.sum(valid)) < 20:
            return None

        hue_v = roi_hue[valid]
        sat_v = roi_sat[valid]
        val_v = roi_val[valid]
        blue_hue = (hue_v >= BLUE_HUE_MIN) & (hue_v <= BLUE_HUE_MAX)
        red_hue = (hue_v <= 35) | (hue_v >= 145)
        blue_seed = blue_hue & (sat_v >= 35) & (val_v >= 20)

        blue_ratio = float(np.mean(blue_hue))
        red_ratio = float(np.mean(red_hue))
        seed_ratio = float(np.mean(blue_seed))
        not_glare = ~((sat_v < 45) & (val_v > 100))
        sat_score = (float(np.mean(sat_v[not_glare])) / 255.0
                     if np.any(not_glare) else float(np.mean(sat_v)) / 255.0)

        if blue_ratio < 0.45:
            return None
        if blue_ratio <= red_ratio + 0.20:
            return None
        if seed_ratio < 0.08 and sat_score < 0.25:
            return None

        shape_conf = 0.86
        color_conf = float(np.clip(max(sat_score, blue_ratio * 0.65), 0.0, 1.0))
        confidence = float(np.sqrt((shape_conf * 0.80) + (color_conf * 0.20)))
        if confidence <= self.confidence_threshold:
            return None

        distance_cm = (self.focal_length_px * self.BALL_DIAMETER_MM) / (radius * 2 * 10.0)
        return DetectedBall(
            color=BallColor.BLUE,
            center=(int(x), int(y)),
            radius=float(radius),
            confidence=confidence,
            detection_method="hough_blue_fallback",
            distance_cm=round(distance_cm, 1),
            shape_confidence=shape_conf,
            color_confidence=color_conf,
        )
    
    def _validate_contour(self, contour: np.ndarray, color: BallColor, method: str,
                           hsv: Optional[np.ndarray] = None) -> Optional[DetectedBall]:
        """
        Validerer en contour og returnerer DetectedBall hvis valid.
        Krever at formen er tilnærmet sirkulær (ball-form).
        
        Args:
            contour: OpenCV contour
            color: Forventet farge
            method: Deteksjonsmetode
            
        Returns:
            DetectedBall hvis valid, None ellers
        """
        area = cv2.contourArea(contour)
        
        # Area check - må være minst som en liten ball
        if area < np.pi * (self.min_radius ** 2):
            return None
        
        # Minimum enclosing circle — brukes KUN for radius
        (enc_x, enc_y), radius = cv2.minEnclosingCircle(contour)

        # Area-tyngdepunkt som senter: gjennomsnitt av ALLE piksler i konturen.
        # Langt mer stabilt enn minEnclosingCircle-senter, som hopper kraftig
        # hvis noen få kant-piksler endrer seg (skygge, refleks) mellom frames.
        M = cv2.moments(contour)
        if M['m00'] > 1.0:
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
        else:
            cx, cy = enc_x, enc_y
        center = (int(round(cx)), int(round(cy)))

        # Radius check
        if radius < self.min_radius or radius > self.max_radius:
            return None
        
        # Circularity check (4πA / P²) - perfekt sirkel = 1.0
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return None
        circularity = (4 * np.pi * area) / (perimeter ** 2)

        bx, by, bw, bh = cv2.boundingRect(contour)
        
        # Dynamiske form-krav: Hvis ballen er klippet av kanten, tillater vi en D-form
        # 0.65 (ned fra 0.72) for å tolerere perspektivforvrengning og spekulær highlight
        # som gjør konturen litt ujevn — spesielt på matte røde baller.
        min_circularity = 0.65
        min_aspect = 0.75
        min_vertices = 6
        edge_clipped = False
        
        img_h, img_w = hsv.shape[:2] if hsv is not None else (0, 0)
        if img_h > 0 and img_w > 0:
            if bx <= 2 or by <= 2 or (bx + bw) >= (img_w - 2) or (by + bh) >= (img_h - 2):
                # Ballen berører kanten av bildet og er fysisk klippet.
                # En halvsirkel (D-form) har circularity ≈ 0.59, så vi tillater ned til 0.55.
                # Falske positiver fra kabler ved kanten håndteres av den
                # edge-spesifikke fargegaten lenger ned (sat_score-sjekk).
                edge_clipped = True
                min_circularity = 0.55
                min_aspect = 0.65
                min_vertices = 4

        # Blue balls under specular or dim lighting appear as arcs/crescents
        # in the HSV mask because only the lit half (or shadowed half) falls
        # within the strict HSV ranges.  Relax shape gates so these partial-
        # disc shapes still pass; the colour gate later confirms blue identity.
        if color == BallColor.BLUE and not edge_clipped:
            min_circularity = 0.55
            min_aspect      = 0.65

        # Sirkulæritet: ≥0.65 for normale, 0.55 for klippede (og blå).
        if circularity < min_circularity:
            return None

        # Corner detection — reject polygons with few vertices (squares/cubes).
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) <= min_vertices:
            return None

        # Aspect ratio — bounding box nær kvadratisk for en sirkel.
        aspect_ratio = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
        if aspect_ratio < min_aspect:
            return None

        # Soliditet — fyller det meste av sitt konvekse skrog
        # Scoping-fix: definer solidity utenfor if-blokk slik at den alltid er tilgjengelig.
        hull      = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity  = (area / hull_area) if hull_area > 0 else 0.0
        # Blue masks are often partial discs → relaxed solidity floor.
        _min_sol = 0.65 if (color == BallColor.BLUE and not edge_clipped) else 0.75
        if solidity < _min_sol:
            return None

        # Enclosing-circle fill is the most important round-object-only gate:
        # elongated arms, wrappers, tape strips and jeans folds can have decent
        # circularity/solidity, but they occupy too little of the circle needed
        # to enclose them.  A real ball fills most of its enclosing circle.
        circle_area = math.pi * (radius ** 2)
        circle_fill = area / circle_area if circle_area > 0 else 0.0
        if edge_clipped:
            min_circle_fill = 0.48
        elif color == BallColor.BLUE:
            min_circle_fill = 0.60   # relaxed: partial-disc masks fill less
        else:
            min_circle_fill = 0.72
        if circle_fill < min_circle_fill:
            return None

        # Ellipse fit catches near-oval fabric/arm blobs that still pass the
        # bounding-box aspect check after anti-aliasing or perspective rotation.
        ellipse_aspect = 1.0
        if edge_clipped:
            min_ellipse_aspect = 0.60
        elif color == BallColor.BLUE:
            min_ellipse_aspect = 0.70   # relaxed: crescent → oval fit
        else:
            min_ellipse_aspect = 0.82
        if len(contour) >= 5:
            try:
                (_ecx, _ecy), (axis_a, axis_b), _angle = cv2.fitEllipse(contour)
                major_axis = max(axis_a, axis_b)
                minor_axis = min(axis_a, axis_b)
                ellipse_aspect = minor_axis / major_axis if major_axis > 0 else 0.0
            except cv2.error:
                ellipse_aspect = 0.0
        if ellipse_aspect < min_ellipse_aspect:
            return None

        # Fargemetning: sample KUN piksler inne i konturen (ikke firkant-ROI).
        # Firkant-ROI inkluderer ~21.5% bakgrunnspikslene i hjørnene og trekker
        # ned sat_score. Konturmask gir rene ballverdier → korrekt confidence.
        sat_score = 0.0
        red_hue_ratio = 0.0
        blue_hue_ratio = 0.0
        if hsv is not None:
            _cmask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(_cmask, [contour], -1, 255, cv2.FILLED)
            _hue_flat = hsv[:, :, 0][_cmask > 0]
            _sat_flat = hsv[:, :, 1][_cmask > 0]
            _val_flat = hsv[:, :, 2][_cmask > 0]
            if _sat_flat.size > 0:
                # Ekskluder glanspiksler fra metningsberegningen.  Red/orange
                # balls under presentation lights often produce broad highlights
                # with S≈30-90 and V≈170-255, not only the pure-white S<30,V>210
                # case.  Use the broader exclusion only for red, where a hue-ratio
                # gate below still prevents neutral background from passing.
                if color == BallColor.RED:
                    _not_glare = ~((_sat_flat <= 90) & (_val_flat >= 165))
                else:
                    # For blue: exclude pure specular highlights (S<45, V>100)
                    # that were recovered by _repair_blue_mask.  These pixels
                    # have undefined hue and would dilute blue_hue_ratio and
                    # pull down sat_score, causing the colour gate to reject
                    # an otherwise valid cyan ball.
                    _not_glare = ~((_sat_flat < 45) & (_val_flat > 100))
                if np.sum(_not_glare) > 5:
                    sat_score = float(np.mean(_sat_flat[_not_glare])) / 255.0
                    if color == BallColor.RED:
                        _red_hues = ((_hue_flat[_not_glare] <= 35) |
                                     (_hue_flat[_not_glare] >= 145))
                        red_hue_ratio = float(np.mean(_red_hues))
                    if color == BallColor.BLUE:
                        _blue_hues = ((_hue_flat[_not_glare] >= BLUE_HUE_MIN) &
                                      (_hue_flat[_not_glare] <= BLUE_HUE_MAX))
                        blue_hue_ratio = float(np.mean(_blue_hues))
                else:
                    sat_score = float(np.mean(_sat_flat)) / 255.0
                    if color == BallColor.RED:
                        _red_hues = ((_hue_flat <= 35) | (_hue_flat >= 145))
                        red_hue_ratio = float(np.mean(_red_hues))
                    if color == BallColor.BLUE:
                        _blue_hues = (_hue_flat >= BLUE_HUE_MIN) & (_hue_flat <= BLUE_HUE_MAX)
                        blue_hue_ratio = float(np.mean(_blue_hues))

        # Kvalitetsscore: normalisert 0-1 for hver komponent.
        # Bonusen starter fra base-terskelene (0.65/0.75), slik at baller
        # som så vidt passerer base-kravene får noe bonus → ikke filtrert ut.
        cir_bonus = float(np.clip((circularity  - 0.65) / 0.35, 0.0, 1.0))
        asp_bonus = float(np.clip((aspect_ratio - 0.75) / 0.25, 0.0, 1.0))
        sol_bonus = float(np.clip((solidity     - 0.75) / 0.25, 0.0, 1.0))
        fill_bonus = float(np.clip((circle_fill - min_circle_fill) / (1.0 - min_circle_fill), 0.0, 1.0))
        ell_bonus = float(np.clip((ellipse_aspect - min_ellipse_aspect) / (1.0 - min_ellipse_aspect), 0.0, 1.0))
        col_bonus = float(np.clip((sat_score    - 0.30) / 0.70, 0.0, 1.0))

        color_conf = sat_score                                    # ren kontur-metning 0-1
        shape_conf = (cir_bonus * 0.35 + asp_bonus * 0.15 + sol_bonus * 0.15 +
                      fill_bonus * 0.25 + ell_bonus * 0.10)  # normalisert margin

        # ── Universal fargegate ─────────────────────────────────────────────
        # Shadows from balls on the desk can pass shape checks (roughly circular)
        # but have low saturation. CLAHE can boost shadow sat_score to ~0.30-0.38.
        # Real balls always have sat_score ≥ 0.45+, even matte ones.
        # Grey/neutral ground surfaces (e.g. textured carpet, concrete) can reach
        # sat_score ≈ 0.40-0.44 after CLAHE — raised red gate to 0.45 to reject.
        if color == BallColor.BLUE:
            # Lys/cyan-ish blå baller kan ha lavere snittmetning enn røde baller,
            # særlig ved lav/manuell eksponering der cyan ball-piksler havner rundt
            # H≈78-90, S≈45-70 og V≈25-70. Tillat lavere metning kun når konturen
            # er hue-dominert blå/cyan og fortsatt må passere de strenge
            # round-object gates above, so elongated jeans/arm blobs remain rejected.
            if sat_score < 0.18:
                return None
            if sat_score < 0.40 and blue_hue_ratio < 0.80:
                return None
        else:
            # Red/orange balls with broad glare can have lower mean saturation
            # after repaired highlight pixels are included in the contour.  Permit
            # that only when the non-glare pixels are still clearly red/orange.
            if sat_score < 0.35:
                return None
            if sat_score < 0.45 and red_hue_ratio < 0.65:
                return None

        # ── Edge-spesifikk fargegate ────────────────────────────────────────
        # Edge-klippede konturer med sub-normal circularity (< 0.65) MÅ kompensere
        # med sterk fargemetning. Ekte baller har sat_score ≥ 0.50 selv ved kanten,
        # mens kabler/ledninger typisk har sat_score ≈ 0.30-0.40.
        # Ikke-klippede konturer trenger ikke denne sjekken — de passerte allerede
        # circularity ≥ 0.65.
        if edge_clipped and circularity < 0.65:
            if color == BallColor.BLUE:
                if sat_score < 0.35 or blue_hue_ratio < 0.75:
                    return None
            elif sat_score < 0.45 and red_hue_ratio < 0.75:
                return None

        # ── Fix 3: Proporsjonal confidence ──────────────────────────────────
        # Erstatter det tidligere 90%-gulvet. Confidence reflekterer nå faktisk
        # kvalitet: 70% base + 30% kvalitetsbonus → range [0.70, 1.00].
        # En god ball (quality≈0.8) → ~94%. En marginal edge-klippet kontur
        # (quality≈0.1) → ~73%, som lettere filtreres bort.
        quality    = (cir_bonus * 0.28 + asp_bonus * 0.14 + sol_bonus * 0.14 +
                      fill_bonus * 0.20 + ell_bonus * 0.08 + col_bonus * 0.16)
        confidence = 0.70 + float(np.clip(quality * 0.30, 0.0, 0.30))

        # Avstandsberegning: d = (f * D_real) / D_pixel
        # D_pixel = diameter i piksler = radius * 2
        distance_cm = (self.focal_length_px * self.BALL_DIAMETER_MM) / (radius * 2 * 10.0)

        return DetectedBall(
            color=color,
            center=center,
            radius=float(radius),
            confidence=float(confidence),
            detection_method=method,
            distance_cm=round(distance_cm, 1),
            shape_confidence=float(min(shape_conf, 1.0)),
            color_confidence=float(min(color_conf, 1.0)),
        )
    
    # ─── Post-processing ──────────────────────────────────────────────

    def ensemble_merge(self, hsv_balls: List[DetectedBall], hough_balls: List[DetectedBall]) -> List[DetectedBall]:
        """
        Merger deteksjoner fra forskjellige metoder med ensemble voting.
        
        To deteksjoner "matcher" hvis de overlapper betydelig.
        Deteksjoner som finnes i flere metoder får høyere confidence.
        
        Args:
            hsv_balls: Deteksjoner fra HSV
            hough_balls: Deteksjoner fra Hough
            
        Returns:
            Mergede og validerte deteksjoner
        """
        all_detections = hsv_balls + hough_balls
        
        if len(all_detections) == 0:
            return []
        
        n = len(all_detections)
        
        # Union-Find for transitiv clustering
        parent = list(range(n))
        
        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        
        def union(x, y):
            parent[find(x)] = find(y)
        
        for i in range(n):
            for j in range(i + 1, n):
                b1, b2 = all_detections[i], all_detections[j]
                # Ulike farger merges ALDRI — en rød og en blå deteksjon er alltid ulike baller
                if b1.color != b2.color:
                    continue
                dx = b1.center[0] - b2.center[0]
                dy = b1.center[1] - b2.center[1]
                dist = math.hypot(dx, dy)
                # Merge hvis senter er innenfor 1.5× største radius —
                # Hough-senter (geometrisk) og HSV-senter (farge-tyngdepunkt) kan
                # lett ligge 10-20 px fra hverandre på en 40 px radius-ball.
                if dist < max(b1.radius, b2.radius) * 1.5:
                    union(i, j)
        
        # Grupper etter root
        cluster_map: dict = {}
        for i in range(n):
            root = find(i)
            cluster_map.setdefault(root, []).append(all_detections[i])
        
        # Merge hver cluster til én deteksjon
        merged_balls = []
        
        for cluster in cluster_map.values():
            best_ball = max(cluster, key=lambda b: b.confidence)
            color = best_ball.color

            # Bruk beste ball sitt senter (HSV moments-tyngdepunkt) istedenfor gjennomsnitt.
            # Gjennomsnitt mellom HSV-senter og Hough-senter gir ~8px periodisk hopp
            # hvert 4. frame (Hough-frame) som Kalman ikke klarer å kompensere raskt nok.
            center_x = best_ball.center[0]
            center_y = best_ball.center[1]
            avg_radius = float(np.mean([b.radius for b in cluster]))

            num_methods = len(set(b.detection_method for b in cluster))
            best_confidence  = max(b.confidence for b in cluster)
            final_confidence = min(best_confidence * (1.0 + 0.08 * (num_methods - 1)), 1.0)

            avg_shape = float(np.mean([b.shape_confidence for b in cluster]))
            avg_color = float(np.mean([b.color_confidence for b in cluster]))

            merged_ball = DetectedBall(
                color=color,
                center=(center_x, center_y),
                radius=avg_radius,
                confidence=float(final_confidence),
                detection_method="ensemble" if num_methods > 1 else best_ball.detection_method,
                shape_confidence=min(avg_shape, 1.0),
                color_confidence=min(avg_color, 1.0),
            )
            merged_balls.append(merged_ball)
        
        self.stats['ensemble_detections'] = len(merged_balls)
        return merged_balls

    def _post_merge_nms(self, balls: List[DetectedBall]) -> List[DetectedBall]:
        """
        Siste NMS-runde per farge: beholder bare den beste deteksjonen
        innenfor en avstand tilsvarende én radius.

        Håndterer tilfeller der USB-kabelens bøyninger skaper Hough-sirkler
        langt nok fra ballen til å unngå ensemble_merge, men som likevel er
        falske duplikater (fargeregion overlapper med den ekte ballen).
        """
        if len(balls) <= 1:
            return balls

        # Sorter: høyest confidence først (vi beholder "vinneren" per klynge)
        sorted_balls = sorted(balls, key=lambda b: b.confidence, reverse=True)
        kept = []

        for candidate in sorted_balls:
            duplicate = False
            for accepted in kept:
                if candidate.color != accepted.color:
                    continue
                dx = candidate.center[0] - accepted.center[0]
                dy = candidate.center[1] - accepted.center[1]
                dist = math.hypot(dx, dy)
                # Samme ball hvis sentrene er innenfor 1.3× radius.
                # 2.0× var for aggressivt: to baller side om side (avstand ~2×diameter)
                # ble feilaktig merget til én.
                if dist < max(accepted.radius, candidate.radius) * 1.3:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(candidate)

        return kept  # per-farge-filteret under sørger for korrekt cap per farge

    def _limit_per_color(self, balls: List[DetectedBall]) -> List[DetectedBall]:
        """
        Beholder kun de N beste deteksjonene per farge (sortert etter confidence).
        Forhindrer at falske positiver teller med når vi vet maks antall baller i scenen.
        """
        per_color: dict = defaultdict(list)
        for b in balls:
            per_color[b.color].append(b)

        result = []
        for color_balls in per_color.values():
            color_balls.sort(key=lambda b: b.confidence, reverse=True)
            result.extend(color_balls[:self.max_balls_per_color])
        return result

    def _verify_with_svm(self, frame: np.ndarray, balls: List[DetectedBall]) -> List[DetectedBall]:
        """
        SVM color verification — **DISABLED**.

        The SVM classifier erroneously relabels dark-blue balls as red.
        HSV+Hough ensemble is reliable enough without secondary SVM voting.
        This method is kept as a no-op hook for potential future re-enablement.
        """
        return balls

    # ─── Public API ─────────────────────────────────────────────────

    def detect_balls(self, frame: np.ndarray) -> Tuple[List[DetectedBall], Dict]:
        """
        Hovedfunksjon for å detektere baller med ensemble pipeline og adaptiv lyshåndtering.
        
        Args:
            frame: Input frame i BGR (3-kanals)
            
        Returns:
            Tuple (baller, statistikk): liste med DetectedBall og dict med statistikk
        """
        # Valider frame-input
        if frame is None or frame.size == 0:
            return [], self.stats.copy()
        if len(frame.shape) != 3 or frame.shape[2] != 3:
            return [], self.stats.copy()

        # Skaler ned for prosessering — 0.75 gir 480x300 på 640x400-input.
        # VIKTIG: 0.5 (320x200) ga 13×13 close-kernel = 87 % av balldiameter (15px ødelagt form).
        # Ved 0.75: ballradius ~11px, 13×13-kernel = 57 % av diameter → form-gates passerer.
        # Benchmark 02.04: 100 % deteksjon bekreftet med DETECTION_SCALE=0.75 (summary.json).
        _scale = self.DETECTION_SCALE
        proc_frame = cv2.resize(frame, (0, 0), fx=_scale, fy=_scale,
                                interpolation=cv2.INTER_LINEAR)

        # Compute scaled radius bounds as local variables (thread-safe — no mutation of self)
        orig_min_r = self.min_radius
        orig_max_r = self.max_radius
        scaled_min_r = max(5, int(orig_min_r * _scale))
        scaled_max_r = int(orig_max_r * _scale)

        # Temporarily set instance radius for pipeline methods that read self.min/max_radius
        self.min_radius = scaled_min_r
        self.max_radius = scaled_max_r

        try:
            # 1. Analyser lysforhold (300-700 lux range)
            lighting_info = self.analyze_lighting(proc_frame)
            self.stats['lighting_level'] = lighting_info['level']
            
            # 2. Appliser lyskompensasjon hvis nødvendig
            compensated_frame = self.apply_lighting_compensation(proc_frame, lighting_info)
            
            # 3. Color space conversions
            hsv  = cv2.cvtColor(compensated_frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(compensated_frame, cv2.COLOR_BGR2GRAY)

            # 4. Gaussian blur – kun på gray (HSV-masking trenger ikke blur)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            # 5. HSV multi-range detection med adaptiv justering
            red_balls_hsv, blue_balls_hsv = self.detect_with_hsv_multirange(hsv, lighting_info)
            hsv_balls = red_balls_hsv + blue_balls_hsv
            self.stats['hsv_red_candidates'] = len(red_balls_hsv)
            self.stats['hsv_blue_candidates'] = len(blue_balls_hsv)

            # 6. Hough Circle Transform — always run (interval is always 1)
            self._frame_counter += 1
            hough_balls = self.detect_with_hough(gray, hsv)
            self.stats['hough_red_candidates'] = len([b for b in hough_balls if b.color == BallColor.RED])
            self.stats['hough_blue_candidates'] = len([b for b in hough_balls if b.color == BallColor.BLUE])
            
            # 7. Ensemble merge - kombinerer begge metodene
            merged_balls = self.ensemble_merge(hsv_balls, hough_balls)
            self.stats['merged_red_candidates'] = len([b for b in merged_balls if b.color == BallColor.RED])
            self.stats['merged_blue_candidates'] = len([b for b in merged_balls if b.color == BallColor.BLUE])
            
            # 8. Post-merge NMS per farge: fjern gjenværende duplikater
            merged_balls = self._post_merge_nms(merged_balls)

            # 9. SVM-fargeverifisering (disabled — returns input unchanged)
            merged_balls = self._verify_with_svm(compensated_frame, merged_balls)

            # 10. Behold maks N baller per farge (sortert etter confidence)
            merged_balls = self._limit_per_color(merged_balls)

            # 11. Hard confidence-gate
            merged_balls = [
                b for b in merged_balls
                if b.confidence >= self.confidence_threshold
            ]

        finally:
            # Alltid gjenopprett originale radius-grenser
            self.min_radius, self.max_radius = orig_min_r, orig_max_r

        # Skaler koordinater og radius tilbake til original oppløsning
        inv = 1.0 / _scale
        for ball in merged_balls:
            ball.center = (int(ball.center[0] * inv), int(ball.center[1] * inv))
            ball.radius = ball.radius * inv
            # Omberegn avstand med korrekt (oppskalert) radius
            if ball.radius > 0:
                ball.distance_cm = round(
                    (self.focal_length_px * self.BALL_DIAMETER_MM) / (ball.radius * 2 * 10.0), 1
                )

        # 11. Persistent ball tracking (etter skalering) — tracker opererer i original oppløsning
        # slik at avstandsmålinger mellom frames er konsekvente.
        # KRITISK: bruk returverdien til tracker.update() som returnerer ALLE aktive balls,
        # inkludert Kalman-predikerte baller der deteksjonen droppet ut 1-N frames.
        # Uten dette forsvinner sirkelen fullstendig i dropout-frames → ser ut som hopping.
        active_balls = list(self._tracker.update(merged_balls).values())

        # Etter tracking: begrens til N beste per farge igjen.
        # Trackeren kan ha akkumulert 2+ spor av samme farge (f.eks. to røde spor fra
        # samme ball som midlertidig ble splittet til 2 konturer). Sorter på confidence
        # slik at vi beholder det sporet med høyest score.
        active_balls = self._limit_per_color(active_balls)

        return active_balls, self.stats.copy()
    
    def _draw_ball(
        self,
        output: np.ndarray,
        ball: DetectedBall,
        show_label: bool,
        others: Optional[List[Tuple[int, int, int]]] = None,
        ball_number: int = 0,
    ) -> None:
        """
        Tegner én ball med leader-linje til etikett utenfor sirkelen.

        others: liste med (cx, cy, radius) for alle andre baller i samme frame.
                Brukes til å velge en retning som unngår at etiketten
                legger seg oppå naboballer — viktig når det er 2–6 baller i scenen.
        """
        if ball.color == BallColor.RED:
            draw_color = (0, 0, 255)
            color_name = "ROD"
        elif ball.color == BallColor.BLUE:
            draw_color = (255, 0, 0)
            color_name = "BLA"
        else:
            draw_color = (128, 128, 128)
            color_name = "???"

        # Sirkel med sort kontur for å skille seg fra bakgrunnen
        cv2.circle(output, ball.center, int(ball.radius), (0, 0, 0), 6)
        cv2.circle(output, ball.center, int(ball.radius), draw_color, 3)
        cv2.circle(output, ball.center, 6, (0, 0, 0), -1)
        cv2.circle(output, ball.center, 4, (255, 255, 255), -1)

        FONT = cv2.FONT_HERSHEY_SIMPLEX

        # Ballnummer utenfor sirkelen – liten tekst oppe til høyre
        if ball_number > 0:
            num_text = str(ball_number)
            cx, cy = ball.center
            r = int(ball.radius)
            (nw, nh), _ = cv2.getTextSize(num_text, FONT, 0.45, 1)
            offset = int(r * 0.707) + 5
            nx = cx + offset - nw // 2
            ny = cy - offset + nh // 2
            cv2.putText(output, num_text, (nx, ny), FONT, 0.45, (0, 0, 0), 3)
            cv2.putText(output, num_text, (nx, ny), FONT, 0.45, (255, 255, 255), 1)

        if not show_label:
            return

        conf_pct  = int(ball.confidence * 100)
        dist_text = f"  {ball.distance_cm:.0f}cm" if ball.distance_cm else ""
        text = f"{color_name}  {conf_pct}%{dist_text}"

        f_scale, f_thick = 0.52, 1
        (tw, th), baseline = cv2.getTextSize(text, FONT, f_scale, f_thick)
        pad = 4

        cx, cy = ball.center
        r      = int(ball.radius)
        H, W   = output.shape[:2]
        GAP    = 14   # mellomrom fra sirkelkant til nærmeste tekstboks-kant

        # ── 8 kandidat-retninger (enhetsvektorer ved 45°-steg) ───────────────
        D = float(np.sqrt(2) / 2)
        candidates = [
            ( 0.0,  -1.0),   # opp
            (  D,    -D ),   # opp-høyre
            ( 1.0,   0.0),   # høyre
            (  D,     D ),   # ned-høyre
            ( 0.0,   1.0),   # ned
            ( -D,     D ),   # ned-venstre
            (-1.0,   0.0),   # venstre
            ( -D,    -D ),   # opp-venstre
        ]

        best_score = -1e9
        best_state = None  # (dx, dy, bx1, by1, bx2, by2, xt, yt)

        for dx, dy in candidates:
            # Ankerpunkt der linjen treffer tekstboksen
            ax = cx + int(dx * (r + GAP))
            ay = cy + int(dy * (r + GAP))

            # Horisontalt: tekst starter til høyre / slutter til venstre / sentrert
            if   dx >  0.1: xt = ax
            elif dx < -0.1: xt = ax - tw - 2 * pad
            else:           xt = cx - tw // 2 - pad

            # Vertikalt: boks over / under ankerpunktet / midtstilt
            if   dy < -0.1: yt = ay - pad
            elif dy >  0.1: yt = ay + th + pad
            else:           yt = ay + th // 4

            bx1 = xt - pad;       by1 = yt - th - pad
            bx2 = xt + tw + pad;  by2 = yt + baseline + pad

            # Straff hvis boksen stikker utenfor rammen
            in_frame    = (bx1 >= 0 and by1 >= 0 and bx2 <= W and by2 <= H)
            frame_score = 0.0 if in_frame else -40.0

            # Belønn stor avstand fra andre baller
            # (bokssenter vs. sirkelkant på naboballen)
            dist_score = 0.0
            if others:
                bcx = float(bx1 + bx2) / 2
                bcy = float(by1 + by2) / 2
                for ocx, ocy, or_ in others:
                    d = float(np.hypot(bcx - ocx, bcy - ocy)) - or_
                    dist_score += max(0.0, d)

            # Svak oppover-bias — etiketter over ballen er lettere å lese
            score = frame_score + dist_score + (-dy * 4.0)

            if score > best_score:
                best_score = score
                best_state = (dx, dy, bx1, by1, bx2, by2, xt, yt)

        if best_state is None:
            return

        dx, dy, bx1, by1, bx2, by2, xt, yt = best_state

        # Klem til rammen (edge case: ball svært nær kanten)
        xt  = max(pad, min(xt,  W - tw - 2 * pad))
        yt  = max(th + pad, min(yt, H - baseline - pad))
        bx1 = xt - pad;       by1 = yt - th - pad
        bx2 = xt + tw + pad;  by2 = yt + baseline + pad

        # ── Leader-linje ─────────────────────────────────────────────────────
        # Startpunkt: sirkelkantens punkt i valgt retning
        ls = (int(cx + dx * r), int(cy + dy * r))

        # Endepunkt: nærmeste kant på tekstboksen
        if   dx >  0.1: le = (bx1, (by1 + by2) // 2)
        elif dx < -0.1: le = (bx2, (by1 + by2) // 2)
        elif dy < -0.1: le = ((bx1 + bx2) // 2, by2)
        else:           le = ((bx1 + bx2) // 2, by1)

        cv2.line(output, ls, le, (0, 0, 0), 3)
        cv2.line(output, ls, le, (200, 200, 200), 1)
        cv2.circle(output, ls, 4, (0, 0, 0), -1)
        cv2.circle(output, ls, 3, (200, 200, 200), -1)

        # ── Tekstboks ────────────────────────────────────────────────────────
        cv2.rectangle(output, (bx1, by1), (bx2, by2), (20, 20, 20), -1)
        cv2.rectangle(output, (bx1, by1), (bx2, by2), (160, 160, 160), 1)
        cv2.putText(output, text, (xt, yt), FONT, f_scale, (255, 255, 255), f_thick)

    def _draw_hud_panel(
        self,
        output: np.ndarray,
        lines: List[Tuple[str, Tuple[int, int, int]]],
    ) -> None:
        """
        Tegner semi-transparent HUD-panel øverst til venstre med tekstlinjer.

        Args:
            output: Frame å tegne på (in-place)
            lines: Liste med (tekst, BGR-farge)
        """
        FONT       = cv2.FONT_HERSHEY_SIMPLEX
        FONT_SCALE = 0.52
        THICKNESS  = 1
        LINE_H     = 22
        PAD_X      = 10
        PAD_TOP    = 10

        # Beregn boks-dimensjoner (tekst-linjer + separatorer + blanke linjer)
        visible = [t for t, _ in lines if t.strip() and t != "---"]
        seps    = sum(1 for t, _ in lines if t == "---")
        blanks  = sum(1 for t, _ in lines if t and not t.strip())
        total_h = LINE_H * len(visible) + 10 * seps + (LINE_H // 2) * blanks
        max_w = max(
            (cv2.getTextSize(t, FONT, FONT_SCALE, THICKNESS)[0][0]
             for t, _ in lines if t.strip() and t != "---"),
            default=120,
        )
        box_w = min(PAD_X * 2 + max_w + 16, output.shape[1])
        box_h = min(PAD_TOP * 2 + total_h + LINE_H, output.shape[0])

        # Semi-transparent mørk bakgrunn
        roi  = output[0:box_h, 0:box_w].copy()
        dark = np.full_like(roi, (10, 10, 10))
        cv2.addWeighted(dark, 0.82, roi, 0.18, 0, roi)
        output[0:box_h, 0:box_w] = roi

        cv2.rectangle(output, (0, 0), (box_w - 1, box_h - 1), (100, 100, 100), 1)

        y_pos = PAD_TOP
        for text, color in lines:
            if text == "---":
                y_pos += 5
                cv2.line(output, (PAD_X // 2, y_pos), (box_w - PAD_X // 2, y_pos), (70, 70, 70), 1)
                y_pos += 5
                continue
            if not text.strip():
                y_pos += LINE_H // 2
                continue
            y_pos += LINE_H
            cv2.putText(output, text, (PAD_X, y_pos), FONT, FONT_SCALE, (0, 0, 0), THICKNESS + 2)
            cv2.putText(output, text, (PAD_X, y_pos), FONT, FONT_SCALE, color, THICKNESS)

    def draw_detections(
        self,
        frame: np.ndarray,
        balls: List[DetectedBall],
        show_info: bool = True,
        overlay: Optional[Dict] = None,
    ) -> np.ndarray:
        """
        Tegner detekterte baller og valgfri overlay-statistikk på frame.

        Args:
            frame: Frame å tegne på
            balls: Liste med baller
            show_info: Vis info-tekst og HUD-panel
            overlay: Valgfri dict med ekstra nøkkel-verdi-par til HUD-panelet
                     Eksempel: {"FPS": 15, "Frame": 42}

        Returns:
            Annotert frame (kopi — original uendret)
        """
        output = frame.copy()

        for i, ball in enumerate(balls):
            others = [
                (b.center[0], b.center[1], int(b.radius))
                for j, b in enumerate(balls) if j != i
            ]
            # Bruk tracker-ID for stabilt ballnummer på tvers av frames
            bnr = ball.track_id if ball.track_id > 0 else (i + 1)
            self._draw_ball(output, ball, show_label=show_info, others=others, ball_number=bnr)

        if not show_info:
            return output

        WHITE = (255, 255, 255)
        SEP   = (0, 0, 0)         # farge ignoreres for separator-linjer
        lines: List[Tuple[str, Tuple[int, int, int]]] = []

        fps_v = str(overlay.get("FPS", "--")) if overlay else "--"
        lines.append((f"BALL DETECTOR      {fps_v} FPS", WHITE))
        lines.append(("---", SEP))

        frames_v = str(overlay.get("Frames", "--")) if overlay else "--"
        snitt_v  = str(overlay.get("Snitt konf", "--")) if overlay else "--"
        lines.append((f"Frame: {frames_v}   Snitt: {snitt_v}", WHITE))

        red_list  = [b for b in balls if b.color == BallColor.RED]
        blue_list = [b for b in balls if b.color == BallColor.BLUE]
        lines.append((f"Baller: {len(balls)}  (Rod:{len(red_list)}  Bla:{len(blue_list)})", WHITE))

        if balls:
            lines.append(("---", SEP))
            for ball in balls:
                clabel = "ROD" if ball.color == BallColor.RED else "BLA"
                conf   = int(ball.confidence * 100)
                dist_s = f"  {ball.distance_cm:.0f}cm" if ball.distance_cm else ""
                tid    = ball.track_id if ball.track_id > 0 else "?"
                lines.append((f"  #{tid} {clabel}  {conf}%{dist_s}", WHITE))

        if lines:
            self._draw_hud_panel(output, lines)

        return output

    def get_statistics(self) -> Dict[str, int]:
        """Returnerer gjeldende deteksjonsstatistikk."""
        return self.stats.copy()

    def reset(self) -> None:
        """Nullstill statistikk, Hough-cache og ball-tracker."""
        self.stats = {
            'total_hsv_detections':      0,
            'total_hough_detections':    0,
            'ensemble_detections': 0,
            'lighting_level':      'unknown',
        }
        self._frame_counter = 0
        self._tracker.reset()

    def reset_tracker(self) -> None:
        """Clear all active tracks without resetting statistics.

        Call this between scan rounds (e.g. after the arm has picked up a
        ball and moved away) so that stale Kalman predictions from the
        previous scan do not produce phantom detections in the next scan.
        """
        self._tracker.reset()
