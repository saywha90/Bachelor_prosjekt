"""
Simple Ball Detector
====================

Ensemble-system for pålitelig deteksjon av røde og blå baller med
Luxonis OAK Series 2 kamera.

Deteksjonsrørledning:
  1. Multi-range HSV color detection (2 red ranges, 2 blue ranges)
  2. Hough Circle Transform (geometrisk validering, aktiveres hvert N-te frame)
  3. Ensemble voting (slår sammen begge metoder)
  4. Adaptiv lyskompensasjon (CLAHE ved lavt lys)
  5. SVM-fargeverifisering (sekundær — korrigerer feil fargelabel)
  6. Kalman-filter ball-tracker (stabile ID-er på tvers av frames)

HSV-ranges er kalibrert live for OAK IMX378-sensor.

Author: Bachelor Project 2026 - Autonomia
"""

import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum

# SVM-fargebeklassifiserer (sekundær voting) — laster ved import
try:
    from vision.color_histogram_classifier import ColorHistogramClassifier as _CHC
    _SVM_AVAILABLE = True
except ImportError:
    _SVM_AVAILABLE = False
    _CHC = None

__all__ = ["SimpleBallDetector", "BallColor", "DetectedBall"]


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
                    kf = self._kalman.get(oid)
                    if kf is not None:
                        kf.statePost    = kf.statePre.copy()
                        kf.errorCovPost = kf.errorCovPre.copy()
                    if oid in predicted and oid in self._objects:
                        px, py = predicted[oid]
                        self._objects[oid].center = (int(round(px)), int(round(py)))
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
                cost[i, j] = float(np.sqrt(dx * dx + dy * dy))

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
                    kf = self._kalman.get(oid)
                    if kf is not None:
                        kf.statePost    = kf.statePre.copy()
                        kf.errorCovPost = kf.errorCovPre.copy()
                    if oid in predicted and oid in self._objects:
                        px, py = predicted[oid]
                        self._objects[oid].center = (int(round(px)), int(round(py)))

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
      3. Multi-range HSV deteksjon (6 red, 3 blue ranges)
      4. Hough Circle Transform (geometrisk validering, cachet)
      5. Ensemble merge + NMS
      6. SVM fargeverifisering (sekundær)
      7. Persistent ball tracking (stabile ID-er)
      8. Returner maks N baller per farge
    """
    
    # Kjent balldiameter i mm (brukes til avstandsberegning)
    BALL_DIAMETER_MM = 50.0

    # ─── Initialization ──────────────────────────────────────────────────

    def __init__(self,
                 min_radius: int = 10,
                 max_radius: int = 150,
                 confidence_threshold: float = 0.35,
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
        self.red_ranges = [
            # Rød høy side (H wraparound nær 180) — primær range
            (np.array([165, 120, 130]), np.array([179, 255, 255])),
            # Rød lav side (H wraparound fra 0) — sikrer at H=0-5 fanges
            (np.array([0,   120, 130]), np.array([6,   255, 255])),
        ]

        # Multi-range HSV thresholds for BLUE
        # ✅ KALIBRERT live med diagnose_detection.py — egne målinger på faktiske baller
        # Målte piksler: H=103-110, S=174-255, V=92-200
        self.blue_ranges = [
            # Blå — primær range (høy metning, kjernen av ballen)
            (np.array([100, 200,  85]), np.array([115, 255, 255])),
            # Litt bredere for kant-piksler (målte S ned til 174)
            (np.array([ 98, 170,  85]), np.array([118, 255, 255])),
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

        # SVM-fargebeklassifiserer — lastes hvis modell finnes
        self._svm_classifier = None
        if _SVM_AVAILABLE:
            _model_path = Path(__file__).parent / "models" / "ball_color_classifier.pkl"
            try:
                self._svm_classifier = _CHC(str(_model_path))
            except (FileNotFoundError, Exception):
                pass

        # Statistics
        self.stats = {
            'hsv_detections': 0,
            'hough_detections': 0,
            'ensemble_detections': 0,
            'lighting_level': 'unknown'
        }
        self._frame_counter  = 0
        self._hough_interval = 9999      # effektivt deaktivert
        self._hough_cache: List[DetectedBall] = []
        self._tracker = BallTracker(
            max_disappeared=6, max_distance=100.0
        )
    
    # ─── Lighting analysis ─────────────────────────────────────────────

    def analyze_lighting(self, frame: np.ndarray) -> Dict[str, any]:
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
        
        Args:
            frame: Input frame i BGR
            lighting_info: Lysanalyse fra analyze_lighting()
            
        Returns:
            Kompensert frame
        """
        if not self.enable_adaptive_lighting:
            return frame
        
        # Ved lavt lys (300-400 lux): Bruk CLAHE for å forbedre kontrast
        if lighting_info['needs_boost']:
            # Konverter til LAB color space
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Appliser CLAHE på L-kanalen (bruker forhåndsopprettet instans)
            l = self.clahe.apply(l)
            
            # Merge tilbake
            enhanced = cv2.merge([l, a, b])
            frame = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        return frame
    
    def get_adaptive_hsv_ranges(self, lighting_info: Dict) -> Tuple[List, List]:
        """
        Justerer HSV-ranges dynamisk basert på lysforhold.
        
        Args:
            lighting_info: Lysanalyse fra analyze_lighting()
            
        Returns:
            Tuple med (red_ranges, blue_ranges) justert for lysforhold
        """
        if not self.enable_adaptive_lighting:
            return self.red_ranges, self.blue_ranges
        
        red_ranges_adjusted = []
        blue_ranges_adjusted = []
        
        # Juster basert på lysnivå
        if lighting_info['level'] == 'low':
            # Lavt lys (300-400 lux): Utvid V-range nedover, reduser S-krav
            for lower, upper in self.red_ranges:
                new_lower = lower.copy()
                new_upper = upper.copy()
                new_lower[1] = max(60, lower[1] - 20)  # Reduser minimum saturation
                new_lower[2] = max(40, lower[2] - 20)  # Reduser minimum value
                red_ranges_adjusted.append((new_lower, new_upper))
            
            for lower, upper in self.blue_ranges:
                new_lower = lower.copy()
                new_upper = upper.copy()
                new_lower[1] = max(50, lower[1] - 20)
                new_lower[2] = max(40, lower[2] - 20)
                blue_ranges_adjusted.append((new_lower, new_upper))
        
        elif lighting_info['level'] == 'high':
            # Høyt lys (550-700 lux): Stram inn S- og V-krav for å unngå falske positiver
            for lower, upper in self.red_ranges:
                new_lower = lower.copy()
                new_upper = upper.copy()
                new_lower[1] = min(255, lower[1] + 10)  # Øk minimum saturation
                new_lower[2] = min(255, lower[2] + 10)  # Øk minimum value
                red_ranges_adjusted.append((new_lower, new_upper))
            
            for lower, upper in self.blue_ranges:
                new_lower = lower.copy()
                new_upper = upper.copy()
                new_lower[1] = min(255, lower[1] + 10)
                new_lower[2] = min(255, lower[2] + 10)
                blue_ranges_adjusted.append((new_lower, new_upper))
        
        else:
            # Medium lys (400-550 lux): Bruk standard ranges
            red_ranges_adjusted = self.red_ranges
            blue_ranges_adjusted = self.blue_ranges
        
        return red_ranges_adjusted, blue_ranges_adjusted
    
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
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv, lower, upper))

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
        if lighting_info and self.enable_adaptive_lighting:
            red_ranges, blue_ranges = self.get_adaptive_hsv_ranges(lighting_info)
        else:
            red_ranges, blue_ranges = self.red_ranges, self.blue_ranges

        red_balls  = self._apply_hsv_ranges(hsv, red_ranges,  BallColor.RED)
        blue_balls = self._apply_hsv_ranges(hsv, blue_ranges, BallColor.BLUE)

        self.stats['hsv_detections'] += len(red_balls) + len(blue_balls)
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
            param2=35,   # 35 stemmer krevd → kun klare, godt-definerte sirkler (var 25)
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        detected_balls = []
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            for (x, y, radius) in circles:
                # Bestem farge ved å sjekke HSV-verdier i sentrum
                color = self._determine_color_from_hsv(hsv, (x, y), radius)
                
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
        
        self.stats['hough_detections'] += len(detected_balls)
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
        
        # Bare tell piksler med tilstrekkelig farge og lysstyrke.
        # sat >= 90 ekskluderer grå/brunlige overflater som ellers trigger rød-range.
        valid_mask = (sat_ch >= 90) & (val_ch >= 50)
        valid_pixels = int(np.sum(valid_mask))

        # Krev minst 25% av ROI-pikslene å ha gyldig farge (var 10%).
        # Kabelbøyninger er typisk < 25% farget i et 60%-radius-sample.
        if valid_pixels < roi.shape[0] * roi.shape[1] * 0.25:
            return BallColor.UNKNOWN
        
        # Rød: Hue 0-20 ELLER 160-179 (wraparound)
        red_mask = valid_mask & ((hue_ch <= 20) | (hue_ch >= 160))
        red_pixels = int(np.sum(red_mask))
        
        # Blå: Hue 95-135
        blue_mask = valid_mask & (hue_ch >= 95) & (hue_ch <= 135)
        blue_pixels = int(np.sum(blue_mask))
        
        # Bestemmelse: klar majoritet av gyldige piksler (60%), ikke bare 50%.
        # Setter en høyere bar slik at blandete fargeregioner ikke godkjennes.
        if red_pixels > blue_pixels and red_pixels >= valid_pixels * 0.60:
            return BallColor.RED
        if blue_pixels > red_pixels and blue_pixels >= valid_pixels * 0.60:
            return BallColor.BLUE
        
        return BallColor.UNKNOWN
    
    def _validate_contour(self, contour: np.ndarray, color: BallColor, method: str,
                           hsv: Optional[np.ndarray] = None) -> Optional['DetectedBall']:
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
        x, y = cx, cy  # For SAT-ROI-utregning nedenfor
        
        # Radius check
        if radius < self.min_radius or radius > self.max_radius:
            return None
        
        # Circularity check (4πA / P²) - perfekt sirkel = 1.0
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return None
        circularity = (4 * np.pi * area) / (perimeter ** 2)

        # Sirkulæritet: ≥0.60.
        # En ekte ball med spekulær highlight og 13×13 closing oppnår typisk
        # 0.65-0.95 ved halvskala. Kabler/kluter ender på 0.20-0.55.
        if circularity < 0.60:
            return None

        # Aspect ratio — bounding box nær kvadratisk for en sirkel
        bx, by, bw, bh = cv2.boundingRect(contour)
        aspect_ratio = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
        if aspect_ratio < 0.70:
            return None

        # Soliditet — fyller det meste av sitt konvekse skrog
        # Scoping-fix: definer solidity utenfor if-blokk slik at den alltid er tilgjengelig.
        hull      = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity  = (area / hull_area) if hull_area > 0 else 0.0
        if solidity < 0.75:
            return None

        # Fargemetning: sample KUN piksler inne i konturen (ikke firkant-ROI).
        # Firkant-ROI inkluderer ~21.5% bakgrunnspikslene i hjørnene og trekker
        # ned sat_score. Konturmask gir rene ballverdier → korrekt confidence.
        sat_score = 0.0
        if hsv is not None:
            _cmask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(_cmask, [contour], -1, 255, cv2.FILLED)
            _sat_flat = hsv[:, :, 1][_cmask > 0]
            _val_flat = hsv[:, :, 2][_cmask > 0]
            if _sat_flat.size > 0:
                # Ekskluder rene glanspiksler (S<30, V>210) fra metningsberegningen
                _not_glare = ~((_sat_flat < 30) & (_val_flat > 210))
                if np.sum(_not_glare) > 5:
                    sat_score = float(np.mean(_sat_flat[_not_glare])) / 255.0
                else:
                    sat_score = float(np.mean(_sat_flat)) / 255.0

        # Confidence: 90 % gulv for alle baller som passerer gate-filtrene,
        # + opptil 10 % bonus for eksepsjonell form/farge-kvalitet.
        # Normalisert: 0.0 ved terskelverdi, 1.0 ved perfekt verdi.
        cir_bonus = float(np.clip((circularity  - 0.60) / 0.40, 0.0, 1.0))
        asp_bonus = float(np.clip((aspect_ratio - 0.70) / 0.30, 0.0, 1.0))
        sol_bonus = float(np.clip((solidity     - 0.75) / 0.25, 0.0, 1.0))
        col_bonus = float(np.clip((sat_score    - 0.40) / 0.60, 0.0, 1.0))
        quality   = cir_bonus * 0.40 + asp_bonus * 0.20 + sol_bonus * 0.20 + col_bonus * 0.20
        confidence = 0.90 + float(np.clip(quality * 0.10, 0.0, 0.10))
        # → GARANTERT ≥ 90 % for enhver ball som passerer alle gate-filtrene
        # → Opptil 100 % for en svært god ball (sirkulær, fast, høy metning)

        color_conf = sat_score                                    # ren kontur-metning 0-1
        shape_conf = cir_bonus * 0.50 + asp_bonus * 0.25 + sol_bonus * 0.25  # normalisert margin

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
                dist = np.sqrt(dx**2 + dy**2)
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
                dist = np.sqrt(dx**2 + dy**2)
                # Samme ball hvis senter er innenfor den størst aksepterte radiusen
                if dist < max(accepted.radius, candidate.radius) * 2.0:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(candidate)

        return kept[:self.max_balls_per_color * 2]  # hard cap før per-farge-filter

    def _limit_per_color(self, balls: List[DetectedBall]) -> List[DetectedBall]:
        """
        Beholder kun de N beste deteksjonene per farge (sortert etter confidence).
        Forhindrer at falske positiver teller med når vi vet maks antall baller i scenen.
        """
        from collections import defaultdict
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
        Sekundær fargeverifisering med SVM-klassifiserer.
        Korrigerer fargelabel hvis SVM har >= 75% konfidanse og disagreer med HSV/Hough.

        Args:
            frame: Kompensert BGR-frame
            balls: Detekterte baller fra ensemble

        Returns:
            Baller med potensielt korrigert fargelabel
        """
        if self._svm_classifier is None or len(balls) == 0:
            return balls

        h_fr, w_fr = frame.shape[:2]

        for ball in balls:
            cx, cy = ball.center
            r = max(1, int(ball.radius))

            x1 = max(0, cx - r)
            y1 = max(0, cy - r)
            x2 = min(w_fr, cx + r)
            y2 = min(h_fr, cy + r)

            roi = frame[y1:y2, x1:x2]
            if roi.size < 64:
                continue

            svm_color, svm_conf = self._svm_classifier.predict(roi)

            if svm_conf >= 0.75:
                if svm_color == "red":
                    ball.color = BallColor.RED
                elif svm_color == "blue":
                    ball.color = BallColor.BLUE

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
        # Benchmark 02.04: 100 % deteksjon bekreftet med _SCALE=0.75 (summary.json).
        _SCALE = 0.75
        proc_frame = cv2.resize(frame, (0, 0), fx=_SCALE, fy=_SCALE,
                                interpolation=cv2.INTER_LINEAR)

        # Juster radius-grenser til skalert koordinatrom
        orig_min_r, orig_max_r = self.min_radius, self.max_radius
        self.min_radius = max(5, int(orig_min_r * _SCALE))
        self.max_radius = int(orig_max_r * _SCALE)

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

            # 6. Hough Circle Transform — kjøres hvert N-te frame for ytelse.
            # Når Hough ikke kjøres brukes tom liste: stale cache ville ellers skape
            # spøkelsesballer fra forrige Hough-kjøring (ball har beveget seg).
            self._frame_counter += 1
            if (self._frame_counter - 1) % self._hough_interval == 0:
                hough_balls = self.detect_with_hough(gray, hsv)
            else:
                hough_balls = []
            
            # 7. Ensemble merge - kombinerer begge metodene
            merged_balls = self.ensemble_merge(hsv_balls, hough_balls)
            
            # 8. Post-merge NMS per farge: fjern gjenværende duplikater
            merged_balls = self._post_merge_nms(merged_balls)

            # 9. SVM-fargeverifisering (sekundær — korrigerer feil fargelabel)
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
        inv = 1.0 / _SCALE
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
            'hsv_detections':      0,
            'hough_detections':    0,
            'ensemble_detections': 0,
            'lighting_level':      'unknown',
        }
        self._frame_counter = 0
        self._hough_cache   = []
        self._tracker.reset()

