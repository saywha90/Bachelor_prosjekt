"""
Simple Ball Detector
====================

Ensemble-system for pålitelig deteksjon av røde og blå baller med
Luxonis OAK Series 2 kamera.

Deteksjonsrørledning:
  1. Multi-range HSV color detection (6 red ranges, 3 blue ranges)
  2. Hough Circle Transform (geometrisk validering)
  3. Ensemble voting (slår sammen begge metoder)
  4. Adaptiv lyskompensasjon (CLAHE ved lavt lys)
  5. SVM-fargeverifisering (sekundær — korrigerer feil fargelabel)

HSV-ranges er kalibrert for OAK IMX378-sensor: ballene er nesten svarte
(V ≈ 14–45) men maksimalt mettet (S ≈ 255), som er den primære diskriminatoren.

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


class SimpleBallDetector:
    """
    Ball detector med ensemble approach og adaptiv lyshåndtering.

    Rørledning per frame:
      1. Lysanalyse — klassifiser som low / medium / high
      2. CLAHE-kompensasjon ved lavt lys
      3. Multi-range HSV deteksjon (6 red, 3 blue ranges)
      4. Hough Circle Transform (geometrisk validering)
      5. Ensemble merge + NMS
      6. SVM fargeverifisering (sekundær)
      7. Returner maks N baller per farge
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
        self.morph_kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))

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

        # Fyll innvendige hull som skyldes gjenskin/spekulær highlight:
        # lyse flekker inne i ballen matcher ingen fargerange → hull i masken
        # → sirkularitet synker → deteksjonen feiler.
        # Flood fill fra hjørne → bakgrunn blir hvit → bitwise_not gir kun lukkede hull.
        _flood = combined.copy()
        _ff_mask = np.zeros((combined.shape[0] + 2, combined.shape[1] + 2), dtype=np.uint8)
        cv2.floodFill(_flood, _ff_mask, (0, 0), 255)
        combined = cv2.bitwise_or(combined, cv2.bitwise_not(_flood))

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
            minDist=max(80, gray.shape[1] // 10),
            param1=50,   # Canny edge threshold
            param2=20,   # Senket fra 28 → 20 så skinnende baller med ujevne kanter fanges
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
                        edges = cv2.Canny(roi, 50, 150)
                        edge_ratio = np.sum(edges > 0) / roi.size
                        confidence = min(edge_ratio * 3.0, 1.0)  # Scale til 0-1
                        
                        if confidence > self.confidence_threshold:
                            ball = DetectedBall(
                                color=color,
                                center=(x, y),
                                radius=float(radius),
                                confidence=confidence,
                                detection_method="hough"
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
        
        # Bare tell piksler med tilstrekkelig farge og lysstyrke
        valid_mask = (sat_ch >= 60) & (val_ch >= 40)
        valid_pixels = int(np.sum(valid_mask))
        
        if valid_pixels < roi.shape[0] * roi.shape[1] * 0.1:
            return BallColor.UNKNOWN
        
        # Rød: Hue 0-20 ELLER 160-179 (wraparound)
        red_mask = valid_mask & ((hue_ch <= 20) | (hue_ch >= 160))
        red_pixels = int(np.sum(red_mask))
        
        # Blå: Hue 95-135
        blue_mask = valid_mask & (hue_ch >= 95) & (hue_ch <= 135)
        blue_pixels = int(np.sum(blue_mask))
        
        # Bestemmelse: flertall av gyldige piksler, minimum 50% av valid
        # Høy terskel hindrer kabelbøyninger (sirkelformede) i å bli feilklassifisert
        if red_pixels > blue_pixels and red_pixels >= valid_pixels * 0.5:
            return BallColor.RED
        if blue_pixels > red_pixels and blue_pixels >= valid_pixels * 0.5:
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
        
        # Minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        
        # Radius check
        if radius < self.min_radius or radius > self.max_radius:
            return None
        
        # Circularity check (4πA / P²) - perfekt sirkel = 1.0
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return None
        circularity = (4 * np.pi * area) / (perimeter ** 2)

        # Krav: ≥65% sirkulær etter morfologisk closing.
        # En ekte ball gir en kompakt, tilnærmet sirkulær HSV-maske.
        if circularity < 0.65:
            return None

        # Aspect ratio — bounding box skal være nær kvadratisk for en sirkel
        bx, by, bw, bh = cv2.boundingRect(contour)
        aspect_ratio = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
        if aspect_ratio < 0.75:
            return None

        # Soliditet — fyller det meste av sitt konvekse skrog (ball er konveks)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = area / hull_area
            if solidity < 0.72:
                return None
        
        # Confidence basert på form + fargemetning.
        # Metning (S-kanal) inne i konturen belønner sterkt fargete baller —
        # dette løfter rød confidence som ellers straffes av wrapround-masken.
        ideal_area = np.pi * (radius ** 2)
        area_match = min(area / ideal_area, ideal_area / area)

        sat_score = 0.0
        if hsv is not None:
            cx_i, cy_i = int(x), int(y)
            r_i = max(1, int(radius))
            roi_s = hsv[max(0, cy_i - r_i):min(hsv.shape[0], cy_i + r_i),
                        max(0, cx_i - r_i):min(hsv.shape[1], cx_i + r_i), 1]
            roi_v = hsv[max(0, cy_i - r_i):min(hsv.shape[0], cy_i + r_i),
                        max(0, cx_i - r_i):min(hsv.shape[1], cx_i + r_i), 2]
            if roi_s.size > 0:
                # Ekskluder rene gjensikinspunkter (S<30 og V>210) fra metningsberegningen
                # slik at et skinnende lyspunkt ikke trekker ned sat_score urettmessig.
                not_glare = ~((roi_s < 30) & (roi_v > 210))
                if np.sum(not_glare) > 5:
                    sat_score = float(np.mean(roi_s[not_glare])) / 255.0
                else:
                    sat_score = float(np.mean(roi_s)) / 255.0

        # Vekter: sirkulæritet 40%, areal-match 25%, aspekt 15%, metning 20%
        confidence = (circularity * 0.40) + (area_match * 0.25) + (aspect_ratio * 0.15) + (sat_score * 0.20)

        # Avstandsberegning: d = (f * D_real) / D_pixel
        # D_pixel = diameter i piksler = radius * 2
        distance_cm = (self.focal_length_px * self.BALL_DIAMETER_MM) / (radius * 2 * 10.0)

        return DetectedBall(
            color=color,
            center=center,
            radius=float(radius),
            confidence=float(confidence),
            detection_method=method,
            distance_cm=round(distance_cm, 1)
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
            
            avg_x = int(np.mean([b.center[0] for b in cluster]))
            avg_y = int(np.mean([b.center[1] for b in cluster]))
            avg_radius = np.mean([b.radius for b in cluster])
            
            num_methods = len(set(b.detection_method for b in cluster))
            avg_confidence = np.mean([b.confidence for b in cluster])
            final_confidence = min(avg_confidence * (1.0 + 0.3 * (num_methods - 1)), 1.0)
            
            merged_ball = DetectedBall(
                color=color,
                center=(avg_x, avg_y),
                radius=float(avg_radius),
                confidence=float(final_confidence),
                detection_method="ensemble" if num_methods > 1 else best_ball.detection_method
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

        # Skaler ned for prosessering — ved USB 2.0 (640x400 native) skalerer vi til
        # 480x300 for ekstra marginer på tregere hardware (Mac via Dell-adapter).
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
            hsv = cv2.cvtColor(compensated_frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(compensated_frame, cv2.COLOR_BGR2GRAY)
            
            # 4. Gaussian blur for å redusere noise
            hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 5. HSV multi-range detection med adaptiv justering
            red_balls_hsv, blue_balls_hsv = self.detect_with_hsv_multirange(hsv, lighting_info)
            hsv_balls = red_balls_hsv + blue_balls_hsv
            
            # 6. Hough Circle detection
            hough_balls = self.detect_with_hough(gray, hsv)
            
            # 7. Ensemble merge - kombinerer begge metodene
            merged_balls = self.ensemble_merge(hsv_balls, hough_balls)
            
            # 8. Post-merge NMS per farge: fjern gjenværende duplikater
            merged_balls = self._post_merge_nms(merged_balls)

            # 9. SVM-fargeverifisering (sekundær — korrigerer feil fargelabel ved høy konfidanse)
            merged_balls = self._verify_with_svm(compensated_frame, merged_balls)

            # 10. Behold maks N baller per farge (sortert etter confidence)
            merged_balls = self._limit_per_color(merged_balls)

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

        return merged_balls, self.stats.copy()
    
    def _draw_ball(
        self,
        output: np.ndarray,
        ball: DetectedBall,
        show_label: bool,
        others: Optional[List[Tuple[int, int, int]]] = None,
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

        if not show_label:
            return

        dist_text    = f"{ball.distance_cm:.0f} cm" if ball.distance_cm else ""
        conf_pct     = int(ball.confidence * 100)
        method_short = {"hsv": "HSV", "hough": "HGH", "ensemble": "ENS"}.get(
            ball.detection_method, ball.detection_method[:3].upper()
        )
        text = f"{color_name}  {dist_text}  {conf_pct}%  [{method_short}]"

        FONT = cv2.FONT_HERSHEY_SIMPLEX
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
        cv2.line(output, ls, le, draw_color, 1)
        cv2.circle(output, ls, 4, (0, 0, 0), -1)
        cv2.circle(output, ls, 3, draw_color, -1)

        # ── Tekstboks ────────────────────────────────────────────────────────
        cv2.rectangle(output, (bx1, by1), (bx2, by2), (20, 20, 20), -1)
        cv2.rectangle(output, (bx1, by1), (bx2, by2), draw_color, 1)
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
        FONT_SCALE = 0.42
        THICKNESS  = 1
        LINE_H     = 18
        PAD_X      = 8
        PAD_TOP    = 6

        max_w = max(cv2.getTextSize(t, FONT, FONT_SCALE, THICKNESS)[0][0] for t, _ in lines)
        box_w = min(PAD_X * 2 + max_w + 8, output.shape[1])
        box_h = min(PAD_TOP * 2 + LINE_H * len(lines), output.shape[0])

        # Semi-transparent mørk bakgrunn
        roi  = output[0:box_h, 0:box_w].copy()
        dark = np.full_like(roi, (15, 15, 15))
        cv2.addWeighted(dark, 0.78, roi, 0.22, 0, roi)
        output[0:box_h, 0:box_w] = roi

        cv2.rectangle(output, (0, 0), (box_w - 1, box_h - 1), (80, 80, 80), 1)

        for i, (text, color) in enumerate(lines):
            y = PAD_TOP + LINE_H // 2 + LINE_H // 4 + i * LINE_H
            cv2.putText(output, text, (PAD_X, y), FONT, FONT_SCALE, (0, 0, 0), THICKNESS + 2)
            cv2.putText(output, text, (PAD_X, y), FONT, FONT_SCALE, color,     THICKNESS)

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
            self._draw_ball(output, ball, show_label=show_info, others=others)

        if not show_info:
            return output

        WHITE = (255, 255, 255)
        lines: List[Tuple[str, Tuple[int, int, int]]] = []

        if self.enable_adaptive_lighting:
            level = self.stats.get('lighting_level', 'unknown')
            label_map = {
                'low':    "Light: LOW  (300-400 lux)",
                'medium': "Light: MED  (400-550 lux)",
                'high':   "Light: HIGH (550-700 lux)",
            }
            lines.append((label_map.get(level, "Light: UNKNOWN"), WHITE))

        if overlay:
            for key, val in overlay.items():
                lines.append((f"{key}: {val}", WHITE))

        if lines:
            self._draw_hud_panel(output, lines)

        return output

    def get_statistics(self) -> Dict[str, int]:
        """Returnerer gjeldende deteksjonsstatistikk."""
        return self.stats.copy()

