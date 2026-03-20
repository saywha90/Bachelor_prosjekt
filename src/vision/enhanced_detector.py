"""
Simple Ball Detector - Reliable Detection System with Adaptive Lighting
========================================================================

Et forenklet ensemble-system for pålitelig deteksjon av røde og blå baller.

Kombinerer:
- Multi-range HSV color detection (6 red ranges, 3 blue ranges)
- Hough Circle Transform (geometric validation)
- Ensemble voting (combines both methods)
- Adaptive lighting compensation (300-700 lux)

Designet for å være ENKEL og PÅLITELIG under varierende lysforhold.
Detekterer kun røde og blå baller i kameraets synsfelt.

Author: Bachelor Project 2026 - Autonomia
Date: March 2026 (Simplified version with adaptive lighting)
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum


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
    Forenklet ball detector med ensemble approach og adaptiv lyshåndtering.
    
    Kombinerer to robuste metoder:
    1. Multi-range HSV color detection (6 red, 3 blue ranges)
    2. Hough Circle Transform (geometric validation)
    3. Ensemble voting (combines results for reliability)
    4. Adaptive lighting compensation (300-700 lux)
    
    Designet for å være ENKEL og PÅLITELIG for statiske baller under varierende lys.
    """
    
    # Kjent balldiameter i mm (brukes til avstandsberegning)
    BALL_DIAMETER_MM = 50.0

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
        # ✅ KALIBRERT basert på analyse av 18 bilder (34M piksler) av din røde ball
        # Hue: 0-11, Saturation: 147-255, Value: 59-255
        self.red_ranges = [
            # Bright red (godt lys) - high saturation, high value
            # ✅ KALIBRERT fra 107 bilder (18M piksler) - oppdatert 20. mars 2026
            (np.array([0, 180, 149]), np.array([11, 255, 255])),
            (np.array([170, 180, 149]), np.array([179, 255, 255])),

            # Medium red (medium lys) - medium saturation/value
            (np.array([0, 150, 114]), np.array([11, 255, 255])),
            (np.array([170, 150, 114]), np.array([179, 255, 255])),

            # Dark red (dårlig lys) - low value for mørke forhold
            (np.array([0, 130, 99]), np.array([11, 255, 175])),
            (np.array([170, 130, 99]), np.array([179, 255, 175])),
        ]
        
        # Multi-range HSV thresholds for BLUE
        # Optimized for reliable blue ball detection under various lighting
        self.blue_ranges = [
            # Bright blue
            (np.array([100, 100, 100]), np.array([130, 255, 255])),
            
            # Medium blue
            (np.array([95, 70, 70]), np.array([135, 255, 255])),
            
            # Dark blue
            (np.array([100, 80, 40]), np.array([130, 255, 150])),
        ]
        
        # Morphological kernels for noise reduction
        self.morph_kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.morph_kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        # CLAHE opprettes én gang (ikke per frame) for ytelse
        self.clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        
        # Statistics
        self.stats = {
            'hsv_detections': 0,
            'hough_detections': 0,
            'ensemble_detections': 0,
            'lighting_level': 'unknown'
        }
    
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
        
        # Estimer lux-nivå basert på mean brightness
        # Antakelse: 300 lux ≈ 80 brightness, 700 lux ≈ 180 brightness (kalibrert for typisk innendørslys)
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
    
    def detect_with_hsv_multirange(self, hsv: np.ndarray, lighting_info: Optional[Dict] = None) -> Tuple[List[DetectedBall], List[DetectedBall]]:
        """
        Detekterer baller med multi-range HSV (med adaptiv justering).
        
        Args:
            hsv: Frame i HSV color space
            lighting_info: Optional lysanalyse for adaptiv justering
            
        Returns:
            Tuple med (red_balls, blue_balls)
        """
        red_balls = []
        blue_balls = []
        
        # Få adaptive ranges hvis tilgjengelig
        if lighting_info and self.enable_adaptive_lighting:
            red_ranges, blue_ranges = self.get_adaptive_hsv_ranges(lighting_info)
        else:
            red_ranges = self.red_ranges
            blue_ranges = self.blue_ranges
        
        # RØD: Kombiner alle ranges
        red_masks = []
        for lower, upper in red_ranges:
            mask = cv2.inRange(hsv, lower, upper)
            red_masks.append(mask)
        
        # Kombiner alle red masks med OR
        red_mask_combined = red_masks[0]
        for mask in red_masks[1:]:
            red_mask_combined = cv2.bitwise_or(red_mask_combined, mask)
        
        # Morfologiske operasjoner for å fjerne støy
        red_mask_combined = cv2.morphologyEx(red_mask_combined, cv2.MORPH_OPEN, self.morph_kernel_small)
        red_mask_combined = cv2.morphologyEx(red_mask_combined, cv2.MORPH_CLOSE, self.morph_kernel_large)
        
        # Finn contours
        red_contours, _ = cv2.findContours(red_mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in red_contours:
            ball = self._validate_contour(contour, BallColor.RED, "hsv")
            if ball:
                red_balls.append(ball)
        
        # BLÅ: Samme prosess
        blue_masks = []
        for lower, upper in blue_ranges:
            mask = cv2.inRange(hsv, lower, upper)
            blue_masks.append(mask)
        
        blue_mask_combined = blue_masks[0]
        for mask in blue_masks[1:]:
            blue_mask_combined = cv2.bitwise_or(blue_mask_combined, mask)
        
        blue_mask_combined = cv2.morphologyEx(blue_mask_combined, cv2.MORPH_OPEN, self.morph_kernel_small)
        blue_mask_combined = cv2.morphologyEx(blue_mask_combined, cv2.MORPH_CLOSE, self.morph_kernel_large)
        
        blue_contours, _ = cv2.findContours(blue_mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in blue_contours:
            ball = self._validate_contour(contour, BallColor.BLUE, "hsv")
            if ball:
                blue_balls.append(ball)
        
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
            minDist=max(100, gray.shape[1] // 8),  # Minst 1/8 av bildebredden mellom sirkler
            param1=60,   # Canny edge threshold (høyere = sterkere kanter kreves)
            param2=55,   # Akkumulatortreshold (høyere = færre falske sirkler)
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
    
    def _validate_contour(self, contour: np.ndarray, color: BallColor, method: str) -> Optional[DetectedBall]:
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
        
        # Krav: minst 55% sirkulær (baller er runde, ikke firkanter/striper)
        if circularity < 0.55:
            return None
        
        # Aspect ratio check - bounding box bør være tilnærmet kvadratisk
        bx, by, bw, bh = cv2.boundingRect(contour)
        aspect_ratio = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
        if aspect_ratio < 0.65:
            return None
        
        # Soliditet check - contour-areal / konveks skrog-areal
        # En ball er konveks (~1.0), lange smale former har lav soliditet
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = area / hull_area
            if solidity < 0.75:
                return None
        
        # Confidence basert på alle shape-mål
        ideal_area = np.pi * (radius ** 2)
        area_match = min(area / ideal_area, ideal_area / area)
        confidence = (circularity * 0.5) + (area_match * 0.3) + (aspect_ratio * 0.2)

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
                # Merge hvis senter til én sirkel er innenfor radius til den andre
                if dist < max(b1.radius, b2.radius):
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
                if dist < max(accepted.radius, candidate.radius) * 1.5:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(candidate)

        return kept

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
        
        # 1. Analyser lysforhold (300-700 lux range)
        lighting_info = self.analyze_lighting(frame)
        self.stats['lighting_level'] = lighting_info['level']
        
        # 2. Appliser lyskompensasjon hvis nødvendig
        compensated_frame = self.apply_lighting_compensation(frame, lighting_info)
        
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
        
        # 9. Behold maks N baller per farge (sortert etter confidence)
        merged_balls = self._limit_per_color(merged_balls)
        
        return merged_balls, self.stats.copy()
    
    def draw_detections(self, frame: np.ndarray, balls: List[DetectedBall], show_info: bool = True,
                        overlay: Optional[Dict] = None) -> np.ndarray:
        """
        Tegner detekterte baller og valgfri overlay-statistikk på frame.
        
        Args:
            frame: Frame å tegne på
            balls: Liste med baller
            show_info: Vis info-tekst ved siden av ballene
            overlay: Valgfri dict med ekstra nøkkel-verdi-par å vise øverst til venstre
                     Eksempel: {"FPS": 15, "Frame": 42}
            
        Returns:
            Annotated frame
        """
        output = frame.copy()
        
        for ball in balls:
            # Farge basert på ballens farge
            if ball.color == BallColor.RED:
                draw_color = (0, 0, 255)
                color_name = "RED"
            elif ball.color == BallColor.BLUE:
                draw_color = (255, 0, 0)
                color_name = "BLUE"
            else:
                draw_color = (128, 128, 128)
                color_name = "???"
            
            # Sirkel med sort kontur for å skille seg fra bakgrunnen
            cv2.circle(output, ball.center, int(ball.radius), (0, 0, 0), 6)
            cv2.circle(output, ball.center, int(ball.radius), draw_color, 3)
            cv2.circle(output, ball.center, 6, (0, 0, 0), -1)
            cv2.circle(output, ball.center, 4, (255, 255, 255), -1)

            if show_info:
                # Info-tekst med avstand — hvit tekst på mørk boks med farget kant
                dist_text = f"{ball.distance_cm:.0f} cm" if ball.distance_cm else ""
                conf_pct = int(ball.confidence * 100)
                text = f"{color_name}  {dist_text}  {conf_pct}%"

                FONT       = cv2.FONT_HERSHEY_SIMPLEX
                f_scale    = 0.58
                f_thick    = 1
                (tw, th), baseline = cv2.getTextSize(text, FONT, f_scale, f_thick)
                cx, cy = ball.center
                r = int(ball.radius)

                # Plasser tekst sentrert over sirkelen, klem til rammen
                xt = max(6, min(cx - tw // 2, output.shape[1] - tw - 10))
                yt = max(th + 10, cy - r - 10)

                pad = 5
                # Mørk boks
                cv2.rectangle(output,
                              (xt - pad, yt - th - pad),
                              (xt + tw + pad, yt + baseline + pad),
                              (20, 20, 20), -1)
                # Farget kant rundt boksen
                cv2.rectangle(output,
                              (xt - pad, yt - th - pad),
                              (xt + tw + pad, yt + baseline + pad),
                              draw_color, 1)
                # Hvit tekst
                cv2.putText(output, text, (xt, yt), FONT, f_scale, (255, 255, 255), f_thick)
        
        if not show_info:
            return output
        
        # --- Overlay panel øverst til venstre ---
        FONT       = cv2.FONT_HERSHEY_SIMPLEX
        FONT_SCALE = 0.65
        THICKNESS  = 1
        LINE_H     = 30        # piksel-avstand mellom linjer
        PAD_X      = 12
        PAD_TOP    = 10        # avstand fra topp til første linje
        
        WHITE = (255, 255, 255)
        lines = []  # (tekst, BGR-farge)
        
        # Lysnivå
        if self.enable_adaptive_lighting:
            level = self.stats.get('lighting_level', 'unknown')
            if level == 'low':
                lines.append(("Light: LOW (300-400 lux)",    WHITE))
            elif level == 'medium':
                lines.append(("Light: MEDIUM (400-550 lux)", WHITE))
            elif level == 'high':
                lines.append(("Light: HIGH (550-700 lux)",   WHITE))
            else:
                lines.append(("Light: UNKNOWN",               WHITE))
        
        # Ekstra overlay fra kallende kode (FPS, frame, osv.) — alltid hvit tekst
        if overlay:
            for key, val in overlay.items():
                lines.append((f"{key}: {val}", WHITE))
        
        if not lines:
            return output

        # Beregn boks-størrelse
        max_w = max(cv2.getTextSize(t, FONT, FONT_SCALE, THICKNESS)[0][0] for t, _ in lines)
        box_w = PAD_X * 2 + max_w + 8
        box_h = PAD_TOP * 2 + LINE_H * len(lines)
        h_fr, w_fr = output.shape[:2]
        box_w = min(box_w, w_fr)
        box_h = min(box_h, h_fr)

        # Semi-transparent mørk bakgrunn (75% mørk, 25% original)
        roi = output[0:box_h, 0:box_w].copy()
        dark = np.full_like(roi, (15, 15, 15))
        cv2.addWeighted(dark, 0.78, roi, 0.22, 0, roi)
        output[0:box_h, 0:box_w] = roi

        # Tynn grå kant rundt panelet
        cv2.rectangle(output, (0, 0), (box_w - 1, box_h - 1), (80, 80, 80), 1)

        for i, (text, color) in enumerate(lines):
            y = PAD_TOP + LINE_H // 2 + LINE_H // 4 + i * LINE_H
            # Sort skygge bak teksten for lesbarhet
            cv2.putText(output, text, (PAD_X, y), FONT, FONT_SCALE, (0, 0, 0), THICKNESS + 2)
            # Farget tekst øverst
            cv2.putText(output, text, (PAD_X, y), FONT, FONT_SCALE, color, THICKNESS)

        return output
    
    def get_statistics(self) -> Dict[str, int]:
        """Returnerer detection statistics."""
        return self.stats.copy()


# Alias for backwards compatibility
EnhancedBallDetector = SimpleBallDetector
