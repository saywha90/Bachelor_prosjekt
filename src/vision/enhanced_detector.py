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
    
    def __init__(self,
                 min_radius: int = 10,
                 max_radius: int = 150,
                 confidence_threshold: float = 0.35,
                 enable_adaptive_lighting: bool = True):
        """
        Initialiserer forenklet detector.
        
        Args:
            min_radius: Minimum ball radius i piksler
            max_radius: Maximum ball radius i piksler
            confidence_threshold: Minimum confidence for å godkjenne deteksjon
            enable_adaptive_lighting: Aktiver adaptiv lyshåndtering for 300-700 lux
        """
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.confidence_threshold = confidence_threshold
        self.enable_adaptive_lighting = enable_adaptive_lighting
        
        # Multi-range HSV thresholds for RED
        # ✅ KALIBRERT basert på analyse av 18 bilder (34M piksler) av din røde ball
        # Hue: 0-11, Saturation: 147-255, Value: 59-255
        self.red_ranges = [
            # Bright red (godt lys) - high saturation, high value
            (np.array([0, 177, 150]), np.array([11, 255, 255])),
            (np.array([170, 177, 150]), np.array([179, 255, 255])),
            
            # Medium red (medium lys) - medium saturation/value
            (np.array([0, 157, 96]), np.array([11, 255, 255])),
            (np.array([170, 157, 96]), np.array([179, 255, 255])),
            
            # Dark red (dårlig lys) - low value for mørke forhold
            (np.array([0, 147, 59]), np.array([11, 255, 156])),
            (np.array([170, 147, 59]), np.array([179, 255, 156])),
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
            
            # Appliser CLAHE på L-kanalen
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            l = clahe.apply(l)
            
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
            minDist=30,  # Minimum avstand mellom sirkler
            param1=50,   # Canny edge detection threshold
            param2=30,   # Accumulator threshold (lower = more sensitive)
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
        
        # Beregn gjennomsnittlig Hue og Saturation
        mean_hue = np.mean(roi[:, :, 0])
        mean_sat = np.mean(roi[:, :, 1])
        
        # Klassifiser basert på Hue og Saturation
        if mean_sat < 40:  # Low saturation - trolig ikke en farget ball
            return BallColor.UNKNOWN
        
        # Rød: Hue 0-20 eller 160-179
        if (0 <= mean_hue <= 20) or (160 <= mean_hue <= 179):
            return BallColor.RED
        
        # Blå: Hue 95-135
        if 95 <= mean_hue <= 135:
            return BallColor.BLUE
        
        return BallColor.UNKNOWN
    
    def _validate_contour(self, contour: np.ndarray, color: BallColor, method: str) -> Optional[DetectedBall]:
        """
        Validerer en contour og returnerer DetectedBall hvis valid.
        
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
        
        # Circularity check - hvor rund er formen?
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return None
        
        circularity = (4 * np.pi * area) / (perimeter ** 2)
        
        # Må være relativt rund (0.4 = 40% av perfekt sirkel)
        if circularity < 0.4:
            return None
        
        # Confidence basert på circularity og area match
        ideal_area = np.pi * (radius ** 2)
        area_match = min(area / ideal_area, ideal_area / area)
        confidence = (circularity * 0.6) + (area_match * 0.4)
        
        return DetectedBall(
            color=color,
            center=center,
            radius=float(radius),
            confidence=float(confidence),
            detection_method=method
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
        
        # Cluster overlappende deteksjoner
        clusters = []
        used = set()
        
        for i, ball1 in enumerate(all_detections):
            if i in used:
                continue
            
            cluster = [ball1]
            used.add(i)
            
            for j, ball2 in enumerate(all_detections):
                if j in used or j == i:
                    continue
                
                # Sjekk overlap
                dx = ball1.center[0] - ball2.center[0]
                dy = ball1.center[1] - ball2.center[1]
                dist = np.sqrt(dx**2 + dy**2)
                
                # Hvis sentre er innenfor combined radius * 0.7, er de samme ball
                combined_radius = (ball1.radius + ball2.radius) * 0.7
                
                if dist < combined_radius:
                    cluster.append(ball2)
                    used.add(j)
            
            clusters.append(cluster)
        
        # Merge hver cluster
        merged_balls = []
        
        for cluster in clusters:
            # Beregn gjennomsnittlig posisjon og radius
            avg_x = int(np.mean([b.center[0] for b in cluster]))
            avg_y = int(np.mean([b.center[1] for b in cluster]))
            avg_radius = np.mean([b.radius for b in cluster])
            
            # Fargebestemmelse: Velg fargen fra deteksjonen med høyest confidence
            best_ball = max(cluster, key=lambda b: b.confidence)
            color = best_ball.color
            
            # Confidence boost hvis multiple metoder er enige
            num_methods = len(set(b.detection_method for b in cluster))
            avg_confidence = np.mean([b.confidence for b in cluster])
            
            # Boost confidence hvis flere metoder detekterte samme ball
            # Dette gjør deteksjonen mer pålitelig
            final_confidence = min(avg_confidence * (1.0 + 0.3 * (num_methods - 1)), 1.0)
            
            merged_ball = DetectedBall(
                color=color,
                center=(avg_x, avg_y),
                radius=float(avg_radius),
                confidence=float(final_confidence),
                detection_method="ensemble"
            )
            
            merged_balls.append(merged_ball)
        
        self.stats['ensemble_detections'] = len(merged_balls)
        return merged_balls
    
    def detect_balls(self, frame: np.ndarray) -> List[DetectedBall]:
        """
        Hovedfunksjon for å detektere baller med ensemble pipeline og adaptiv lyshåndtering.
        
        Args:
            frame: Input frame i BGR
            
        Returns:
            Liste med detekterte baller
        """
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
        
        return merged_balls
    
    def draw_detections(self, frame: np.ndarray, balls: List[DetectedBall], show_info: bool = True) -> np.ndarray:
        """
        Tegner detekterte baller på frame med lysinformasjon.
        
        Args:
            frame: Frame å tegne på
            balls: Liste med baller
            show_info: Vis info-tekst
            
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
            
            # Tegn sirkel
            cv2.circle(output, ball.center, int(ball.radius), draw_color, 3)
            cv2.circle(output, ball.center, 5, draw_color, -1)
            
            if show_info:
                # Info-tekst
                text = f"{color_name} C:{ball.confidence:.2f}"
                
                # Tegn tekst over ballen
                y_pos = ball.center[1] - int(ball.radius) - 10
                cv2.putText(output, text, (ball.center[0] - 50, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)
        
        # Vis lysinformasjon i øvre venstre hjørne
        if show_info and self.enable_adaptive_lighting:
            lighting_level = self.stats.get('lighting_level', 'unknown')
            
            # Fargekode basert på lysnivå
            if lighting_level == 'low':
                light_color = (0, 165, 255)  # Orange
                light_text = "Light: LOW (300-400 lux)"
            elif lighting_level == 'medium':
                light_color = (0, 255, 0)  # Grønn
                light_text = "Light: MEDIUM (400-550 lux)"
            elif lighting_level == 'high':
                light_color = (0, 255, 255)  # Gul
                light_text = "Light: HIGH (550-700 lux)"
            else:
                light_color = (128, 128, 128)  # Grå
                light_text = "Light: UNKNOWN"
            
            # Tegn bakgrunn for bedre lesbarhet
            cv2.rectangle(output, (5, 5), (280, 30), (0, 0, 0), -1)
            cv2.putText(output, light_text, (10, 23),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, light_color, 2)
        
        return output
    
    def get_statistics(self) -> Dict[str, int]:
        """Returnerer detection statistics."""
        return self.stats.copy()


# Alias for backwards compatibility
EnhancedBallDetector = SimpleBallDetector
