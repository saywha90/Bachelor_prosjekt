"""
Lighting Adaptation Module for Ball Detection
==============================================

Dette modulet inneholder funksjoner for å håndtere varierende lysforhold
som innelys, utelys, dagslys, kunstig lys, skygger og refleksjoner.

Author: Bachelor Project 2026 - Autonomia
"""

import cv2
import numpy as np
from typing import Tuple, Dict, Optional
from enum import Enum


class LightingCondition(Enum):
    """Enum for forskjellige lysforhold"""
    BRIGHT_DAYLIGHT = "bright_daylight"      # Sterkt dagslys, evt. direkte sollys
    NORMAL_INDOOR = "normal_indoor"          # Normal innendørs belysning
    DIM_INDOOR = "dim_indoor"                # Svakt innendørs lys
    MIXED_LIGHTING = "mixed_lighting"        # Blanding av naturlig og kunstig
    SHADOW_HEAVY = "shadow_heavy"            # Mye skygger
    UNKNOWN = "unknown"


class AdaptiveLightingHandler:
    """
    Håndterer adaptiv HSV-terskelverdi basert på lysforhold.
    
    Dette systemet analyserer bildet og justerer HSV-grenser dynamisk
    for å håndtere forskjellige lysforhold.
    """
    
    def __init__(self):
        """Initialiserer adaptiv lyshåndtering"""
        self.current_condition = LightingCondition.UNKNOWN
        self.brightness_history = []
        self.history_size = 30  # 30 frames historie
        
        # Base HSV-verdier (optimale forhold)
        self.base_hsv_ranges = {
            'red_low': {
                'h': (0, 10),
                's': (100, 255),
                'v': (100, 255)
            },
            'red_high': {
                'h': (170, 179),
                's': (100, 255),
                'v': (100, 255)
            },
            'blue': {
                'h': (100, 130),
                's': (100, 255),
                'v': (100, 255)
            }
        }
    
    def analyze_lighting(self, frame: np.ndarray) -> Tuple[LightingCondition, Dict]:
        """
        Analyserer lysforhold i et bilde.
        
        Denne funksjonen beregner flere metriske for å bestemme lysforhold:
        - Gjennomsnittlig lysstyrke (brightness)
        - Standardavvik (kontrast)
        - Histogram-analyse
        
        Args:
            frame: Input BGR-bilde
        
        Returns:
            Tuple med (LightingCondition, metrics dictionary)
        """
        # Konverter til grayscale for lysanalyse
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Beregn lysmetriske
        mean_brightness = np.mean(gray)
        std_brightness = np.std(gray)
        min_brightness = np.min(gray)
        max_brightness = np.max(gray)
        
        # Beregn histogram
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist = hist.flatten() / hist.sum()  # Normaliser
        
        # Beregn entropy (måler kompleksitet/variasjon)
        entropy = -np.sum(hist * np.log2(hist + 1e-10))
        
        # Legg til i historie
        self.brightness_history.append(mean_brightness)
        if len(self.brightness_history) > self.history_size:
            self.brightness_history.pop(0)
        
        # Bestem lysforhold basert på metriske
        condition = self._classify_lighting(
            mean_brightness, std_brightness, 
            min_brightness, max_brightness, entropy
        )
        
        metrics = {
            'mean_brightness': mean_brightness,
            'std_brightness': std_brightness,
            'min_brightness': min_brightness,
            'max_brightness': max_brightness,
            'contrast_ratio': max_brightness / (min_brightness + 1),
            'entropy': entropy,
            'brightness_stable': self._is_brightness_stable()
        }
        
        self.current_condition = condition
        return condition, metrics
    
    def _classify_lighting(self, mean_b: float, std_b: float, 
                           min_b: float, max_b: float, entropy: float) -> LightingCondition:
        """
        Klassifiserer lysforhold basert på metriske.
        
        Beslutningslogikk:
        - Sterkt lys: Mean > 180, høy kontrast
        - Normalt innelys: Mean 80-180, moderat kontrast
        - Svakt lys: Mean < 80
        - Mye skygger: Høy std (> 60), høy kontrast
        """
        contrast_ratio = max_b / (min_b + 1)
        
        # Sterkt dagslys / direkte sollys
        if mean_b > 180 and std_b > 50:
            return LightingCondition.BRIGHT_DAYLIGHT
        
        # Mye skygger
        elif std_b > 60 and contrast_ratio > 15:
            return LightingCondition.SHADOW_HEAVY
        
        # Svakt innelys
        elif mean_b < 80:
            return LightingCondition.DIM_INDOOR
        
        # Blandet belysning (høy variasjon)
        elif std_b > 45 and 80 <= mean_b <= 180:
            return LightingCondition.MIXED_LIGHTING
        
        # Normal innendørs belysning
        elif 80 <= mean_b <= 180 and std_b < 45:
            return LightingCondition.NORMAL_INDOOR
        
        return LightingCondition.UNKNOWN
    
    def _is_brightness_stable(self) -> bool:
        """Sjekker om lysstyrke har vært stabil de siste N frames"""
        if len(self.brightness_history) < 10:
            return False
        
        recent = self.brightness_history[-10:]
        variance = np.var(recent)
        return variance < 100  # Stabil hvis liten variasjon
    
    def get_adaptive_hsv_ranges(self, color_key: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returnerer adaptive HSV-grenser basert på nåværende lysforhold.
        
        Dette er kjernen i adaptiv lyshåndtering. Basert på detektert
        lysforhold justeres HSV-grenser for optimal deteksjon.
        
        Args:
            color_key: 'red_low', 'red_high', eller 'blue'
        
        Returns:
            Tuple med (lower_bound, upper_bound) som numpy arrays
        """
        base_range = self.base_hsv_ranges[color_key]
        
        # Start med base-verdier
        h_min, h_max = base_range['h']
        s_min, s_max = base_range['s']
        v_min, v_max = base_range['v']
        
        # Juster basert på lysforhold
        if self.current_condition == LightingCondition.BRIGHT_DAYLIGHT:
            # Sterkt lys: Øk minimum Value, kan senke Saturation litt
            v_min = max(120, v_min + 20)  # Lyse forhold
            s_min = max(70, s_min - 30)   # Tillat litt mindre metning (overexposure)
        
        elif self.current_condition == LightingCondition.DIM_INDOOR:
            # Svakt lys: Senk minimum Value drastisk, behold høy Saturation
            v_min = max(40, v_min - 60)   # Aksepter mørke forhold
            s_min = max(80, s_min - 20)   # Litt mer tolerant på saturation
        
        elif self.current_condition == LightingCondition.SHADOW_HEAVY:
            # Mye skygger: Bred V-range, streng på S
            v_min = max(50, v_min - 50)   # Aksepter skygger
            v_max = 255                    # Men tillat også lyse områder
            s_min = max(90, s_min - 10)   # Relativt streng på saturation
        
        elif self.current_condition == LightingCondition.MIXED_LIGHTING:
            # Blandet: Moderat tilpasning
            v_min = max(70, v_min - 30)
            s_min = max(80, s_min - 20)
        
        elif self.current_condition == LightingCondition.NORMAL_INDOOR:
            # Optimal: Bruk base-verdier (ingen endring)
            pass
        
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        
        return lower, upper
    
    def print_lighting_info(self, metrics: Dict):
        """Skriver ut leselig informasjon om lysforhold"""
        print("\n" + "="*60)
        print("LYSFORHOLD-ANALYSE")
        print("="*60)
        print(f"Tilstand: {self.current_condition.value}")
        print(f"Gjennomsnittlig lysstyrke: {metrics['mean_brightness']:.1f}")
        print(f"Standardavvik: {metrics['std_brightness']:.1f}")
        print(f"Kontrast-ratio: {metrics['contrast_ratio']:.1f}")
        print(f"Min/Max lysstyrke: {metrics['min_brightness']:.0f} / {metrics['max_brightness']:.0f}")
        print(f"Stabil lysstyrke: {'Ja' if metrics['brightness_stable'] else 'Nei'}")
        print("="*60 + "\n")


def apply_clahe(frame: np.ndarray, clip_limit: float = 2.0) -> np.ndarray:
    """
    Anvender CLAHE (Contrast Limited Adaptive Histogram Equalization).
    
    CLAHE forbedrer lokal kontrast og er spesielt nyttig i:
    - Ujevn belysning
    - Skygger og høylys samtidig
    - Svakt lys
    
    OBS: Bruk med forsiktighet - kan forsterke støy!
    
    Args:
        frame: Input BGR-bilde
        clip_limit: Begrensning på kontrastforsterkning (2.0 er moderat)
    
    Returns:
        Forbedret BGR-bilde
    """
    # Konverter til LAB fargerom (bedre for equalizing)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_channel, a, b = cv2.split(lab)
    
    # Anvend CLAHE på L-kanalen (lysstyrke)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
    l_channel = clahe.apply(l_channel)
    
    # Merge tilbake
    lab = cv2.merge([l_channel, a, b])
    enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    return enhanced


def apply_gamma_correction(frame: np.ndarray, gamma: float = 1.0) -> np.ndarray:
    """
    Anvender gamma-korreksjon for lysjustering.
    
    Gamma-korreksjon justerer mellomtoner uten å overexpose høylys:
    - gamma < 1.0: Lysere bilde (for mørke forhold)
    - gamma = 1.0: Ingen endring
    - gamma > 1.0: Mørkere bilde (for lyse forhold)
    
    Args:
        frame: Input BGR-bilde
        gamma: Gamma-verdi (typisk 0.5 - 2.0)
    
    Returns:
        Gamma-korrigert bilde
    """
    # Bygg lookup table
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 
                      for i in range(256)]).astype("uint8")
    
    # Anvend lookup table
    return cv2.LUT(frame, table)


def auto_adjust_brightness(frame: np.ndarray, target_mean: float = 128.0) -> np.ndarray:
    """
    Automatisk justering av lysstyrke til målverdi.
    
    Justerer bildet slik at gjennomsnittlig lysstyrke treffer target_mean.
    Nyttig for å normalisere bilder fra forskjellige lysforhold.
    
    Args:
        frame: Input BGR-bilde
        target_mean: Ønsket gjennomsnittlig lysstyrke (0-255)
    
    Returns:
        Justert bilde
    """
    # Konverter til HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # Beregn nåværende gjennomsnitt
    current_mean = np.mean(v)
    
    # Beregn justeringsfaktor
    if current_mean > 0:
        adjustment = target_mean / current_mean
        # Begrens justering for å unngå ekstreme endringer
        adjustment = np.clip(adjustment, 0.5, 2.0)
    else:
        adjustment = 1.0
    
    # Juster V-kanalen
    v_adjusted = np.clip(v * adjustment, 0, 255).astype(np.uint8)
    
    # Merge og konverter tilbake
    hsv_adjusted = cv2.merge([h, s, v_adjusted])
    adjusted = cv2.cvtColor(hsv_adjusted, cv2.COLOR_HSV2BGR)
    
    return adjusted


def normalize_lighting_with_bilateral_filter(frame: np.ndarray) -> np.ndarray:
    """
    Bruker bilateral filter for å jevne ut belysning mens kanter bevares.
    
    Bilateral filter er utmerket for:
    - Redusere lysvariasjoner
    - Bevare objektkanter (viktig for deteksjon)
    - Fjerne skygger
    
    Mer avansert enn Gaussisk blur fordi den bevarer kanter bedre.
    
    Args:
        frame: Input BGR-bilde
    
    Returns:
        Filtrert bilde
    """
    # Bilateral filter: jevner ut farger men bevarer kanter
    # d: Diameter of pixel neighborhood
    # sigmaColor: Filter sigma in the color space
    # sigmaSpace: Filter sigma in the coordinate space
    filtered = cv2.bilateralFilter(frame, d=9, sigmaColor=75, sigmaSpace=75)
    
    return filtered


if __name__ == "__main__":
    """Test av lyshåndtering"""
    print("Testing Adaptive Lighting Handler...")
    
    # Test med dummy frame
    test_frame = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
    
    handler = AdaptiveLightingHandler()
    condition, metrics = handler.analyze_lighting(test_frame)
    handler.print_lighting_info(metrics)
    
    # Test HSV-tilpasning
    print("\nAdaptive HSV Ranges:")
    for color in ['red_low', 'red_high', 'blue']:
        lower, upper = handler.get_adaptive_hsv_ranges(color)
        print(f"{color}: {lower} - {upper}")
    
    print("\n✓ Lighting adaptation module test complete!")
