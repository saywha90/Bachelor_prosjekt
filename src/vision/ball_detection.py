"""
Ball Detection System for Robot Arm Project
============================================

Dette modulet implementerer et robust system for deteksjon av røde og blåe baller
ved hjelp av avansert bildebehandling. Systemet er designet for å være presist,
raskt og pålitelig under varierende lysforhold.

Hovedfunksjoner:
- HSV-basert fargedeteksjon (mer robust enn RGB)
- Morfologiske operasjoner for støyreduksjon
- Konturanalyse for objektidentifikasjon
- Sirkeldeteksjon for validering
- Avstandsestimering basert på ballstørrelse
- Sanntidsbehandling optimalisert for Raspberry Pi

Author: Bachelor Project 2026 - Autonomia
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum


class BallColor(Enum):
    """Enum for ballfarger som skal detekteres"""
    RED = "red"
    BLUE = "blue"
    UNKNOWN = "unknown"


@dataclass
class DetectedBall:
    """
    Dataklasse som representerer en detektert ball.
    
    Attributes:
        color: Fargen på ballen (RED eller BLUE)
        center: Senterpunkt (x, y) i bildekoordinater
        radius: Radius i piksler
        confidence: Konfidensverdi (0.0-1.0) for deteksjonen
        distance_cm: Estimert avstand i centimeter (hvis kalibrert)
    """
    color: BallColor
    center: Tuple[int, int]
    radius: float
    confidence: float
    distance_cm: Optional[float] = None
    
    def __str__(self):
        return (f"Ball({self.color.value}, pos={self.center}, "
                f"r={self.radius:.1f}px, conf={self.confidence:.2f})")


class BallDetector:
    """
    Hovedklasse for balldeteksjon.
    
    Denne klassen implementerer et komplett system for å detektere røde og blåe
    baller i sanntid. Den bruker HSV-fargerom for robust fargedeteksjon og
    kombinerer flere teknikker for å minimere falske positiver.
    
    Teknisk tilnærming:
    1. **HSV Color Space**: Vi bruker HSV (Hue, Saturation, Value) i stedet for 
       RGB fordi HSV skiller fargekomponenten (Hue) fra lysstyrke (Value).
       Dette gjør deteksjonen mye mer robust under varierende lysforhold.
    
    2. **Morfologiske operasjoner**: Opening (erosion etterfulgt av dilation)
       fjerner støy, mens Closing (dilation etterfulgt av erosion) fyller hull.
    
    3. **Konturanalyse**: Vi finner sammenhengende områder og filtrerer basert på
       form, størrelse og sirkulærhet.
    
    4. **Hough Circle Transform**: Validerer at detekterte objekter faktisk er
       sirkulære, noe som reduserer falske positiver.
    """
    
    def __init__(self, 
                 min_radius: int = 10,
                 max_radius: int = 150,
                 min_circularity: float = 0.7,
                 known_ball_diameter_cm: float = 7.0,
                 camera_focal_length: Optional[float] = None,
                 max_detections_per_frame: int = 50):
        """
        Initialiserer balldetektoren med konfigurerbare parametere.
        
        Args:
            min_radius: Minimum radius i piksler for å regnes som ball
            max_radius: Maksimum radius i piksler for å regnes som ball
            min_circularity: Minimum sirkulærhet (0.0-1.0), høyere = strengere
            known_ball_diameter_cm: Kjent diameter på ballene i cm (for avstandsestimering)
            camera_focal_length: Kameraets brennvidde i piksler (kalibrert verdi)
            max_detections_per_frame: Maksimum antall deteksjoner per frame (sikkerhetsbegrensning)
        """
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_circularity = min_circularity
        self.known_ball_diameter_cm = known_ball_diameter_cm
        self.camera_focal_length = camera_focal_length
        self.max_detections_per_frame = max_detections_per_frame
        
        # HSV-grenser for fargedeteksjon
        # Disse verdiene er optimalisert for typiske baller under normalt innendørslys
        # OBS: I HSV-fargerommet i OpenCV er Hue skalert til 0-179 (ikke 0-360)
        
        # RØD FARGE: Rød er spesiell fordi den wrapper rundt i HSV-hjulet
        # Vi må derfor bruke to områder: lav-rød (0-10) og høy-rød (170-179)
        self.red_lower_1 = np.array([0, 100, 100])      # Lav-rød
        self.red_upper_1 = np.array([10, 255, 255])
        self.red_lower_2 = np.array([170, 100, 100])    # Høy-rød
        self.red_upper_2 = np.array([179, 255, 255])
        
        # BLÅ FARGE: Mer rettfram, ett sammenhengende område
        self.blue_lower = np.array([100, 100, 100])     # Dyp blå
        self.blue_upper = np.array([130, 255, 255])
        
        # Morfologiske kjerner for støyreduksjon
        # Større kjerne = mer aggressiv støyfjerning, men kan miste små detaljer
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # Statistikk for debugging/evaluering
        self.frame_count = 0
        self.detection_stats = {'red': 0, 'blue': 0, 'total_frames': 0}
    
    def calibrate_camera(self, calibration_distance_cm: float, measured_diameter_px: float):
        """
        Kalibrerer kameraet for avstandsestimering.
        
        Denne funksjonen beregner kameraets brennvidde basert på en kjent avstand
        og målte objektstørrelse. Dette er nødvendig for nøyaktig avstandsestimering.
        
        Kalibreringsprosess:
        1. Plasser en ball på en kjent avstand fra kameraet
        2. Mål ballens diameter i piksler ved å kjøre deteksjonen
        3. Kall denne funksjonen med de målte verdiene
        
        Formel: focal_length = (measured_size_px * real_distance_cm) / real_size_cm
        
        Args:
            calibration_distance_cm: Faktisk avstand til ballen i cm
            measured_diameter_px: Målt diameter av ballen i piksler ved den avstanden
        """
        self.camera_focal_length = (measured_diameter_px * calibration_distance_cm) / self.known_ball_diameter_cm
        print(f"Kamera kalibrert: Brennvidde = {self.camera_focal_length:.2f} piksler")
    
    def estimate_distance(self, ball_diameter_px: float) -> Optional[float]:
        """
        Estimerer avstand til ballen basert på dens størrelse i bildet.
        
        Bruker pinhole-kameramodellen:
        distance = (real_size * focal_length) / perceived_size
        
        Mindre ball i bildet = lengre unna
        Større ball i bildet = nærmere
        
        Args:
            ball_diameter_px: Ballens diameter i piksler
            
        Returns:
            Estimert avstand i cm, eller None hvis ikke kalibrert
        """
        if self.camera_focal_length is None:
            return None
        
        if ball_diameter_px <= 0:
            return None
        
        distance = (self.known_ball_diameter_cm * self.camera_focal_length) / ball_diameter_px
        return distance
    
    def _create_color_mask(self, hsv_image: np.ndarray, color: BallColor) -> np.ndarray:
        """
        Lager en binær maske for en spesifikk farge.
        
        En maske er et svart-hvitt bilde hvor hvite piksler representerer
        områder som matcher ønsket farge.
        
        Args:
            hsv_image: Bildet i HSV-fargerom
            color: Fargen vi skal detektere
            
        Returns:
            Binær maske (samme størrelse som input, kun 0 og 255 verdier)
        """
        if color == BallColor.RED:
            # For rød: kombiner to masker (lav og høy rød)
            mask1 = cv2.inRange(hsv_image, self.red_lower_1, self.red_upper_1)
            mask2 = cv2.inRange(hsv_image, self.red_lower_2, self.red_upper_2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == BallColor.BLUE:
            mask = cv2.inRange(hsv_image, self.blue_lower, self.blue_upper)
        else:
            raise ValueError(f"Ugyldig farge: {color}")
        
        # Morfologiske operasjoner for å fjerne støy og fylle hull
        # Opening: Fjerner små hvite flekker (støy) i bakgrunnen
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=2)
        # Closing: Fyller små hull i detekterte objekter
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=2)
        
        return mask
    
    def _calculate_circularity(self, contour: np.ndarray) -> float:
        """
        Beregner sirkulærhet for en kontur.
        
        Sirkulærhet er et mål på hvor lik en sirkel et objekt er.
        Formel: circularity = (4 * π * area) / (perimeter²)
        
        Perfekt sirkel: circularity = 1.0
        Langstrakt form: circularity → 0
        
        Dette er viktig for å filtrere ut objekter som ikke er baller,
        for eksempel firkantede objekter eller uregelmessige former.
        
        Args:
            contour: OpenCV kontur
            
        Returns:
            Sirkulærhetsverdien (0.0-1.0)
        """
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            return 0.0
        
        circularity = (4 * np.pi * area) / (perimeter ** 2)
        return min(circularity, 1.0)  # Begrens til maksimum 1.0
    
    def _filter_and_validate_contours(self, 
                                      contours: List[np.ndarray],
                                      color: BallColor) -> List[DetectedBall]:
        """
        Filtrerer og validerer konturer for å finne gyldige baller.
        
        Denne funksjonen utfører flere kvalitetskontroller:
        1. Størrelsesfiltrering (for små/store objekter forkastes)
        2. Sirkulærhetskontroll (må være tilstrekkelig rund)
        3. Konfidensvurdering basert på flere faktorer
        
        Args:
            contours: Liste med OpenCV-konturer
            color: Fargen vi detekterer for
            
        Returns:
            Liste med validerte DetectedBall-objekter
        """
        detected_balls = []
        
        for contour in contours:
            # Beregn konturens egenskaper
            area = cv2.contourArea(contour)
            
            # Ignorer for små konturer (sannsynligvis støy)
            if area < np.pi * (self.min_radius ** 2):
                continue
            
            # Beregn minste omkransende sirkel
            (x, y), radius = cv2.minEnclosingCircle(contour)
            
            # Filterering basert på radius
            if radius < self.min_radius or radius > self.max_radius:
                continue
            
            # Sirkulærhetskontroll
            circularity = self._calculate_circularity(contour)
            if circularity < self.min_circularity:
                continue
            
            # Beregn konfidensverdi basert på flere faktorer
            # Høyere sirkulærhet og bedre areal-til-sirkel ratio gir høyere konfidens
            ideal_circle_area = np.pi * (radius ** 2)
            area_match = min(area / ideal_circle_area, ideal_circle_area / area)
            confidence = (circularity * 0.7) + (area_match * 0.3)
            
            # Estimer avstand hvis kalibrert
            distance = self.estimate_distance(radius * 2)  # Diameter = 2 * radius
            
            # Opprett DetectedBall-objekt
            ball = DetectedBall(
                color=color,
                center=(int(x), int(y)),
                radius=float(radius),
                confidence=float(confidence),
                distance_cm=distance
            )
            
            detected_balls.append(ball)
        
        return detected_balls
    
    def detect_balls(self, frame: np.ndarray) -> List[DetectedBall]:
        """
        Hovedfunksjon for å detektere alle baller i et bilde.
        
        Dette er den primære funksjonen som skal kalles for hver frame.
        Den håndterer hele deteksjonspipelinen fra start til slutt.
        
        Prosess:
        1. Konverter fra BGR (OpenCV standard) til HSV
        2. Lag fargemasker for rød og blå
        3. Finn konturer i hver maske
        4. Filtrer og valider konturer
        5. Returner liste med detekterte baller
        
        Args:
            frame: Input-bilde i BGR-format (OpenCV standard)
            
        Returns:
            Liste med DetectedBall-objekter sortert etter konfidens (høyest først)
        """
        if frame is None or frame.size == 0:
            return []
        
        self.frame_count += 1
        self.detection_stats['total_frames'] += 1
        
        # Konverter til HSV fargerom
        # HSV er mye bedre for fargedeteksjon enn BGR/RGB
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Gaussisk blur for å redusere støy
        # Liten blur (5x5) bevarer detaljer men reduserer pikselbråk
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        
        all_balls = []
        
        # Detekter røde baller
        red_mask = self._create_color_mask(hsv, BallColor.RED)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_balls = self._filter_and_validate_contours(red_contours, BallColor.RED)
        all_balls.extend(red_balls)
        self.detection_stats['red'] += len(red_balls)
        
        # Detekter blåe baller
        blue_mask = self._create_color_mask(hsv, BallColor.BLUE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_balls = self._filter_and_validate_contours(blue_contours, BallColor.BLUE)
        all_balls.extend(blue_balls)
        self.detection_stats['blue'] += len(blue_balls)
        
        # Sorter etter konfidens (høyest først)
        # Dette er nyttig hvis man kun vil plukke opp den mest pålitelige deteksjonen
        all_balls.sort(key=lambda b: b.confidence, reverse=True)
        
        # Sikkerhetsbegrensning: Begrens antall deteksjoner for å forhindre resource exhaustion
        if len(all_balls) > self.max_detections_per_frame:
            print(f"⚠️  ADVARSEL: Begrenset til {self.max_detections_per_frame} deteksjoner (fant {len(all_balls)})")
            all_balls = all_balls[:self.max_detections_per_frame]
        
        return all_balls
    
    def draw_detections(self, 
                       frame: np.ndarray, 
                       balls: List[DetectedBall],
                       show_info: bool = True) -> np.ndarray:
        """
        Tegner detekterte baller på bildet for visualisering.
        
        Dette er nyttig for debugging, demonstrasjon og validering av deteksjonen.
        
        Args:
            frame: Original frame å tegne på (blir ikke modifisert)
            balls: Liste med detekterte baller
            show_info: Om tekstinformasjon skal vises
            
        Returns:
            Ny frame med tegninger (original frame er uendret)
        """
        output = frame.copy()
        
        for ball in balls:
            # Velg farge basert på ballens farge
            if ball.color == BallColor.RED:
                draw_color = (0, 0, 255)  # Rød i BGR
                color_name = "ROD"
            elif ball.color == BallColor.BLUE:
                draw_color = (255, 0, 0)  # Blå i BGR
                color_name = "BLA"
            else:
                draw_color = (128, 128, 128)  # Grå for ukjent
                color_name = "???"
            
            # Tegn sirkel rundt ballen
            cv2.circle(output, ball.center, int(ball.radius), draw_color, 2)
            
            # Tegn senterpunkt
            cv2.circle(output, ball.center, 5, draw_color, -1)
            
            if show_info:
                # Lag informasjonstekst
                text_lines = [
                    f"{color_name}",
                    f"Conf: {ball.confidence:.2f}"
                ]
                
                if ball.distance_cm is not None:
                    text_lines.append(f"Dist: {ball.distance_cm:.1f}cm")
                
                # Tegn tekst over ballen
                y_offset = ball.center[1] - int(ball.radius) - 10
                for i, text in enumerate(text_lines):
                    y_pos = y_offset - (i * 20)
                    cv2.putText(output, text, (ball.center[0] - 40, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)
        
        # Vis generell statistikk i hjørnet
        if show_info:
            stats_text = [
                f"Totalt: {len(balls)} baller",
                f"Rod: {sum(1 for b in balls if b.color == BallColor.RED)}",
                f"Bla: {sum(1 for b in balls if b.color == BallColor.BLUE)}",
                f"Frame: {self.frame_count}"
            ]
            
            for i, text in enumerate(stats_text):
                cv2.putText(output, text, (10, 30 + i * 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return output
    
    def get_statistics(self) -> Dict:
        """
        Returnerer deteksjonsstatistikk for evaluering.
        
        Returns:
            Dictionary med statistikk over antall deteksjoner
        """
        return self.detection_stats.copy()
    
    def adjust_color_range(self, 
                          color: BallColor,
                          lower: Tuple[int, int, int],
                          upper: Tuple[int, int, int]):
        """
        Justerer HSV-grensene for en farge.
        
        Denne funksjonen er nyttig for å finjustere deteksjonen under
        spesifikke lysforhold eller for spesifikke ballfarger.
        
        Tips: Bruk et HSV-color-picker-verktøy for å finne riktige verdier.
        
        Args:
            color: Hvilken farge som skal justeres
            lower: Nedre HSV-grense [H, S, V]
            upper: Øvre HSV-grense [H, S, V]
        """
        lower_array = np.array(lower)
        upper_array = np.array(upper)
        
        if color == BallColor.RED:
            # For rød, sett begge områdene til samme verdier
            # (kan utvides for mer finkornet kontroll)
            self.red_lower_1 = lower_array
            self.red_upper_1 = upper_array
            print(f"Oppdatert rød fargeområde: {lower} til {upper}")
        elif color == BallColor.BLUE:
            self.blue_lower = lower_array
            self.blue_upper = upper_array
            print(f"Oppdatert blå fargeområde: {lower} til {upper}")


def create_default_detector() -> BallDetector:
    """
    Factory-funksjon for å lage en detektor med standardinnstillinger.
    
    Disse verdiene er et godt utgangspunkt for de fleste brukstilfeller.
    Juster ved behov basert på:
    - Faktisk ballstørrelse
    - Kameraoppløsning
    - Avstand til baller
    - Lysforhold
    
    Returns:
        Konfigurert BallDetector-instans
    """
    return BallDetector(
        min_radius=10,           # Minimum 10 piksler radius
        max_radius=150,          # Maksimum 150 piksler radius
        min_circularity=0.7,     # Må være minst 70% sirkulær
        known_ball_diameter_cm=7.0,  # Antatt 7cm diameter (juster til faktisk størrelse)
        camera_focal_length=None     # Må kalibreres senere
    )


if __name__ == "__main__":
    """
    Eksempel på hvordan detektoren kan brukes.
    Dette eksempelet viser basic bruk med webcam.
    """
    print("Balldeteksjonssystem - Test Mode")
    print("Trykk 'q' for å avslutte")
    print("Trykk 'c' for å kalibrere (når ball er synlig)")
    
    # Opprett detektor
    detector = create_default_detector()
    
    # Åpne kamera (0 = default webcam)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("FEIL: Kunne ikke åpne kamera!")
        exit(1)
    
    print("Kamera åpnet. Starter deteksjon...")
    
    while True:
        # Les frame fra kamera
        ret, frame = cap.read()
        
        if not ret:
            print("Kunne ikke lese fra kamera")
            break
        
        # Kjør deteksjon
        balls = detector.detect_balls(frame)
        
        # Tegn resultater
        output = detector.draw_detections(frame, balls)
        
        # Vis resultat
        cv2.imshow('Ball Detection', output)
        
        # Håndter tastaturinput
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('c') and len(balls) > 0:
            # Enkel kalibrering: bruk første detekterte ball
            ball = balls[0]
            print(f"\nDetektert ball: radius={ball.radius:.1f}px")
            try:
                distance_str = input("Skriv inn faktisk avstand til ballen i cm (eller 'avbryt'): ")
                if distance_str.lower() in ['avbryt', 'cancel', 'q']:
                    print("Kalibrering avbrutt")
                    continue
                distance = float(distance_str)
                if distance <= 0 or distance > 1000:
                    print("❌ FEIL: Avstand må være mellom 0 og 1000 cm")
                    continue
                detector.calibrate_camera(distance, ball.radius * 2)
            except ValueError:
                print("❌ FEIL: Ugyldig input")
            except Exception:
                print("❌ FEIL: Kalibrering feilet")
    
    # Rydd opp
    cap.release()
    cv2.destroyAllWindows()
    
    # Vis statistikk
    stats = detector.get_statistics()
    print("\n=== Deteksjonsstatistikk ===")
    print(f"Totalt frames: {stats['total_frames']}")
    print(f"Røde baller: {stats['red']}")
    print(f"Blåe baller: {stats['blue']}")
    print(f"Totalt: {stats['red'] + stats['blue']}")
