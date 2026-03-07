"""
Test Script for Ball Detection System
======================================

Dette scriptet demonstrerer hvordan du bruker balldeteksjonssystemet
og lar deg finjustere parametere i sanntid.

Tastaturkommandoer:
  q - Avslutt
  c - Kalibrer avstandsestimering
  s - Lagre gjeldende innstillinger
  r - Vis/skjul måltavle for kalibrering
  1 - Vis original frame
  2 - Vis rød maske
  3 - Vis blå maske
  4 - Vis kombinert maske
  h - Vis hjelpemeny

Author: Bachelor Project 2026 - Autonomia
"""

import cv2
import sys
import os
import numpy as np
from pathlib import Path

# Legg til src-mappen i path for å kunne importere moduler
src_path = Path(__file__).parent.parent
sys.path.insert(0, str(src_path))

from vision.ball_detection import (
    BallDetector, 
    create_default_detector, 
    BallColor,
    DetectedBall
)
from vision.privacy_utils import (
    request_camera_consent,
    get_validated_float_input,
    validate_camera_index
)


class BallDetectionTester:
    """Interaktiv test-klasse for balldeteksjon"""
    
    def __init__(self, camera_index=0):
        """
        Args:
            camera_index: Indeks for kamera (0 = standard webcam)
        """
        self.detector = create_default_detector()
        self.camera_index = camera_index
        self.cap = None
        
        # Display-innstillinger
        self.display_mode = 'normal'  # 'normal', 'red_mask', 'blue_mask', 'combined_mask'
        self.show_calibration_grid = False
        
        # Statistikk
        self.total_frames = 0
        self.total_detections = 0
        
    def initialize_camera(self):
        """Åpner og konfigurerer kameraet"""
        # GDPR: Be om samtykke før kamerabruk
        if not request_camera_consent():
            print("\n❌ Kamerasamtykke ikke gitt. Avslutter.")
            return False
        
        print(f"\nÅpner kamera {self.camera_index}...")
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"FEIL: Kunne ikke åpne kamera {self.camera_index}")
            print("Tips:")
            print("  - Sjekk at kamera er tilkoblet")
            print("  - Prøv en annen kameraindeks (0, 1, 2, ...)")
            print("  - Lukk andre programmer som bruker kameraet")
            return False
        
        # Sett oppløsning (640x480 er godt balansert for ytelse)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Les faktisk oppløsning
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        print(f"✓ Kamera åpnet: {width}x{height} @ {fps} FPS")
        return True
    
    def draw_calibration_grid(self, frame):
        """Tegner et rutenett for kalibrering"""
        h, w = frame.shape[:2]
        
        # Vertikale linjer
        for x in range(0, w, 50):
            cv2.line(frame, (x, 0), (x, h), (0, 255, 0), 1)
        
        # Horisontale linjer
        for y in range(0, h, 50):
            cv2.line(frame, (0, y), (w, y), (0, 255, 0), 1)
        
        # Sentermerke
        cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 255), 2)
        cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 255), 2)
        
        return frame
    
    def create_mask_visualization(self, frame):
        """Lager visualisering av fargemasker"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        
        # Lag masker
        red_mask = self.detector._create_color_mask(hsv, BallColor.RED)
        blue_mask = self.detector._create_color_mask(hsv, BallColor.BLUE)
        
        if self.display_mode == 'red_mask':
            return cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
        elif self.display_mode == 'blue_mask':
            return cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)
        elif self.display_mode == 'combined_mask':
            # Kombiner masker med forskjellige farger
            combined = cv2.merge([
                blue_mask,
                np.zeros_like(red_mask),
                red_mask
            ])
            return combined
        
        return frame
    
    def calibrate_camera_interactive(self, balls):
        """Interaktiv kalibrering av kamera"""
        if not balls:
            print("Ingen baller funnet! Plasser en ball i bildet først.")
            return
        
        # Bruk ballen med høyest konfidens
        ball = balls[0]
        
        print("\n" + "="*50)
        print("KAMERAKALIBRERING")
        print("="*50)
        print(f"Detektert ball: {ball.color.value}")
        print(f"Radius: {ball.radius:.1f} piksler")
        print(f"Diameter: {ball.radius * 2:.1f} piksler")
        print()
        print("Instruksjoner:")
        print("1. Mål nøyaktig avstand fra kamera til ball")
        print("2. Skriv inn avstanden i cm nedenfor")
        print()
        
        # Bruk sikker input-validering
        distance = get_validated_float_input(
            "Avstand til ball (cm)",
            min_value=5.0,
            max_value=500.0,
            allow_cancel=True
        )
        
        if distance is None:
            print("Kalibrering avbrutt")
            return
        
        try:
            
            # Utfør kalibrering
            self.detector.calibrate_camera(distance, ball.radius * 2)
            print(f"✓ Kalibrering fullført!")
            print(f"  Brennvidde: {self.detector.camera_focal_length:.2f} piksler")
            print()
            
        except Exception as e:
            print(f"❌ FEIL under kalibrering. Prøv igjen.")
            # Log detaljert feil for debugging (ikke vis til bruker)
            import logging
            logging.error(f"Kalibreringsfeil: {e}")
    
    def print_controls(self):
        """Skriver ut kontroller"""
        print("\n" + "="*60)
        print("BALL DETECTION TEST - KONTROLLER")
        print("="*60)
        print("  q   - Avslutt programmet")
        print("  c   - Kalibrer kamera (med synlig ball)")
        print("  r   - Vis/skjul kalibreringsnett")
        print("  1   - Normal visning")
        print("  2   - Vis rød maske")
        print("  3   - Vis blå maske")
        print("  4   - Vis kombinert maske")
        print("  h   - Vis denne hjelpemenyen")
        print("  s   - Lagre innstillinger (kommer)")
        print("="*60 + "\n")
    
    def run(self):
        """Hovedløkke for testing"""
        if not self.initialize_camera():
            return
        
        print("\n" + "="*60)
        print("BALL DETECTION SYSTEM - INTERAKTIV TEST")
        print("="*60)
        print("Systemet er klart. Trykk 'h' for kontroller.")
        print()
        
        window_name = 'Ball Detection Test'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        try:
            while True:
                # Les frame
                ret, frame = self.cap.read()
                
                if not ret:
                    print("FEIL: Kunne ikke lese fra kamera")
                    break
                
                self.total_frames += 1
                
                # Kjør deteksjon
                balls = self.detector.detect_balls(frame)
                self.total_detections += len(balls)
                
                # Vis forskjellige visninger
                if self.display_mode != 'normal':
                    display_frame = self.create_mask_visualization(frame)
                else:
                    display_frame = self.detector.draw_detections(frame, balls)
                
                # Tegn kalibreringsnett hvis aktivert
                if self.show_calibration_grid:
                    display_frame = self.draw_calibration_grid(display_frame)
                
                # Legg til display-mode info
                mode_text = {
                    'normal': 'Normal',
                    'red_mask': 'Rød Maske',
                    'blue_mask': 'Blå Maske',
                    'combined_mask': 'Kombinert Maske'
                }[self.display_mode]
                
                cv2.putText(display_frame, f"Mode: {mode_text}", (10, display_frame.shape[0] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Vis frame
                cv2.imshow(window_name, display_frame)
                
                # Håndter tastaturinput
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("\nAvslutter...")
                    break
                elif key == ord('h'):
                    self.print_controls()
                elif key == ord('c'):
                    self.calibrate_camera_interactive(balls)
                elif key == ord('r'):
                    self.show_calibration_grid = not self.show_calibration_grid
                    status = "PÅ" if self.show_calibration_grid else "AV"
                    print(f"Kalibreringsnett: {status}")
                elif key == ord('1'):
                    self.display_mode = 'normal'
                    print("Visning: Normal")
                elif key == ord('2'):
                    self.display_mode = 'red_mask'
                    print("Visning: Rød maske")
                elif key == ord('3'):
                    self.display_mode = 'blue_mask'
                    print("Visning: Blå maske")
                elif key == ord('4'):
                    self.display_mode = 'combined_mask'
                    print("Visning: Kombinert maske")
                elif key == ord('s'):
                    print("Lagring av innstillinger er ikke implementert ennå")
        
        except KeyboardInterrupt:
            print("\n\nAvbrutt av bruker (Ctrl+C)")
        
        finally:
            # Rydd opp
            self.cleanup()
    
    def cleanup(self):
        """Rydd opp ressurser"""
        print("\nRydder opp...")
        
        if self.cap is not None:
            self.cap.release()
        
        cv2.destroyAllWindows()
        
        # Vis statistikk
        print("\n" + "="*60)
        print("STATISTIKK")
        print("="*60)
        stats = self.detector.get_statistics()
        print(f"Totalt frames prosessert: {self.total_frames}")
        print(f"Totalt deteksjoner: {self.total_detections}")
        print(f"  Røde baller: {stats['red']}")
        print(f"  Blåe baller: {stats['blue']}")
        
        if self.total_frames > 0:
            avg_detection = self.total_detections / self.total_frames
            print(f"Gjennomsnitt per frame: {avg_detection:.2f}")
        
        print("="*60)
        print("Takk for at du testet systemet!")
        print("="*60 + "\n")


def main():
    """Hovedfunksjon"""
    import numpy as np
    
    print("="*60)
    print("BALL DETECTION SYSTEM TEST")
    print("Bachelor Project 2026 - Autonomia")
    print("="*60)
    
    # Sjekk om brukeren vil velge kameraindeks
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print(f"Ugyldig kameraindeks: {sys.argv[1]}")
            print("Bruk: python test_ball_detection.py [kameraindeks]")
            return
    else:
        camera_index = 0
    
    # Opprett og kjør tester
    tester = BallDetectionTester(camera_index=camera_index)
    tester.run()


if __name__ == "__main__":
    main()
