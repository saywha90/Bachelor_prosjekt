"""
HSV Color Tuner for Ball Detection
===================================

Dette verktøyet lar deg interaktivt justere HSV-fargeverdiene
for optimal deteksjon av røde og blåe baller.

Bruk trackbars (skyveknapper) til å justere HSV-verdiene i sanntid
og se resultatene umiddelbart.

Author: Bachelor Project 2026 - Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cv2
import numpy as np
import sys
from pathlib import Path

from vision.camera import OAKCamera


class HSVTuner:
    """Interaktivt verktøy for å tune HSV-verdier"""
    
    def __init__(self):
        self._oak_cam = None
        self.current_color = 'red'  # 'red' eller 'blue'
        
        # Startpunkter kalibrert for OAK IMX378-sensor (verdiene er mørke og høyt mettet)
        self.hsv_ranges = {
            'red_low': {
                'h_min': 0,   'h_max': 11,
                's_min': 120, 's_max': 255,
                'v_min': 15,  'v_max': 255
            },
            'red_high': {
                'h_min': 168, 'h_max': 179,
                's_min': 120, 's_max': 255,
                'v_min': 15,  'v_max': 255
            },
            'blue': {
                'h_min': 90,  'h_max': 135,
                's_min': 90,  's_max': 255,
                'v_min': 8,   'v_max': 255
            }
        }
    
    def nothing(self, x):
        """Callback for trackbars (gjør ingenting)"""
        pass
    
    def create_trackbars(self, window_name):
        """Lager trackbars for HSV-justering"""
        # Hue (0-179 i OpenCV)
        cv2.createTrackbar('H Min', window_name, 0, 179, self.nothing)
        cv2.createTrackbar('H Max', window_name, 179, 179, self.nothing)
        
        # Saturation (0-255)
        cv2.createTrackbar('S Min', window_name, 0, 255, self.nothing)
        cv2.createTrackbar('S Max', window_name, 255, 255, self.nothing)
        
        # Value (0-255)
        cv2.createTrackbar('V Min', window_name, 0, 255, self.nothing)
        cv2.createTrackbar('V Max', window_name, 255, 255, self.nothing)
    
    def set_trackbar_values(self, window_name, color_key):
        """Setter trackbar-verdier basert på color_key"""
        ranges = self.hsv_ranges[color_key]
        cv2.setTrackbarPos('H Min', window_name, ranges['h_min'])
        cv2.setTrackbarPos('H Max', window_name, ranges['h_max'])
        cv2.setTrackbarPos('S Min', window_name, ranges['s_min'])
        cv2.setTrackbarPos('S Max', window_name, ranges['s_max'])
        cv2.setTrackbarPos('V Min', window_name, ranges['v_min'])
        cv2.setTrackbarPos('V Max', window_name, ranges['v_max'])
    
    def get_trackbar_values(self, window_name):
        """Henter nåværende trackbar-verdier"""
        h_min = cv2.getTrackbarPos('H Min', window_name)
        h_max = cv2.getTrackbarPos('H Max', window_name)
        s_min = cv2.getTrackbarPos('S Min', window_name)
        s_max = cv2.getTrackbarPos('S Max', window_name)
        v_min = cv2.getTrackbarPos('V Min', window_name)
        v_max = cv2.getTrackbarPos('V Max', window_name)
        
        return {
            'h_min': h_min, 'h_max': h_max,
            's_min': s_min, 's_max': s_max,
            'v_min': v_min, 'v_max': v_max
        }
    
    def create_mask(self, hsv, ranges):
        """Lager maske fra HSV-ranges"""
        lower = np.array([ranges['h_min'], ranges['s_min'], ranges['v_min']])
        upper = np.array([ranges['h_max'], ranges['s_max'], ranges['v_max']])
        return cv2.inRange(hsv, lower, upper)
    
    def print_instructions(self):
        """Skriver ut instruksjoner"""
        print("\n" + "="*70)
        print("HSV COLOR TUNER")
        print("="*70)
        print("INSTRUKSJONER:")
        print("  1. Juster trackbars til du får en ren hvit silhuett av ballen")
        print("  2. Minimer støy (svart bakgrunn)")
        print("  3. Trykk 'r' for rød, 'b' for blå")
        print("  4. Trykk 's' for å lagre verdiene")
        print("  5. Trykk 'p' for å skrive ut verdiene")
        print("  6. Trykk 'q' for å avslutte")
        print()
        print("TIPS:")
        print("  - H (Hue): Representerer fargen")
        print("  - S (Saturation): Hvor 'ren' fargen er (høy = levende)")
        print("  - V (Value): Lysstyrke (høy = lys)")
        print("  - For RØD: Bruk enten 0-10 ELLER 170-179")
        print("="*70 + "\n")
    
    def print_current_values(self, color_name, ranges):
        """Skriver ut nåværende verdier"""
        print("\n" + "-"*70)
        print(f"NÅVÆRENDE VERDIER FOR {color_name.upper()}")
        print("-"*70)
        print(f"H (Hue):        Min: {ranges['h_min']:3d}   Max: {ranges['h_max']:3d}")
        print(f"S (Saturation): Min: {ranges['s_min']:3d}   Max: {ranges['s_max']:3d}")
        print(f"V (Value):      Min: {ranges['v_min']:3d}   Max: {ranges['v_max']:3d}")
        print()
        print("Python kode for ball_detection.py:")
        print(f"lower = np.array([{ranges['h_min']}, {ranges['s_min']}, {ranges['v_min']}])")
        print(f"upper = np.array([{ranges['h_max']}, {ranges['s_max']}, {ranges['v_max']}])")
        print("-"*70 + "\n")
    
    def save_values(self):
        """Lagrer verdier til fil"""
        filename = "hsv_calibration.txt"
        
        try:
            # Bruk Path for sikker filhåndtering
            output_path = Path.cwd() / filename
            
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write("HSV KALIBRERINGSVERDIER\n")
                f.write("="*70 + "\n\n")
                
                for color_key, ranges in self.hsv_ranges.items():
                    f.write(f"{color_key.upper()}:\n")
                    f.write(f"  H: {ranges['h_min']}-{ranges['h_max']}\n")
                    f.write(f"  S: {ranges['s_min']}-{ranges['s_max']}\n")
                    f.write(f"  V: {ranges['v_min']}-{ranges['v_max']}\n")
                    f.write("\n")
                    f.write(f"  Python kode:\n")
                    f.write(f"  lower = np.array([{ranges['h_min']}, {ranges['s_min']}, {ranges['v_min']}])\n")
                    f.write(f"  upper = np.array([{ranges['h_max']}, {ranges['s_max']}, {ranges['v_max']}])\n")
                    f.write("\n" + "-"*70 + "\n\n")
            
            print(f"✓ Verdier lagret til {output_path}")
        
        except Exception as e:
            print(f"❌ FEIL ved lagring. Prøv igjen.")
            # Log detaljert feil for debugging
            import logging
            logging.error(f"Lagringsfeil: {e}")
    
    def run(self):
        """Hovedløkke"""
        # Åpne OAK kamera
        self._oak_cam = OAKCamera(resolution=(1280, 720))
        self._oak_cam.open()

        if not self._oak_cam.isOpened():
            print("FEIL: Kunne ikke åpne OAK kamera")
            return

        print("✓ OAK kamera åpnet (1280×720)")
        
        # Opprett vinduer
        window_original = 'Original'
        window_hsv = 'HSV'
        window_mask = 'Maske'
        window_result = 'Resultat'
        
        cv2.namedWindow(window_original, cv2.WINDOW_NORMAL)
        cv2.namedWindow(window_hsv, cv2.WINDOW_NORMAL)
        cv2.namedWindow(window_mask, cv2.WINDOW_NORMAL)
        cv2.namedWindow(window_result, cv2.WINDOW_NORMAL)
        
        # Opprett trackbars
        self.create_trackbars(window_mask)
        
        # Sett initial verdier (rød)
        self.set_trackbar_values(window_mask, 'red_low')
        current_mode = 'red_low'
        
        self.print_instructions()
        print(f"Nåværende modus: RØD (lav)")
        
        try:
            while True:
                ret, frame = self._oak_cam.read()
                
                if not ret:
                    print("Kunne ikke lese fra kamera")
                    break
                
                # Konverter til HSV
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                blurred_hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
                
                # Hent trackbar-verdier
                ranges = self.get_trackbar_values(window_mask)
                
                # Opprett maske
                mask = self.create_mask(blurred_hsv, ranges)
                
                # Anvend maske på original
                result = cv2.bitwise_and(frame, frame, mask=mask)
                
                # Vis bilder
                cv2.imshow(window_original, frame)
                cv2.imshow(window_hsv, hsv)
                cv2.imshow(window_mask, mask)
                cv2.imshow(window_result, result)
                
                # Håndter input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    # Bytt til rød (lav)
                    current_mode = 'red_low'
                    self.set_trackbar_values(window_mask, current_mode)
                    print("Byttet til: RØD (lav område: 0-10)")
                elif key == ord('R'):
                    # Bytt til rød (høy)
                    current_mode = 'red_high'
                    self.set_trackbar_values(window_mask, current_mode)
                    print("Byttet til: RØD (høyt område: 170-179)")
                elif key == ord('b'):
                    # Bytt til blå
                    current_mode = 'blue'
                    self.set_trackbar_values(window_mask, current_mode)
                    print("Byttet til: BLÅ")
                elif key == ord('s'):
                    # Lagre nåværende verdier
                    self.hsv_ranges[current_mode] = ranges.copy()
                    self.save_values()
                elif key == ord('p'):
                    # Print verdier
                    self.print_current_values(current_mode, ranges)
                elif key == ord('u'):
                    # Oppdater lagrede verdier
                    self.hsv_ranges[current_mode] = ranges.copy()
                    print(f"✓ Verdier oppdatert for {current_mode}")
        
        except KeyboardInterrupt:
            print("\n\nAvbrutt av bruker")
        
        finally:
            # Rydd opp
            self._oak_cam.release()
            cv2.destroyAllWindows()
            
            print("\n" + "="*70)
            print("SLUTTRESULTAT")
            print("="*70)
            
            for color_key, ranges in self.hsv_ranges.items():
                self.print_current_values(color_key, ranges)


def main():
    """Hovedfunksjon"""
    tuner = HSVTuner()
    tuner.run()


if __name__ == "__main__":
    main()
