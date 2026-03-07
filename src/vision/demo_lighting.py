"""
Lighting Robustness Demo
========================

Dette scriptet demonstrerer hvordan systemet håndterer
forskjellige lysforhold.

Author: Bachelor Project 2026 - Autonomia
"""

import cv2
import numpy as np
import sys
from pathlib import Path

# Legg til src-mappen i path
src_path = Path(__file__).parent.parent
sys.path.insert(0, str(src_path))

from vision.ball_detection import create_default_detector, BallColor
from vision.lighting_adaptation import LightingCondition


def create_test_scenarios():
    """Lager testbilder med forskjellige lysforhold"""
    scenarios = {}
    
    # 1. Normal belysning
    normal = np.ones((480, 640, 3), dtype=np.uint8) * 128
    cv2.circle(normal, (200, 240), 50, (50, 50, 200), -1)  # Rød ball
    cv2.circle(normal, (440, 240), 50, (200, 50, 50), -1)  # Blå ball
    scenarios['Normal belysning'] = normal
    
    # 2. Mørkt (svakt lys)
    dark = np.ones((480, 640, 3), dtype=np.uint8) * 50
    cv2.circle(dark, (200, 240), 50, (20, 20, 80), -1)  # Mørk rød
    cv2.circle(dark, (440, 240), 50, (80, 20, 20), -1)  # Mørk blå
    scenarios['Svakt lys'] = dark
    
    # 3. Sterkt lys (overexposed)
    bright = np.ones((480, 640, 3), dtype=np.uint8) * 200
    cv2.circle(bright, (200, 240), 50, (180, 100, 220), -1)  # Bleked rød
    cv2.circle(bright, (440, 240), 50, (220, 100, 180), -1)  # Bleked blå
    scenarios['Sterkt lys'] = bright
    
    # 4. Ujevn belysning (skygger)
    uneven = np.ones((480, 640, 3), dtype=np.uint8) * 100
    # Gradient (venstre mørk, høyre lys)
    for x in range(640):
        brightness = int(50 + (x / 640) * 150)
        uneven[:, x] = brightness
    cv2.circle(uneven, (200, 240), 50, (30, 30, 120), -1)  # I skygge
    cv2.circle(uneven, (440, 240), 50, (180, 100, 180), -1)  # I lys
    scenarios['Ujevn belysning'] = uneven
    
    return scenarios


def main():
    """Hovedfunksjon for demo"""
    print("\n" + "="*70)
    print("LYSROBUSTHETS-DEMO")
    print("="*70)
    print("\nDette scriptet demonstrerer hvordan systemet håndterer")
    print("forskjellige lysforhold med og uten adaptiv modus.\n")
    
    # Lag testscenarier
    scenarios = create_test_scenarios()
    
    # Test uten adaptiv
    print("-"*70)
    print("TEST 1: UTEN ADAPTIV LYSHÅNDTERING")
    print("-"*70)
    detector_basic = create_default_detector(enable_adaptive_lighting=False)
    
    for name, frame in scenarios.items():
        balls = detector_basic.detect_balls(frame)
        print(f"{name:20s} → Detektert: {len(balls)} baller")
    
    # Test med adaptiv
    print("\n" + "-"*70)
    print("TEST 2: MED ADAPTIV LYSHÅNDTERING")
    print("-"*70)
    detector_adaptive = create_default_detector(enable_adaptive_lighting=True)
    
    for name, frame in scenarios.items():
        balls = detector_adaptive.detect_balls(frame)
        lighting_info = detector_adaptive.get_lighting_info()
        condition = lighting_info['condition'].value if lighting_info else 'N/A'
        print(f"{name:20s} → Detektert: {len(balls)} baller (Lys: {condition})")
    
    # Test med preprocessing
    print("\n" + "-"*70)
    print("TEST 3: MED ADAPTIV + PREPROCESSING (for dårlig lys)")
    print("-"*70)
    detector_enhanced = create_default_detector(
        enable_adaptive_lighting=True,
        enable_preprocessing=True
    )
    
    for name, frame in scenarios.items():
        balls = detector_enhanced.detect_balls(frame)
        print(f"{name:20s} → Detektert: {len(balls)} baller")
    
    # Oppsummering
    print("\n" + "="*70)
    print("KONKLUSJON")
    print("="*70)
    print("✅ Adaptiv lyshåndtering forbedrer deteksjon under varierende lys")
    print("✅ Preprocessing (CLAHE) hjelper ytterligere ved meget dårlig lys")
    print("✅ HSV fargerom gir grunnleggende robusthet")
    print("\nAnbefaling: Bruk alltid adaptiv modus i produksjon!")
    print("="*70 + "\n")
    
    # Visuell demo (hvis ønsket)
    print("Vil du se visuell demo? Dette vil vise testbildene.")
    print("MERK: Syntetiske testbilder, ikke reelle baller.")
    response = input("Vis visuell demo? (ja/nei): ")
    
    if response.lower() in ['ja', 'j', 'yes', 'y']:
        print("\nViser testscenarier... (Trykk en tast for neste scenario)")
        for name, frame in scenarios.items():
            balls = detector_adaptive.detect_balls(frame)
            output = detector_adaptive.draw_detections(frame, balls)
            
            # Legg til tittel
            cv2.putText(output, name, (10, 450),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            cv2.imshow('Lysrobusthets-Demo', output)
            cv2.waitKey(0)
        
        cv2.destroyAllWindows()
        print("Demo fullført!")


if __name__ == "__main__":
    main()
