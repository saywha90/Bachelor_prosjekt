"""
Test Ball Detection uten Fysisk Kamera
=======================================

Dette scriptet tester balldeteksjonssystemet ved å generere
et syntetisk testbilde med fargede sirkler.

Nyttig for:
- Testing uten tilgang til kamera
- Validering av deteksjonslogikk
- Debugging
- Demonstrasjon

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


def create_test_image(width=640, height=480):
    """
    Lager et syntetisk testbilde med røde og blåe sirkler.
    
    Returns:
        Testbilde i BGR-format
    """
    # Lag hvit bakgrunn
    image = np.ones((height, width, 3), dtype=np.uint8) * 255
    
    # Definer baller (x, y, radius, color_bgr, name)
    balls = [
        # Røde baller
        (150, 150, 40, (0, 0, 255), "Rød #1"),
        (400, 200, 50, (0, 0, 200), "Rød #2 (mørkere)"),
        (500, 350, 35, (0, 0, 230), "Rød #3"),
        
        # Blåe baller
        (200, 350, 45, (255, 0, 0), "Blå #1"),
        (450, 100, 38, (200, 0, 0), "Blå #2 (mørkere)"),
        
        # Også en grønn (skal IKKE detekteres)
        (320, 240, 30, (0, 200, 0), "Grønn (skal ignoreres)"),
    ]
    
    print("\nGenererer testbilde med følgende objekter:")
    print("-" * 60)
    
    # Tegn ballene
    for i, (x, y, radius, color, name) in enumerate(balls):
        # Tegn fylt sirkel
        cv2.circle(image, (x, y), radius, color, -1)
        
        # Legg til litt tekstur (gjør den mer realistisk)
        # Lyspunkt i øvre venstre (simulation av lys)
        highlight_pos = (x - radius//3, y - radius//3)
        cv2.circle(image, highlight_pos, radius//4, 
                  tuple(min(c + 50, 255) for c in color), -1)
        
        print(f"{i+1}. {name:25s} @ ({x:3d}, {y:3d}), r={radius}px")
    
    print("-" * 60)
    
    return image


def run_test():
    """Kjører fullstendig test av deteksjonssystemet"""
    
    print("="*70)
    print("BALL DETECTION TEST - SYNTETISK BILDE")
    print("="*70)
    
    # Opprett detektor
    print("\n1. Oppretter detektor...")
    detector = create_default_detector()
    print("   ✓ Detektor opprettet")
    
    # Generer testbilde
    print("\n2. Genererer testbilde...")
    test_image = create_test_image()
    print("   ✓ Testbilde generert (640x480)")
    
    # Kjør deteksjon
    print("\n3. Kjører deteksjon...")
    detected_balls = detector.detect_balls(test_image)
    print(f"   ✓ Deteksjon fullført")
    
    # Vis resultater
    print("\n" + "="*70)
    print("RESULTATER")
    print("="*70)
    print(f"Antall detekterte baller: {len(detected_balls)}")
    print()
    
    if detected_balls:
        # Tell per farge
        red_count = sum(1 for b in detected_balls if b.color == BallColor.RED)
        blue_count = sum(1 for b in detected_balls if b.color == BallColor.BLUE)
        
        print(f"  Røde baller:  {red_count}")
        print(f"  Blåe baller:  {blue_count}")
        print()
        print("Detaljert informasjon:")
        print("-" * 70)
        
        for i, ball in enumerate(detected_balls, 1):
            print(f"{i}. {ball.color.value.upper():4s} @ pos={ball.center}, "
                  f"r={ball.radius:.1f}px, confidence={ball.confidence:.3f}")
    else:
        print("  ⚠️  Ingen baller detektert!")
        print("\nMulige årsaker:")
        print("  - Fargeverdiene må justeres")
        print("  - For strenge deteksjonsparametere")
    
    print("="*70)
    
    # Lag visualisering
    print("\n4. Lager visualisering...")
    output_image = detector.draw_detections(test_image, detected_balls)
    
    # Vis bildene
    print("   ✓ Visualisering klar")
    print("\nViser bilder... (Trykk en tast for å lukke)")
    
    # Kombiner original og output for sammenligning
    combined = np.hstack([test_image, output_image])
    
    cv2.imshow('Test - Original (venstre) | Deteksjon (høyre)', combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Valideringstest
    print("\n" + "="*70)
    print("VALIDERING")
    print("="*70)
    
    # Vi forventer 3 røde + 2 blåe = 5 totalt
    expected_total = 5
    expected_red = 3
    expected_blue = 2
    
    actual_red = sum(1 for b in detected_balls if b.color == BallColor.RED)
    actual_blue = sum(1 for b in detected_balls if b.color == BallColor.BLUE)
    actual_total = len(detected_balls)
    
    print(f"Forventet: {expected_total} baller (R:{expected_red}, B:{expected_blue})")
    print(f"Funnet:    {actual_total} baller (R:{actual_red}, B:{actual_blue})")
    print()
    
    if actual_total == expected_total:
        print("✅ TEST BESTÅTT! Alle baller detektert korrekt.")
    else:
        print("⚠️  TEST DELVIS: Noen baller mangler eller falske positiver.")
        
        if actual_total < expected_total:
            print(f"   → {expected_total - actual_total} ball(er) ikke detektert")
            print("   Tips: Senk min_circularity eller juster HSV-verdier")
        else:
            print(f"   → {actual_total - expected_total} falsk positiv(er)")
            print("   Tips: Øk min_circularity eller smalere HSV-område")
    
    print("="*70)
    
    # Statistikk
    stats = detector.get_statistics()
    print("\nStatistikk:")
    print(f"  Totalt frames prosessert: {stats['total_frames']}")
    print(f"  Totalt deteksjoner: {stats['red'] + stats['blue']}")
    
    print("\n✓ Test fullført!\n")


def run_performance_test():
    """Kjører en ytelsestest"""
    print("\n" + "="*70)
    print("YTELSESTEST")
    print("="*70)
    
    detector = create_default_detector()
    test_image = create_test_image()
    
    import time
    
    num_iterations = 100
    print(f"Kjører {num_iterations} deteksjoner...")
    
    start_time = time.time()
    
    for i in range(num_iterations):
        balls = detector.detect_balls(test_image)
    
    end_time = time.time()
    elapsed = end_time - start_time
    
    fps = num_iterations / elapsed
    avg_time_ms = (elapsed / num_iterations) * 1000
    
    print(f"\nResultater:")
    print(f"  Totalt tid: {elapsed:.2f} sekunder")
    print(f"  Gjennomsnitt: {avg_time_ms:.2f} ms per frame")
    print(f"  FPS: {fps:.1f}")
    print()
    
    if fps >= 30:
        print("✅ Ytelse: Utmerket (30+ FPS)")
    elif fps >= 15:
        print("✓ Ytelse: God (15-30 FPS, OK for Raspberry Pi)")
    else:
        print("⚠️ Ytelse: Lav (<15 FPS, kan være treg på Raspberry Pi)")
    
    print("="*70 + "\n")


def main():
    """Hovedfunksjon"""
    print("\n")
    print("╔" + "="*68 + "╗")
    print("║" + " "*10 + "BALL DETECTION SYSTEM - TEST UTEN KAMERA" + " "*18 + "║")
    print("║" + " "*15 + "Bachelor Project 2026 - Autonomia" + " "*20 + "║")
    print("╚" + "="*68 + "╝")
    print()
    
    try:
        # Kjør hovedtest
        run_test()
        
        # Spør om ytelsestest
        print("\nVil du kjøre en ytelsestest?")
        response = input("Skriv 'ja' for ytelsestest, eller trykk Enter for å avslutte: ")
        
        if response.lower() in ['ja', 'j', 'yes', 'y']:
            run_performance_test()
    
    except KeyboardInterrupt:
        print("\n\nAvbrutt av bruker (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ FEIL: {e}")
        import traceback
        traceback.print_exc()
    
    print("Takk for at du testet systemet!")


if __name__ == "__main__":
    main()
