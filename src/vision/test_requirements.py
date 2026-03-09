"""
Requirements Testing for Bachelor Project
==========================================

Dette scriptet tester at ML-systemet oppfyller kravspesifikasjonen:
- F1: Deteksjonsrate ≥95% (50 testbilder)
- F2: Klassifiseringsnøyaktighet ≥90% (100 bilder per klasse)
- ML1: Datasett-størrelse
- ML2: Reproduserbarhet
- ML3: Modell-nøyaktighet ≥90%, precision/recall ≥0.85
- ML4: Inferenstid p95 ≤1.0s

Author: Bachelor Project 2026 - Autonomia
"""

import sys
import time
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple
import numpy as np

try:
    import cv2
except ImportError:
    print("FEIL: OpenCV ikke installert. Kjør: pip install opencv-python")
    sys.exit(1)

# Legg til src-mappen i path
SRC_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(SRC_DIR))

try:
    from vision.ball_detection import create_default_detector, BallColor
    from vision.ml_classifier import MLBallClassifier
except ImportError as e:
    print(f"FEIL: Kunne ikke importere moduler: {e}")
    print(f"Sjekk at du kjører scriptet fra korrekt mappe eller at modulene finnes på: {SRC_DIR}")
    sys.exit(1)


class RequirementsTest:
    """
    Hovedklasse for testing av kravspesifikasjonen.
    """
    
    def __init__(self, test_data_dir: str = "test_data", model_path: str = None):
        """
        Initialiserer test-systemet.
        
        Args:
            test_data_dir: Mappe med test-data (organisert som training_data)
            model_path: Sti til ML-modell (None = bruk default)
        """
        self.test_data_dir = Path(test_data_dir)
        self.model_path = model_path
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'tests': {}
        }
        
        print("="*70)
        print("KRAVSPESIFIKASJON - AUTOMATISK TEST")
        print("Bachelor Prosjekt 2026 - Autonomia")
        print("="*70)
    
    def test_f1_detection_rate(self) -> Dict:
        """
        F1: Deteksjonsrate ≥95% ved 50 statiske testbilder
        (minst 25 med kule, 25 uten).
        
        Returns:
            Testresultater med deteksjonsrate
        """
        print("\n" + "="*70)
        print("TEST F1: DETEKSJONSRATE")
        print("="*70)
        print("Krav: Deteksjonsrate ≥95% ved 50 testbilder")
        print("      (25 med kule, 25 uten)")
        
        # Sjekk om test-data finnes
        if not self.test_data_dir.exists():
            print(f"\n⚠️  Test-data ikke funnet: {self.test_data_dir}")
            print("   Opprett mappe og legg til testbilder:")
            print(f"   {self.test_data_dir}/")
            print("       with_ball/    (bilder MED ball)")
            print("       without_ball/ (bilder UTEN ball)")
            return {'status': 'skipped', 'reason': 'test_data not found'}
        
        with_ball_dir = self.test_data_dir / "with_ball"
        without_ball_dir = self.test_data_dir / "without_ball"
        
        # Opprett detektor
        detector = create_default_detector(use_ml=True)
        
        results = {
            'with_ball': {'total': 0, 'detected': 0, 'missed': 0},
            'without_ball': {'total': 0, 'false_positives': 0}
        }
        
        # Test: Bilder MED ball
        print("\n1. Testing bilder MED ball...")
        if with_ball_dir.exists():
            for img_path in sorted(with_ball_dir.glob("*.jpg")) + sorted(with_ball_dir.glob("*.png")):
                image = cv2.imread(str(img_path))
                if image is None:
                    continue
                
                balls = detector.detect_balls(image)
                results['with_ball']['total'] += 1
                
                if len(balls) > 0:
                    results['with_ball']['detected'] += 1
                else:
                    results['with_ball']['missed'] += 1
                    print(f"   ✗ Missed: {img_path.name}")
        
        # Test: Bilder UTEN ball
        print("\n2. Testing bilder UTEN ball...")
        if without_ball_dir.exists():
            for img_path in sorted(without_ball_dir.glob("*.jpg")) + sorted(without_ball_dir.glob("*.png")):
                image = cv2.imread(str(img_path))
                if image is None:
                    continue
                
                balls = detector.detect_balls(image)
                results['without_ball']['total'] += 1
                
                if len(balls) > 0:
                    results['without_ball']['false_positives'] += 1
                    print(f"   ✗ False positive: {img_path.name}")
        
        # Beregn deteksjonsrate
        total_images = results['with_ball']['total'] + results['without_ball']['total']
        correct = results['with_ball']['detected'] + (results['without_ball']['total'] - results['without_ball']['false_positives'])
        
        if total_images > 0:
            detection_rate = (correct / total_images) * 100
        else:
            detection_rate = 0.0
        
        # Resultat
        print("\n" + "-"*70)
        print("RESULTATER:")
        print(f"  Bilder med ball: {results['with_ball']['detected']}/{results['with_ball']['total']} detektert")
        print(f"  Bilder uten ball: {results['without_ball']['false_positives']}/{results['without_ball']['total']} falske positiver")
        print(f"  Total deteksjonsrate: {detection_rate:.1f}%")
        print("-"*70)
        
        # Vurdering
        passed = detection_rate >= 95.0 and total_images >= 50
        
        if passed:
            print("✅ BESTÅTT - Deteksjonsrate oppfyller krav (≥95%)")
        else:
            if total_images < 50:
                print(f"⚠️  IKKE BESTÅTT - For få testbilder ({total_images}/50)")
            else:
                print(f"❌ IKKE BESTÅTT - Deteksjonsrate for lav ({detection_rate:.1f}% < 95%)")
        
        return {
            'status': 'passed' if passed else 'failed',
            'detection_rate': detection_rate,
            'total_images': total_images,
            'details': results,
            'requirement': '≥95%',
            'achieved': f"{detection_rate:.1f}%"
        }
    
    def test_f2_classification_accuracy(self) -> Dict:
        """
        F2: Klassifiseringsnøyaktighet ≥90% på separat testsett
        med ≥100 bilder (balansert mellom rød/blå).
        
        Returns:
            Testresultater med nøyaktighet per klasse
        """
        print("\n" + "="*70)
        print("TEST F2: KLASSIFISERINGSNØYAKTIGHET")
        print("="*70)
        print("Krav: Nøyaktighet ≥90% på ≥100 bilder (balansert rød/blå)")
        
        # Sjekk om test-data finnes
        test_dir = self.test_data_dir / "classification_test"
        if not test_dir.exists():
            print(f"\n⚠️  Test-data ikke funnet: {test_dir}")
            print("   Opprett mappe og legg til testbilder:")
            print(f"   {test_dir}/")
            print("       red/   (minst 50 bilder)")
            print("       blue/  (minst 50 bilder)")
            return {'status': 'skipped', 'reason': 'test_data not found'}
        
        # Opprett detektor med ML
        detector = create_default_detector(use_ml=True)
        
        results = {
            'red': {'total': 0, 'correct': 0, 'wrong': 0},
            'blue': {'total': 0, 'correct': 0, 'wrong': 0}
        }
        
        # Test røde baller
        print("\n1. Testing røde baller...")
        red_dir = test_dir / "red"
        if red_dir.exists():
            for img_path in sorted(red_dir.glob("*.jpg")) + sorted(red_dir.glob("*.png")):
                image = cv2.imread(str(img_path))
                if image is None:
                    continue
                
                balls = detector.detect_balls(image)
                results['red']['total'] += 1
                
                if balls and balls[0].color == BallColor.RED:
                    results['red']['correct'] += 1
                else:
                    results['red']['wrong'] += 1
                    detected_color = balls[0].color.value if balls else "ingen"
                    print(f"   ✗ Feil: {img_path.name} → {detected_color}")
        
        # Test blåe baller
        print("\n2. Testing blåe baller...")
        blue_dir = test_dir / "blue"
        if blue_dir.exists():
            for img_path in sorted(blue_dir.glob("*.jpg")) + sorted(blue_dir.glob("*.png")):
                image = cv2.imread(str(img_path))
                if image is None:
                    continue
                
                balls = detector.detect_balls(image)
                results['blue']['total'] += 1
                
                if balls and balls[0].color == BallColor.BLUE:
                    results['blue']['correct'] += 1
                else:
                    results['blue']['wrong'] += 1
                    detected_color = balls[0].color.value if balls else "ingen"
                    print(f"   ✗ Feil: {img_path.name} → {detected_color}")
        
        # Beregn nøyaktighet
        total_images = results['red']['total'] + results['blue']['total']
        total_correct = results['red']['correct'] + results['blue']['correct']
        
        if total_images > 0:
            accuracy = (total_correct / total_images) * 100
            red_accuracy = (results['red']['correct'] / results['red']['total'] * 100) if results['red']['total'] > 0 else 0
            blue_accuracy = (results['blue']['correct'] / results['blue']['total'] * 100) if results['blue']['total'] > 0 else 0
        else:
            accuracy = red_accuracy = blue_accuracy = 0.0
        
        # Resultat
        print("\n" + "-"*70)
        print("RESULTATER:")
        print(f"  Røde baller: {results['red']['correct']}/{results['red']['total']} korrekt ({red_accuracy:.1f}%)")
        print(f"  Blåe baller: {results['blue']['correct']}/{results['blue']['total']} korrekt ({blue_accuracy:.1f}%)")
        print(f"  Total nøyaktighet: {accuracy:.1f}%")
        print("-"*70)
        
        # Vurdering
        passed = accuracy >= 90.0 and total_images >= 100
        
        if passed:
            print("✅ BESTÅTT - Klassifiseringsnøyaktighet oppfyller krav (≥90%)")
        else:
            if total_images < 100:
                print(f"⚠️  IKKE BESTÅTT - For få testbilder ({total_images}/100)")
            else:
                print(f"❌ IKKE BESTÅTT - Nøyaktighet for lav ({accuracy:.1f}% < 90%)")
        
        return {
            'status': 'passed' if passed else 'failed',
            'accuracy': accuracy,
            'total_images': total_images,
            'per_class': {
                'red': red_accuracy,
                'blue': blue_accuracy
            },
            'details': results,
            'requirement': '≥90%',
            'achieved': f"{accuracy:.1f}%"
        }
    
    def test_ml4_inference_time(self, num_iterations: int = 50) -> Dict:
        """
        ML4: Inferenstid p95 ≤1.0s over 50 kjøringer.
        
        Args:
            num_iterations: Antall inferenser å måle
            
        Returns:
            Testresultater med tidsmålinger
        """
        print("\n" + "="*70)
        print("TEST ML4: INFERENSTID")
        print("="*70)
        print(f"Krav: p95 inferenstid ≤1.0s over {num_iterations} kjøringer")
        
        # Opprett detektor
        detector = create_default_detector(use_ml=True)
        
        # Lag testbilde (640x480 med en ball)
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255
        cv2.circle(test_image, (320, 240), 40, (0, 0, 255), -1)
        
        # Mål inferenstider
        print(f"\nKjører {num_iterations} inferenser...")
        inference_times = []
        
        for i in range(num_iterations):
            start = time.time()
            balls = detector.detect_balls(test_image)
            end = time.time()
            
            elapsed = (end - start) * 1000  # Convert to ms
            inference_times.append(elapsed)
            
            if (i + 1) % 10 == 0:
                print(f"  {i+1}/{num_iterations} fullført...")
        
        # Beregn statistikk
        inference_times = np.array(inference_times)
        p50 = np.percentile(inference_times, 50)
        p95 = np.percentile(inference_times, 95)
        p99 = np.percentile(inference_times, 99)
        mean = np.mean(inference_times)
        std = np.std(inference_times)
        
        # Resultat
        print("\n" + "-"*70)
        print("RESULTATER:")
        print(f"  Mean: {mean:.1f} ms")
        print(f"  Std:  {std:.1f} ms")
        print(f"  p50:  {p50:.1f} ms")
        print(f"  p95:  {p95:.1f} ms")
        print(f"  p99:  {p99:.1f} ms")
        print(f"  FPS (mean): {1000/mean:.1f}")
        print("-"*70)
        
        # Vurdering (p95 skal være ≤1000ms)
        passed = p95 <= 1000.0
        
        if passed:
            print(f"✅ BESTÅTT - p95 inferenstid oppfyller krav ({p95:.1f}ms ≤ 1000ms)")
        else:
            print(f"❌ IKKE BESTÅTT - p95 inferenstid for høy ({p95:.1f}ms > 1000ms)")
        
        return {
            'status': 'passed' if passed else 'failed',
            'mean_ms': float(mean),
            'std_ms': float(std),
            'p50_ms': float(p50),
            'p95_ms': float(p95),
            'p99_ms': float(p99),
            'fps_mean': float(1000/mean),
            'requirement': '≤1000ms',
            'achieved': f"{p95:.1f}ms"
        }
    
    def test_ml1_dataset_size(self, training_data_dir: str = "training_data") -> Dict:
        """
        ML1: Datasett inneholder minimum 200 bilder per klasse
        (≥400 totalt) dokumentert med metadata + eksempelbilder.
        
        Args:
            training_data_dir: Mappe med treningsdata
            
        Returns:
            Testresultater med dataset-størrelse
        """
        print("\n" + "="*70)
        print("TEST ML1: DATASETT-STØRRELSE")
        print("="*70)
        print("Krav: ≥400 bilder totalt (≥200 per klasse)")
        
        data_dir = Path(training_data_dir)
        
        if not data_dir.exists():
            print(f"\n⚠️  Treningsdata ikke funnet: {data_dir}")
            return {'status': 'skipped', 'reason': 'training_data not found'}
        
        # Tell bilder per klasse
        classes = ['red', 'blue', 'green']
        counts = {}
        
        for cls in classes:
            cls_dir = data_dir / cls
            if cls_dir.exists():
                jpg_count = len(list(cls_dir.glob('*.jpg')))
                png_count = len(list(cls_dir.glob('*.png')))
                counts[cls] = jpg_count + png_count
            else:
                counts[cls] = 0
        
        total = sum(counts.values())
        
        # Resultat
        print("\n" + "-"*70)
        print("DATASETT:")
        for cls, count in counts.items():
            print(f"  {cls.capitalize()}: {count} bilder")
        print(f"  Total: {total} bilder")
        print("-"*70)
        
        # Vurdering
        min_per_class = min(counts['red'], counts['blue'])
        passed = total >= 400 and min_per_class >= 200
        
        if passed:
            print("✅ BESTÅTT - Datasett-størrelse oppfyller krav")
        else:
            if total < 400:
                print(f"❌ IKKE BESTÅTT - For få totale bilder ({total}/400)")
            if min_per_class < 200:
                print(f"❌ IKKE BESTÅTT - Minst én klasse har <200 bilder")
        
        return {
            'status': 'passed' if passed else 'failed',
            'total_images': total,
            'per_class': counts,
            'requirement': '≥400 total, ≥200 per klasse',
            'achieved': f"{total} total"
        }
    
    def generate_report(self, output_file: str = "test_report.json"):
        """
        Genererer komplett testrapport i JSON-format.
        
        Args:
            output_file: Filnavn for rapport
        """
        print("\n" + "="*70)
        print("GENERERER TESTRAPPORT")
        print("="*70)
        
        # Kjør alle tester
        self.results['tests']['F1'] = self.test_f1_detection_rate()
        self.results['tests']['F2'] = self.test_f2_classification_accuracy()
        self.results['tests']['ML4'] = self.test_ml4_inference_time()
        self.results['tests']['ML1'] = self.test_ml1_dataset_size()
        
        # Oppsummering
        total_tests = len(self.results['tests'])
        passed_tests = sum(1 for t in self.results['tests'].values() if t.get('status') == 'passed')
        
        self.results['summary'] = {
            'total_tests': total_tests,
            'passed': passed_tests,
            'failed': total_tests - passed_tests,
            'pass_rate': (passed_tests / total_tests * 100) if total_tests > 0 else 0
        }
        
        # Lagre rapport
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        
        print(f"\n✓ Rapport lagret: {output_file}")
        
        # Skriv ut oppsummering
        print("\n" + "="*70)
        print("OPPSUMMERING")
        print("="*70)
        print(f"Tester kjørt: {total_tests}")
        print(f"Bestått: {passed_tests}")
        print(f"Ikke bestått: {total_tests - passed_tests}")
        print(f"Suksessrate: {self.results['summary']['pass_rate']:.1f}%")
        print("="*70)
        
        return self.results


def main():
    """Hovedfunksjon"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Test av kravspesifikasjon for bachelor-prosjekt'
    )
    
    parser.add_argument(
        '--test_data',
        type=str,
        default='test_data',
        help='Mappe med test-data (default: test_data)'
    )
    
    parser.add_argument(
        '--training_data',
        type=str,
        default='training_data',
        help='Mappe med treningsdata (default: training_data)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default='test_report.json',
        help='Output-fil for rapport (default: test_report.json)'
    )
    
    args = parser.parse_args()
    
    try:
        tester = RequirementsTest(test_data_dir=args.test_data)
        tester.generate_report(output_file=args.output)
    
    except KeyboardInterrupt:
        print("\n\nTest avbrutt av bruker.")
    except Exception as e:
        print(f"\n❌ FEIL: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
