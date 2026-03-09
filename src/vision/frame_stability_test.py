"""
Frame Stability Test
====================

Tester krav F6: Kamera skal levere kontinuerlig videostrøm uten avbrudd
i testperioden, med ≤1% droppede frames i 5 minutter kontinuerlig drift.

Author: Bachelor Project 2026 - Autonomia
"""

import sys
import time
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List
import numpy as np

try:
    import cv2
except ImportError:
    print("FEIL: OpenCV ikke installert. Kjør: pip install opencv-python")
    sys.exit(1)

# Legg til src-mappen i path
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from vision.ball_detection import create_default_detector
except ImportError as e:
    print(f"FEIL: Kunne ikke importere moduler: {e}")
    sys.exit(1)


class FrameStabilityTest:
    """
    Tester videostrøm-stabilitet og frame-drop rate.
    """
    
    def __init__(self, camera_index: int = 0, target_fps: float = 30.0):
        """
        Initialiserer test.
        
        Args:
            camera_index: Kamera-indeks
            target_fps: Forventet FPS (brukes til å beregne forventede frames)
        """
        self.camera_index = camera_index
        self.target_fps = target_fps
        self.cap = None
        
        print("="*70)
        print("FRAME STABILITY TEST - Krav F6")
        print("="*70)
        print("Krav: ≤1% droppede frames i 5 minutter kontinuerlig drift")
        print(f"Kamera: {camera_index}")
        print(f"Target FPS: {target_fps}")
        print("="*70)
    
    def run_test(self, duration_seconds: int = 300) -> Dict:
        """
        Kjører stabilitetestest i spesifisert varighet.
        
        Args:
            duration_seconds: Varighet av test (default: 300s = 5min)
            
        Returns:
            Testresultater med frame-statistikk
        """
        print(f"\nKjører test i {duration_seconds} sekunder ({duration_seconds//60} minutter)...")
        print("Trykk Ctrl+C for å avbryte.\n")
        
        # Åpne kamera
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"❌ FEIL: Kunne ikke åpne kamera {self.camera_index}")
            return {'status': 'error', 'reason': 'camera_not_opened'}
        
        # Sett kamera-innstillinger
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        
        # Statistikk
        start_time = time.time()
        end_time = start_time + duration_seconds
        
        frame_count = 0
        failed_reads = 0
        frame_times = []
        
        # Opprett detektor for å simulere reell bruk
        detector = create_default_detector(use_ml=True)
        
        print("Status (oppdateres hver 10. sekund):")
        print("-" * 70)
        
        last_status_time = start_time
        
        try:
            while time.time() < end_time:
                current_time = time.time()
                
                # Les frame
                frame_start = time.time()
                ret, frame = self.cap.read()
                frame_read_time = time.time() - frame_start
                
                if not ret:
                    failed_reads += 1
                    continue
                
                frame_count += 1
                frame_times.append(frame_read_time)
                
                # Kjør deteksjon for å simulere reell last
                balls = detector.detect_balls(frame)
                
                # Vis status hver 10. sekund
                if current_time - last_status_time >= 10.0:
                    elapsed = current_time - start_time
                    remaining = end_time - current_time
                    current_fps = frame_count / elapsed if elapsed > 0 else 0
                    
                    print(f"  {elapsed:6.1f}s | Frames: {frame_count:5d} | "
                          f"Failed: {failed_reads:3d} | FPS: {current_fps:5.1f} | "
                          f"Remaining: {remaining:5.1f}s")
                    
                    last_status_time = current_time
        
        except KeyboardInterrupt:
            print("\n\nTest avbrutt av bruker.")
        
        finally:
            # Rydd opp
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
        
        # Beregn statistikk
        actual_duration = time.time() - start_time
        expected_frames = self.target_fps * actual_duration
        actual_fps = frame_count / actual_duration if actual_duration > 0 else 0
        
        # Dropped frames = forventede - mottatte
        dropped_frames = max(0, expected_frames - frame_count)
        drop_rate = (dropped_frames / expected_frames * 100) if expected_frames > 0 else 0
        
        # Frame timing statistikk
        if frame_times:
            frame_times = np.array(frame_times) * 1000  # Convert to ms
            mean_time = np.mean(frame_times)
            std_time = np.std(frame_times)
            max_time = np.max(frame_times)
        else:
            mean_time = std_time = max_time = 0.0
        
        # Resultat
        print("\n" + "="*70)
        print("RESULTATER:")
        print("="*70)
        print(f"Varighet: {actual_duration:.1f} sekunder")
        print(f"Forventede frames (@ {self.target_fps} FPS): {expected_frames:.0f}")
        print(f"Mottatte frames: {frame_count}")
        print(f"Failed reads: {failed_reads}")
        print(f"Droppede frames: {dropped_frames:.0f}")
        print(f"Drop rate: {drop_rate:.2f}%")
        print(f"Faktisk FPS: {actual_fps:.2f}")
        print(f"\nFrame read times:")
        print(f"  Mean: {mean_time:.2f} ms")
        print(f"  Std:  {std_time:.2f} ms")
        print(f"  Max:  {max_time:.2f} ms")
        print("="*70)
        
        # Vurdering
        passed = drop_rate <= 1.0 and actual_duration >= duration_seconds * 0.95
        
        if passed:
            print(f"✅ BESTÅTT - Drop rate oppfyller krav ({drop_rate:.2f}% ≤ 1%)")
        else:
            if drop_rate > 1.0:
                print(f"❌ IKKE BESTÅTT - Drop rate for høy ({drop_rate:.2f}% > 1%)")
            if actual_duration < duration_seconds * 0.95:
                print(f"⚠️  ADVARSEL - Test avbrutt før tid ({actual_duration:.1f}s < {duration_seconds}s)")
        
        return {
            'status': 'passed' if passed else 'failed',
            'duration_seconds': actual_duration,
            'target_duration': duration_seconds,
            'expected_frames': expected_frames,
            'received_frames': frame_count,
            'failed_reads': failed_reads,
            'dropped_frames': dropped_frames,
            'drop_rate_percent': drop_rate,
            'actual_fps': actual_fps,
            'target_fps': self.target_fps,
            'frame_timing': {
                'mean_ms': float(mean_time),
                'std_ms': float(std_time),
                'max_ms': float(max_time)
            },
            'requirement': '≤1% drop rate',
            'achieved': f"{drop_rate:.2f}%"
        }
    
    def save_report(self, results: Dict, output_file: str = "frame_stability_report.json"):
        """
        Lagrer testrapport til fil.
        
        Args:
            results: Testresultater fra run_test()
            output_file: Filnavn for rapport
        """
        report = {
            'timestamp': datetime.now().isoformat(),
            'test': 'F6_frame_stability',
            'results': results
        }
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        print(f"\n✓ Rapport lagret: {output_file}")


def main():
    """Hovedfunksjon"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Test av videostrøm-stabilitet (F6)'
    )
    
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Kamera-indeks (default: 0)'
    )
    
    parser.add_argument(
        '--duration',
        type=int,
        default=300,
        help='Testvarighe i sekunder (default: 300 = 5min)'
    )
    
    parser.add_argument(
        '--fps',
        type=float,
        default=30.0,
        help='Forventet FPS (default: 30.0)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default='frame_stability_report.json',
        help='Output-fil for rapport (default: frame_stability_report.json)'
    )
    
    args = parser.parse_args()
    
    try:
        tester = FrameStabilityTest(
            camera_index=args.camera,
            target_fps=args.fps
        )
        
        results = tester.run_test(duration_seconds=args.duration)
        tester.save_report(results, output_file=args.output)
    
    except KeyboardInterrupt:
        print("\n\nTest avbrutt av bruker.")
    except Exception as e:
        print(f"\n❌ FEIL: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
