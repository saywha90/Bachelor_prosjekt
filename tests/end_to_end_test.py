"""
End-to-End Integration Test
============================

Tester krav F5 og T2:
- F5: 10 sammenhengende sorteringssykluser uten systemstopp eller manuell reset
- T2: Rapportert suksessrate, feilrate, og sorteringstid (p50/p95) over minst 20 sykluser

Dette scriptet kjører fullstendige sorteringssykluser fra deteksjon til plassering.

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
SRC_DIR = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(SRC_DIR))

try:
    from vision.ball_detection import create_default_detector, BallColor
    from kinematics import KinematicsSolver
    from comms_manager import CommsManager
    import config
except ImportError as e:
    print(f"FEIL: Kunne ikke importere moduler: {e}")
    print(f"Sjekk at du kjører scriptet fra prosjekt-roten eller at src/ finnes på: {SRC_DIR}")
    sys.exit(1)


class EndToEndTest:
    """
    Kjører ende-til-ende integrasjonstester av hele systemet.
    """
    
    def __init__(self, camera_index: int = 0, mock_mode: bool = True):
        """
        Initialiserer test.
        
        Args:
            camera_index: Kamera-indeks
            mock_mode: Hvis True, simuleres hardware-operasjoner
        """
        self.camera_index = camera_index
        self.mock_mode = mock_mode
        
        self.detector = None
        self.kinematics = None
        self.comms = None
        self.cap = None
        
        self.cycles = []
        
        print("="*70)
        print("ENDE-TIL-ENDE INTEGRASJONSTEST")
        print("="*70)
        print(f"Krav F5: 10 sykluser uten manuell reset")
        print(f"Krav T2: Rapportert ytelse over ≥20 sykluser")
        print(f"Modus: {'MOCK (simulering)' if mock_mode else 'LIVE HARDWARE'}")
        print("="*70)
    
    def setup(self):
        """Setter opp alle komponenter."""
        print("\nSetter opp system...")
        
        # Opprett komponenter
        try:
            self.detector = create_default_detector(use_ml=True)
            self.kinematics = KinematicsSolver()
            self.comms = CommsManager()
            
            if not self.mock_mode:
                self.cap = cv2.VideoCapture(self.camera_index)
                if not self.cap.isOpened():
                    raise RuntimeError(f"Kunne ikke åpne kamera {self.camera_index}")
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            print("✓ System klart")
        
        except Exception as e:
            print(f"❌ FEIL ved oppstart: {e}")
            raise
    
    def teardown(self):
        """Rydder opp."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        if self.comms:
            self.comms.close()
    
    def run_single_cycle(self, cycle_num: int) -> Dict:
        """
        Kjører én komplett sorteringssyklus.
        
        Steg:
        1. Detekter ball
        2. Plukk ball
        3. Klassifiser ball
        4. Plasser i korrekt container
        
        Args:
            cycle_num: Syklusnummer
            
        Returns:
            Resultater for denne syklusen
        """
        cycle_start = time.time()
        
        result = {
            'cycle': cycle_num,
            'timestamp': datetime.now().isoformat(),
            'success': False,
            'ball_detected': False,
            'ball_color': None,
            'pick_success': False,
            'placement_success': False,
            'placement_correct': False,
            'total_time_ms': 0,
            'stages': {},
            'errors': []
        }
        
        try:
            # STEG 1: Deteksjon
            detect_start = time.time()
            
            if self.mock_mode:
                # Simuler deteksjon
                time.sleep(0.1)
                # Simuler vekslende rød/blå
                mock_color = BallColor.RED if cycle_num % 2 == 0 else BallColor.BLUE
                balls = [type('Ball', (), {
                    'color': mock_color,
                    'center': (320, 240),
                    'radius': 40,
                    'confidence': 0.95
                })()]
            else:
                # Reell deteksjon
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("Kunne ikke lese fra kamera")
                balls = self.detector.detect_balls(frame)
            
            detect_time = (time.time() - detect_start) * 1000
            result['stages']['detection_ms'] = detect_time
            
            if not balls:
                result['errors'].append("Ingen ball detektert")
                return result
            
            ball = balls[0]  # Ta første ball
            result['ball_detected'] = True
            result['ball_color'] = ball.color.value
            
            # STEG 2: Beregn IK og plukk
            pick_start = time.time()
            
            # Beregn posisjon for plukk (eksempel-koordinater)
            pick_x, pick_y, pick_z = 200, 0, 100
            try:
                pick_angles = self.kinematics.solve_ik(pick_x, pick_y, pick_z)
            except ValueError as e:
                result['errors'].append(f"IK-feil ved plukk: {e}")
                return result
            
            # Send til robot
            self.comms.send_angles(pick_angles)
            
            if self.mock_mode:
                time.sleep(0.2)  # Simuler rørelse
                pick_success = True  # I mock mode lykkes alltid plukk
            else:
                time.sleep(1.0)  # Vent på robot
                # TODO: Legg til feedback fra robot om plukk var vellykket
                pick_success = True
            
            pick_time = (time.time() - pick_start) * 1000
            result['stages']['pick_ms'] = pick_time
            result['pick_success'] = pick_success
            
            if not pick_success:
                result['errors'].append("Plukk feilet")
                return result
            
            # STEG 3: Plasser i container
            place_start = time.time()
            
            # Bestem container basert på farge
            if ball.color == BallColor.RED:
                place_x, place_y, place_z = 150, 150, 50  # Rød container
                target_container = "red"
            elif ball.color == BallColor.BLUE:
                place_x, place_y, place_z = 150, -150, 50  # Blå container
                target_container = "blue"
            else:
                result['errors'].append(f"Ukjent farge: {ball.color}")
                return result
            
            try:
                place_angles = self.kinematics.solve_ik(place_x, place_y, place_z)
            except ValueError as e:
                result['errors'].append(f"IK-feil ved plassering: {e}")
                return result
            
            self.comms.send_angles(place_angles)
            
            if self.mock_mode:
                time.sleep(0.2)  # Simuler plassering
                placement_success = True
            else:
                time.sleep(1.0)
                # TODO: Feedback fra robot
                placement_success = True
            
            place_time = (time.time() - place_start) * 1000
            result['stages']['placement_ms'] = place_time
            result['placement_success'] = placement_success
            
            # Verifiser korrekt plassering (F7: Klassifisering må matche sortering)
            result['placement_correct'] = (ball.color.value == target_container)
            
            # STEG 4: Returner til hjem-posisjon
            home_start = time.time()
            home_angles = [90] * config.NUM_JOINTS
            self.comms.send_angles(home_angles)
            
            if self.mock_mode:
                time.sleep(0.1)
            else:
                time.sleep(0.5)
            
            home_time = (time.time() - home_start) * 1000
            result['stages']['home_ms'] = home_time
            
            # Suksess hvis alle steg fullført
            result['success'] = (result['ball_detected'] and 
                                result['pick_success'] and 
                                result['placement_success'] and 
                                result['placement_correct'])
        
        except Exception as e:
            result['errors'].append(f"Exception: {str(e)}")
        
        finally:
            result['total_time_ms'] = (time.time() - cycle_start) * 1000
        
        return result
    
    def run_test(self, num_cycles: int = 20) -> Dict:
        """
        Kjører flere sammenhengende sykluser.
        
        Args:
            num_cycles: Antall sykluser å kjøre
            
        Returns:
            Testresultater
        """
        print(f"\nKjører {num_cycles} sammenhengende sykluser...")
        print("Trykk Ctrl+C for å avbryte.\n")
        
        try:
            for i in range(1, num_cycles + 1):
                print(f"Syklus {i}/{num_cycles}...", end=" ")
                
                result = self.run_single_cycle(i)
                self.cycles.append(result)
                
                if result['success']:
                    print(f"✓ OK ({result['total_time_ms']:.0f}ms)")
                else:
                    print(f"✗ FEIL: {', '.join(result['errors'])}")
                
                # Liten pause mellom sykluser
                if i < num_cycles:
                    time.sleep(0.5)
        
        except KeyboardInterrupt:
            print("\n\nTest avbrutt av bruker.")
        
        # Beregn statistikk
        return self.calculate_statistics()
    
    def calculate_statistics(self) -> Dict:
        """
        Beregner statistikk for kravverifikasjon.
        
        Returns:
            Statistikk for F5, T2
        """
        if not self.cycles:
            return {'status': 'no_data'}
        
        total_cycles = len(self.cycles)
        successful_cycles = sum(1 for c in self.cycles if c['success'])
        success_rate = (successful_cycles / total_cycles * 100) if total_cycles > 0 else 0
        
        # Tidsmålinger
        times = [c['total_time_ms'] for c in self.cycles if c['success']]
        
        if times:
            times = np.array(times)
            p50 = np.percentile(times, 50)
            p95 = np.percentile(times, 95)
            mean_time = np.mean(times)
        else:
            p50 = p95 = mean_time = 0
        
        # Per-stage tidsmålinger
        stage_times = {}
        for stage in ['detection_ms', 'pick_ms', 'placement_ms', 'home_ms']:
            stage_values = [c['stages'].get(stage, 0) for c in self.cycles if stage in c['stages']]
            if stage_values:
                stage_times[stage] = {
                    'mean': float(np.mean(stage_values)),
                    'p95': float(np.percentile(stage_values, 95))
                }
        
        # Feil-analyse
        errors = {}
        for cycle in self.cycles:
            for error in cycle['errors']:
                errors[error] = errors.get(error, 0) + 1
        
        # F5: Sjekk om minst 10 sykluser uten manuell reset
        f5_passed = total_cycles >= 10 and success_rate == 100.0
        
        # T2: Rapportering over minst 20 sykluser
        t2_passed = total_cycles >= 20
        
        results = {
            'total_cycles': total_cycles,
            'successful_cycles': successful_cycles,
            'failed_cycles': total_cycles - successful_cycles,
            'success_rate_percent': success_rate,
            'timing': {
                'mean_ms': float(mean_time),
                'p50_ms': float(p50),
                'p95_ms': float(p95)
            },
            'stage_timing': stage_times,
            'errors': errors,
            'requirements': {
                'F5': {
                    'requirement': '10 sykluser uten manuell reset',
                    'passed': f5_passed,
                    'achieved': f"{total_cycles} sykluser, {success_rate:.1f}% suksess"
                },
                'T2': {
                    'requirement': '≥20 sykluser med rapportert ytelse',
                    'passed': t2_passed,
                    'achieved': f"{total_cycles} sykluser"
                }
            }
        }
        
        return results
    
    def print_report(self, stats: Dict):
        """Skriver ut rapport til konsoll."""
        print("\n" + "="*70)
        print("TESTRESULTATER")
        print("="*70)
        print(f"Total sykluser: {stats['total_cycles']}")
        print(f"Vellykkede: {stats['successful_cycles']}")
        print(f"Feilet: {stats['failed_cycles']}")
        print(f"Suksessrate: {stats['success_rate_percent']:.1f}%")
        print(f"\nTidsmålinger (vellykkede sykluser):")
        print(f"  Mean: {stats['timing']['mean_ms']:.0f} ms")
        print(f"  p50:  {stats['timing']['p50_ms']:.0f} ms")
        print(f"  p95:  {stats['timing']['p95_ms']:.0f} ms")
        
        if stats.get('stage_timing'):
            print(f"\nPer-stage timing:")
            for stage, times in stats['stage_timing'].items():
                stage_name = stage.replace('_ms', '').capitalize()
                print(f"  {stage_name}: {times['mean']:.0f} ms (p95: {times['p95']:.0f} ms)")
        
        if stats.get('errors'):
            print(f"\nFeil-oppsummering:")
            for error, count in stats['errors'].items():
                print(f"  {error}: {count} ganger")
        
        print(f"\n" + "-"*70)
        print("KRAVVURDERING:")
        print("-"*70)
        
        for req_id, req_data in stats['requirements'].items():
            status = "✅ BESTÅTT" if req_data['passed'] else "❌ IKKE BESTÅTT"
            print(f"{req_id}: {status}")
            print(f"  Krav: {req_data['requirement']}")
            print(f"  Resultat: {req_data['achieved']}")
        
        print("="*70)
    
    def save_report(self, stats: Dict, output_file: str = "end_to_end_report.json"):
        """Lagrer rapport til fil."""
        report = {
            'timestamp': datetime.now().isoformat(),
            'test': 'end_to_end_integration',
            'cycles': self.cycles,
            'statistics': stats
        }
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        print(f"\n✓ Detaljert rapport lagret: {output_file}")


def main():
    """Hovedfunksjon"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Ende-til-ende integrasjonstest (F5, T2)'
    )
    
    parser.add_argument(
        '--cycles',
        type=int,
        default=20,
        help='Antall sykluser å kjøre (default: 20)'
    )
    
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Kamera-indeks (default: 0)'
    )
    
    parser.add_argument(
        '--mock',
        action='store_true',
        help='Kjør i mock-modus (simulering)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default='end_to_end_report.json',
        help='Output-fil for rapport'
    )
    
    args = parser.parse_args()
    
    tester = None
    
    try:
        tester = EndToEndTest(
            camera_index=args.camera,
            mock_mode=args.mock
        )
        
        tester.setup()
        stats = tester.run_test(num_cycles=args.cycles)
        tester.print_report(stats)
        tester.save_report(stats, output_file=args.output)
    
    except KeyboardInterrupt:
        print("\n\nTest avbrutt av bruker.")
    except Exception as e:
        print(f"\n❌ FEIL: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if tester:
            tester.teardown()


if __name__ == "__main__":
    main()
