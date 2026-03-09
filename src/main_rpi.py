"""
main_rpi.py

Hovedinngangen til robotstyringsprogrammet på Raspberry Pi.
Knytter sammen kinematikk (hjernen), kommunikasjon (nervene) og datasyn (øynene).

Bruker et enkelt tekstbasert grensesnitt (CLI) for Fase 1-testing.
Dette gjør det lett å teste spesifikke koordinater uten å trenge et GUI eller datasyn.

Logging:
- Alle operasjoner logges til operation_log.json for kravverifikasjon (F3, F4, F7)
"""

import sys
import time
import json
from datetime import datetime
from pathlib import Path
import config
from kinematics import KinematicsSolver
from comms_manager import CommsManager


class OperationLogger:
    """
    Logger operasjoner for kravverifikasjon (F3, F4, F7).
    
    Logg-format:
    {
        "timestamp": "2026-03-09T10:30:45",
        "operation_id": "OP_001",
        "ball_color": "red",
        "detection_confidence": 0.95,
        "pick_success": true,
        "placement_success": true,
        "placement_container": "red",
        "placement_correct": true,
        "errors": []
    }
    """
    
    def __init__(self, log_file: str = "operation_log.json", max_log_size_mb: int = 10):
        self.log_file = Path(log_file)
        self.max_log_size = max_log_size_mb * 1024 * 1024  # Convert to bytes
        self.operations = []
        self.operation_counter = 0
        
        # Sjekk og roter logg hvis for stor
        self._rotate_log_if_needed()
        
        # Last eksisterende logg hvis den finnes
        if self.log_file.exists():
            try:
                with open(self.log_file, 'r') as f:
                    self.operations = json.load(f)
                    if self.operations:
                        # Finn høyeste operation ID
                        ids = [int(op['operation_id'].split('_')[1]) for op in self.operations if 'operation_id' in op]
                        if ids:
                            self.operation_counter = max(ids)
            except Exception as e:
                print(f"Advarsel: Kunne ikke laste eksisterende logg: {e}")
    
    def _rotate_log_if_needed(self):
        """Roterer logg-fil hvis den blir for stor (DoS-beskyttelse)."""
        if self.log_file.exists():
            file_size = self.log_file.stat().st_size
            if file_size > self.max_log_size:
                # Arkiver gammel logg
                archive_name = f"{self.log_file.stem}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                archive_path = self.log_file.parent / archive_name
                self.log_file.rename(archive_path)
                print(f"INFO: Log-fil rotert til {archive_name} (størrelse: {file_size/1024/1024:.1f} MB)")
    
    def start_operation(self, ball_color: str, confidence: float) -> str:
        """
        Starter en ny operasjon (ball detektert).
        
        Args:
            ball_color: Farge på ball (red/blue)
            confidence: Deteksjonskonfiddens (0.0-1.0)
            
        Returns:
            Operation ID
        """
        self.operation_counter += 1
        op_id = f"OP_{self.operation_counter:04d}"
        
        operation = {
            'operation_id': op_id,
            'timestamp_start': datetime.now().isoformat(),
            'ball_color': ball_color,
            'detection_confidence': confidence,
            'pick_success': None,
            'placement_success': None,
            'placement_container': None,
            'placement_correct': None,
            'errors': []
        }
        
        self.operations.append(operation)
        return op_id
    
    def log_pick_result(self, op_id: str, success: bool, error: str = None):
        """
        Logger plukk-resultat (F3: Plukk-suksess ≥90%).
        
        Args:
            op_id: Operation ID
            success: Om plukk var vellykket
            error: Feilmelding hvis det feilet
        """
        for op in self.operations:
            if op['operation_id'] == op_id:
                op['pick_success'] = success
                op['timestamp_pick'] = datetime.now().isoformat()
                if error:
                    op['errors'].append(f"Pick error: {error}")
                break
    
    def log_placement_result(self, op_id: str, success: bool, 
                            container: str, correct: bool, error: str = None):
        """
        Logger plasserings-resultat (F4: Plassering i korrekt container = 100%).
        
        Args:
            op_id: Operation ID
            success: Om plassering var vellykket
            container: Hvilken container ballen ble plassert i
            correct: Om plasseringen var i korrekt container
            error: Feilmelding hvis det feilet
        """
        for op in self.operations:
            if op['operation_id'] == op_id:
                op['placement_success'] = success
                op['placement_container'] = container
                op['placement_correct'] = correct
                op['timestamp_complete'] = datetime.now().isoformat()
                if error:
                    op['errors'].append(f"Placement error: {error}")
                break
    
    def save(self):
        """Lagrer logg til fil."""
        try:
            with open(self.log_file, 'w') as f:
                json.dump(self.operations, f, indent=2)
        except Exception as e:
            print(f"ADVARSEL: Kunne ikke lagre logg: {e}")
    
    def get_statistics(self) -> dict:
        """
        Beregner statistikk for kravverifikasjon.
        
        Returns:
            Dict med statistikk for F3, F4, F7
        """
        completed_ops = [op for op in self.operations if op['placement_success'] is not None]
        
        if not completed_ops:
            return {
                'total_operations': 0,
                'pick_success_rate': 0.0,
                'placement_success_rate': 0.0,
                'correct_placement_rate': 0.0,
                'classification_placement_match_rate': 0.0
            }
        
        total = len(completed_ops)
        pick_success = sum(1 for op in completed_ops if op['pick_success'])
        placement_success = sum(1 for op in completed_ops if op['placement_success'])
        correct_placement = sum(1 for op in completed_ops if op['placement_correct'])
        
        # F7: Samsvar mellom klassifisering og sortering
        classification_match = sum(1 for op in completed_ops 
                                  if op['ball_color'] == op['placement_container'])
        
        return {
            'total_operations': total,
            'pick_success_rate': (pick_success / total * 100) if total > 0 else 0,
            'placement_success_rate': (placement_success / total * 100) if total > 0 else 0,
            'correct_placement_rate': (correct_placement / total * 100) if total > 0 else 0,
            'classification_placement_match_rate': (classification_match / total * 100) if total > 0 else 0
        }

def main():
    print("=== Autonomia Robot Control System (Fase 1: Benk-test) ===")
    print(f"Konfigurasjon: {config.NUM_JOINTS} akser.")
    print(f"Modus: {'MOCK/SIMULERING' if config.MOCK_MODE else 'LIVE HARDWARE'}")
    
    # Initialiser modulene
    try:
        kinematics = KinematicsSolver()
        comms = CommsManager()
        logger = OperationLogger()
    except Exception as e:
        print(f"Kritisk feil under oppstart: {e}")
        sys.exit(1)

    print("\nKommandoer:")
    print("  x y z  : Flytt til koordinater (f.eks '100 50 50')")
    print("  home   : Gå til hjem-posisjon")
    print("  stats  : Vis operasjonsstatistikk")
    print("  q      : Avslutt")
    print("\nLogging aktivert - operasjoner logges til operation_log.json")

    # Hovedløkke
    while True:
        try:
            user_input = input("\nKommando > ").strip().lower()

            if user_input == 'q':
                print("Avslutter...")
                logger.save()
                print("Operasjonslogg lagret.")
                break
            
            elif user_input == 'stats':
                # Vis statistikk
                stats = logger.get_statistics()
                print("\n" + "="*60)
                print("OPERASJONSSTATISTIKK")
                print("="*60)
                print(f"Totalt operasjoner: {stats['total_operations']}")
                print(f"Plukk-suksess (F3): {stats['pick_success_rate']:.1f}% (krav: ≥90%)")
                print(f"Plassering-suksess: {stats['placement_success_rate']:.1f}%")
                print(f"Korrekt plassering (F4): {stats['correct_placement_rate']:.1f}% (krav: 100%)")
                print(f"Klassifisering↔Sortering (F7): {stats['classification_placement_match_rate']:.1f}% (krav: 100%)")
                print("="*60)
            
            elif user_input == 'home':
                # En enkel "Hjem"-posisjon. Juster disse vinklene etter behov.
                # For 3 ledd: 90, 90, 90 er ofte en trygg "L"-form.
                home_angles = [90] * config.NUM_JOINTS
                print("Går til hjem-posisjon...")
                comms.send_angles(home_angles)
            
            else:
                # Forsøk å tolke input som x, y, z koordinater
                parts = user_input.split()
                if len(parts) == 3:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])

                        print(f"Beregner IK for mål: ({x}, {y}, {z})")
                        
                        # 1. Beregn vinkler (Invers Kinematikk)
                        try:
                            target_angles = kinematics.solve_ik(x, y, z)
                        except ValueError as e:
                            print(f"FEIL: {e}")
                            continue
                        
                        # Sjekk om IK returnerte en gyldig løsning
                        print(f"Beregnet vinkler: {['{:.1f}'.format(a) for a in target_angles]}")

                        # 2. Send til Arduino
                        comms.send_angles(target_angles)

                    except ValueError:
                        print("Feil: Koordinater må være tall.")
                else:
                    print("Ukjent kommando. Skriv 'x y z', 'home', 'stats' eller 'q'.")

        except KeyboardInterrupt:
            # Håndter Ctrl+C pent
            print("\nAvbrutt av bruker.")
            logger.save()
            break
        except Exception as e:
            print(f"Uventet feil i hovedløkken: {e}")

    # Rydd opp før vi stenger
    comms.close()

if __name__ == "__main__":
    main()
