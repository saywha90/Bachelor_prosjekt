"""
main_rpi.py

Hovedinngangen til robotstyringsprogrammet på Raspberry Pi.
Knytter sammen kinematikk (hjernen) og kommunikasjon (nervene).

Bruker et enkelt tekstbasert grensesnitt (CLI) for Fase 1-testing.
Dette gjør det lett å teste spesifikke koordinater uten å trenge et GUI eller datasyn.
"""

import sys
import time
import config
from kinematics import KinematicsSolver
from comms_manager import CommsManager

def main():
    print("=== Autonomia Robot Control System (Fase 1: Benk-test) ===")
    print(f"Konfigurasjon: {config.NUM_JOINTS} akser.")
    print(f"Modus: {'MOCK/SIMULERING' if config.MOCK_MODE else 'LIVE HARDWARE'}")
    
    # Initialiser modulene
    try:
        kinematics = KinematicsSolver()
        comms = CommsManager()
    except Exception as e:
        print(f"Kritisk feil under oppstart: {e}")
        sys.exit(1)

    print("\nKommandoer:")
    print("  x y z  : Flytt til koordinater (f.eks '100 50 50')")
    print("  home   : Gå til hjem-posisjon")
    print("  q      : Avslutt")

    # Hovedløkke
    while True:
        try:
            user_input = input("\nKommando > ").strip().lower()

            if user_input == 'q':
                print("Avslutter...")
                break
            
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
                        target_angles = kinematics.solve_ik(x, y, z)
                        
                        # Sjekk om IK returnerte en gyldig løsning (vi vet at solve_ik kan returnere 0-verdier ved feil)
                        # Her kunne vi hatt bedre feilhåndtering i kinematics-klassen.
                        print(f"Beregnet vinkler: {['{:.1f}'.format(a) for a in target_angles]}")

                        # 2. Send til Arduino
                        comms.send_angles(target_angles)

                    except ValueError:
                        print("Feil: Koordinater må være tall.")
                else:
                    print("Ukjent kommando. Skriv 'x y z', 'home' eller 'q'.")

        except KeyboardInterrupt:
            # Håndter Ctrl+C pent
            print("\nAvbrutt av bruker.")
            break
        except Exception as e:
            print(f"Uventet feil i hovedløkken: {e}")

    # Rydd opp før vi stenger
    comms.close()

if __name__ == "__main__":
    main()
