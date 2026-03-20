"""
comms_manager.py

Håndterer all kommunikasjon mellom Raspberry Pi (High-level) og Arduino (Low-level).
Ansvarlig for å pakke data inn i et robust format og sende det over seriell.

Protokoll-design:
Vi bruker en enkel pakkestruktur for å sikre at data kommer frem riktig.
[START_BYTE, COUNT, VINKEL_1, ..., VINKEL_N, CRC, END_BYTE]

Dette gjør det enkelt å oppdage feil (CRC) og vite hvor pakken starter/slutter.
"""

import serial
import time
import struct
import config

class CommsManager:
    def __init__(self):
        self.mock_mode = config.MOCK_MODE
        self.serial_port = None
        
        # Protokoll-konstanter (Magic bytes)
        # 0xFF og 0xFE er valgt fordi de sjelden oppstår naturlig i små vinkelverdier,
        # men for en robust protokoll burde vi egentlig bruke COBS eller escaping.
        # For Fase 1 er dette "godt nok".
        self.START_BYTE = 0xFF
        self.END_BYTE = 0xFE

        if not self.mock_mode:
            self._connect()
        else:
            print(f"[MOCK] CommsManager startet i simuleringsmodus. Ingen porter åpnes.")

    def _connect(self):
        """Forsøker å åpne seriellporten definert i config."""
        try:
            self.serial_port = serial.Serial(
                port=config.SERIAL_PORT,
                baudrate=config.BAUD_RATE,
                timeout=config.SERIAL_TIMEOUT
            )
            # Venter litt på at Arduinoen skal restarte (DTR-linjen trigger ofte reset)
            time.sleep(2)
            print(f"Koblet til Arduino på {config.SERIAL_PORT}")
        except serial.SerialException as e:
            print(f"FEIL: Kunne ikke åpne seriellport {config.SERIAL_PORT}. Er Arduinoen koblet til?")
            print(f"Detaljer: {e}")
            # Vi krasjer ikke programmet, men setter det i en "feil-tilstand" eller tvinger mock?
            # For nå lar vi det bare være, men send_angles vil feile.

    def send_angles(self, angles):
        """
        Pakker og sender en liste med vinkler til Arduino.
        
        Args:
            angles (list): Liste med flyttall eller heltall (grader).
                           Lengden MÅ matche config.NUM_JOINTS.
        """
        
        # Type-validering
        if not isinstance(angles, (list, tuple)):
            print(f"FEIL: angles må være en liste eller tuple, fikk {type(angles)}")
            return
        
        # Sjekk at vi har riktig antall vinkler
        if len(angles) != config.NUM_JOINTS:
            print(f"FEIL: Prøver å sende {len(angles)} vinkler, men systemet er satt opp for {config.NUM_JOINTS}.")
            return
        
        # Valider at alle verdier er numeriske og finite
        try:
            import numpy as np
            if any(not np.isfinite(float(angle)) for angle in angles):
                print("FEIL: angles inneholder NaN eller Inf verdier")
                return
        except (ValueError, TypeError) as e:
            print(f"FEIL: Ikke-numeriske verdier i angles: {e}")
            return

        # Sørg for at vinklene er innenfor grensene (Clamping)
        # Dette er en siste sikkerhetssjekk før vi sender til hardware.
        safe_angles = []
        for i, angle in enumerate(angles):
            min_val, max_val = config.JOINT_LIMITS.get(i, (0, 180))
            clamped = max(min_val, min(angle, max_val))
            safe_angles.append(int(clamped)) # Arduino mottar ofte bytes eller ints

        if self.mock_mode:
            print(f"[MOCK SEND] -> {safe_angles}")
            return

        # --- Bygg Pakken ---
        packet = bytearray()
        packet.append(self.START_BYTE)
        packet.append(len(safe_angles)) # Sender antall motorer så Arduino kan verifisere

        # Legg til vinklene. 
        # Her sender vi 1 byte per vinkel (0-255 grader).
        # Hvis vi trenger mer presisjon (f.eks. 90.5 grader), må vi sende 2 bytes (short/int) per vinkel.
        # For Fase 1 holder 1 byte (1 grads oppløsning).
        for angle in safe_angles:
            packet.append(angle)

        # Beregn en enkel sjekksum (CRC - her bare summen modulo 256)
        # Dette lar Arduino sjekke om dataene ble korrupt underveis.
        checksum = sum(packet) % 256 
        packet.append(checksum)
        
        packet.append(self.END_BYTE)

        # --- Send Pakken ---
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(packet)
            except serial.SerialException as e:
                print(f"FEIL: Kunne ikke sende data til Arduino: {e}")
        else:
            print("FEIL: Seriellport er ikke åpen.")

    def close(self):
        """Rydd opp og lukk porten."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Seriellport lukket.")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - sikrer cleanup."""
        self.close()
        return False
