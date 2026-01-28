"""
config.py

Dette er konfigurasjonsfilen for robotarmen.
Her definerer vi systemkritiske konstanter som brukes på tvers av prosjektet.

Hensikten med å samle dette i én fil er å gjøre det enkelt å endre systemet 
fra 3 til 6 akser, eller endre fysiske dimensjoner uten å måtte lete gjennom 
hundrevis av linjer med kode.
"""

# ==========================================
# SYSTEM KONFIGURASJON
# ==========================================

# Bestemmer om vi kjører simulering eller kobler til faktisk hardware.
# True = Ingen seriell-port åpnes, data printes til terminal.
# False = Prøver å åpne seriell-port mot Arduino.
MOCK_MODE = True  

# Antall ledd (motorer) i robotarmen.
# Endre denne til 6 når dere oppgraderer armen.
# Dette tallet styrer lengden på datapakkene og kinematikk-logikken.
NUM_JOINTS = 3

# ==========================================
# SERIELL KOMMUNIKASJON
# ==========================================

# Porten Arduino er koblet til. 
# På Linux/Mac er dette ofte '/dev/ttyACM0' eller '/dev/ttyUSB0'.
# På Windows er det ofte 'COM3', 'COM4' etc.
SERIAL_PORT = '/dev/ttyACM0' 

# Baud rate må matche det som er satt i Arduino-koden (Serial.begin).
# 115200 er en standard hastighet som gir rask nok overføring for sanntidsstyring.
BAUD_RATE = 115200

# Timeout for lesing fra seriellporten (sekunder).
SERIAL_TIMEOUT = 1

# ==========================================
# ROBOT GEOMETRI (Kinematikk)
# ==========================================

# Lengder på robotarmens lenker (links) i millimeter.
# Dette brukes av kinematikk-modulen for å beregne posisjoner.
# Må oppdateres med faktiske mål fra CAD-modellen eller fysisk måling.
LINK_LENGTHS = {
    'L1': 100.0,  # Høyde fra base til skulder-ledd
    'L2': 150.0,  # Lengde på overarm (mellom skulder og albue)
    'L3': 150.0   # Lengde på underarm (mellom albue og håndledd/tupp)
    # Hvis vi utvider til 6 akser, kan vi legge til flere lengder her:
    # 'L4': 50.0,
    # 'L5': 50.0, ...
}

# Grenser for motorvinkler (i grader).
# For å hindre at roboten krasjer i seg selv eller overbelaster kabler.
# Format: {MotorID: (MinGrad, MaxGrad)}
JOINT_LIMITS = {
    0: (0, 180),   # Base rotasjon
    1: (0, 180),   # Skulder
    2: (0, 180),   # Albue
    3: (0, 180),   # (Eventuelt håndledd rotasjon)
    4: (0, 180),   # (Eventuelt håndledd bøy)
    5: (0, 180)    # (Eventuelt flens rotasjon)
}
