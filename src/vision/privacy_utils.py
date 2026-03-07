"""
Privacy and Security Utilities
===============================

Dette modulet inneholder hjelpefunksjoner for personvern og sikkerhet
i balldeteksjonssystemet.

Author: Bachelor Project 2026 - Autonomia
"""

from typing import Optional
import sys


def request_camera_consent(auto_consent: bool = False) -> bool:
    """
    Ber om brukersamtykke før kamerabruk (GDPR-compliance).
    
    Dette er viktig for å overholde personvernlovgivning (GDPR/CCPA).
    Selv om systemet ikke lagrer bilder, må brukeren informeres om
    at kameraet brukes.
    
    Args:
        auto_consent: Hvis True, hopp over prompt (for automatisert testing)
    
    Returns:
        True hvis bruker samtykker, False ellers
    """
    if auto_consent:
        print("INFO: Automatisk kamerasamtykke aktivert (testmodus)")
        return True
    
    print("\n" + "="*70)
    print("⚠️  PERSONVERN OG KAMERABRUK")
    print("="*70)
    print()
    print("Dette programmet trenger tilgang til kameraet ditt for å")
    print("detektere røde og blåe baller.")
    print()
    print("PERSONVERNINFORMASJON:")
    print("  ✓ Ingen bilder eller video lagres til disk")
    print("  ✓ All prosessering skjer lokalt på din enhet")
    print("  ✓ Ingen data sendes til eksterne servere")
    print("  ✓ Kun ballposisjoner og farger detekteres")
    print("  ✓ Du kan avslutte når som helst med 'q'")
    print()
    print("="*70)
    
    try:
        consent = input("\nGi tillatelse til å bruke kamera? (ja/nei): ")
        return consent.lower() in ['ja', 'j', 'yes', 'y']
    except (KeyboardInterrupt, EOFError):
        print("\n\nAvbrutt av bruker")
        return False


def get_validated_float_input(prompt: str, 
                               min_value: Optional[float] = None,
                               max_value: Optional[float] = None,
                               allow_cancel: bool = True) -> Optional[float]:
    """
    Henter og validerer float-input fra bruker.
    
    Denne funksjonen beskytter mot:
    - ValueError (ugyldig input)
    - Out-of-range verdier
    - KeyboardInterrupt
    
    Args:
        prompt: Spørsmål til bruker
        min_value: Minimum tillatt verdi
        max_value: Maksimum tillatt verdi
        allow_cancel: Om bruker kan avbryte
    
    Returns:
        Validert float-verdi, eller None hvis avbrutt
    """
    while True:
        try:
            if allow_cancel:
                print(f"\n{prompt}")
                print("(Skriv 'avbryt' eller trykk Ctrl+C for å avbryte)")
                user_input = input("> ")
            else:
                user_input = input(f"\n{prompt}\n> ")
            
            # Sjekk for avbryt
            if allow_cancel and user_input.lower() in ['avbryt', 'cancel', 'q', 'quit']:
                print("Avbrutt av bruker")
                return None
            
            # Konverter til float
            value = float(user_input)
            
            # Valider område
            if min_value is not None and value < min_value:
                print(f"❌ FEIL: Verdien må være minst {min_value}")
                continue
            
            if max_value is not None and value > max_value:
                print(f"❌ FEIL: Verdien må være maks {max_value}")
                continue
            
            return value
        
        except ValueError:
            print("❌ FEIL: Ugyldig input. Skriv inn et gyldig tall.")
        except (KeyboardInterrupt, EOFError):
            if allow_cancel:
                print("\n\n⚠️  Avbrutt av bruker")
                return None
            else:
                print("\n\n❌ Avbryt ikke tillatt i denne konteksten")


def get_validated_int_input(prompt: str,
                            min_value: Optional[int] = None,
                            max_value: Optional[int] = None,
                            allow_cancel: bool = True) -> Optional[int]:
    """
    Henter og validerer integer-input fra bruker.
    
    Args:
        prompt: Spørsmål til bruker
        min_value: Minimum tillatt verdi
        max_value: Maksimum tillatt verdi
        allow_cancel: Om bruker kan avbryte
    
    Returns:
        Validert integer-verdi, eller None hvis avbrutt
    """
    result = get_validated_float_input(prompt, min_value, max_value, allow_cancel)
    
    if result is None:
        return None
    
    return int(result)


def validate_camera_index(camera_index: int) -> bool:
    """
    Validerer at en kameraindeks er gyldig.
    
    Args:
        camera_index: Indeks å validere
    
    Returns:
        True hvis gyldig, False ellers
    """
    if not isinstance(camera_index, int):
        return False
    
    if camera_index < 0 or camera_index > 10:  # Realistisk maks 10 kameraer
        return False
    
    return True


def safe_file_path(filename: str, allowed_extensions: list = None) -> bool:
    """
    Validerer at et filnavn er trygt å bruke.
    
    Beskytter mot:
    - Path traversal (../)
    - Absolutte stier
    - Farlige tegn
    
    Args:
        filename: Filnavn å validere
        allowed_extensions: Liste med tillatte filendelser (f.eks. ['.txt', '.json'])
    
    Returns:
        True hvis trygt, False ellers
    """
    import os
    
    # Sjekk for path traversal
    if '..' in filename or filename.startswith('/') or filename.startswith('\\'):
        return False
    
    # Sjekk for absolutt path (Windows)
    if ':' in filename:
        return False
    
    # Sjekk filendelse hvis spesifisert
    if allowed_extensions:
        _, ext = os.path.splitext(filename)
        if ext.lower() not in [e.lower() for e in allowed_extensions]:
            return False
    
    return True


def limit_detections(detections: list, max_count: int = 100) -> tuple:
    """
    Begrenser antall deteksjoner for å forhindre resource exhaustion.
    
    Args:
        detections: Liste med deteksjoner
        max_count: Maksimum antall deteksjoner å returnere
    
    Returns:
        Tuple: (begrenset liste, bool som indikerer om noe ble kuttet)
    """
    if len(detections) > max_count:
        return detections[:max_count], True
    return detections, False


# GDPR/Personverndokumentasjon
PRIVACY_POLICY = """
PERSONVERNERKLÆRING - Balldeteksjonssystem
==========================================

1. DATABEHANDLING
   - Systemet prosesserer live video fra kamera
   - Ingen bilder eller video lagres permanent
   - Kun ballposisjoner og farger ekstraheres

2. DATALAGRING
   - Ingen persondata lagres
   - Kun tekniske kalibreringsdata (HSV-verdier) lagres lokalt
   - Ingen data sendes til eksterne servere

3. DINE RETTIGHETER
   - Du kan når som helst avslutte programmet (trykk 'q')
   - Du kan nekte kamerabruk ved oppstart
   - All data slettes når programmet avsluttes

4. FORMÅL
   - Systemet brukes kun for deteksjon av fargede baller
   - Dette er et bachelor-prosjekt for akademiske formål

5. KONTAKT
   - For spørsmål om personvern, kontakt prosjektteamet

Sist oppdatert: Mars 2026
"""


def print_privacy_policy():
    """Skriver ut personvernerklæringen"""
    print(PRIVACY_POLICY)


if __name__ == "__main__":
    """Test av sikkerhetsfunksjoner"""
    print("=== TEST AV SIKKERHETSFUNKSJONER ===\n")
    
    # Test 1: Kamerasamtykke
    print("Test 1: Kamerasamtykke")
    consent = request_camera_consent(auto_consent=True)
    print(f"Resultat: {consent}\n")
    
    # Test 2: Validert float input
    print("Test 2: Validert float input (auto-test)")
    # I test, kan ikke be om input, så hopper over
    
    # Test 3: Filnavn-validering
    print("Test 3: Filnavn-validering")
    test_files = [
        ("config.txt", True),
        ("../etc/passwd", False),
        ("C:\\Windows\\System32\\file.txt", False),
        ("normal_file.json", True),
    ]
    
    for filename, expected in test_files:
        result = safe_file_path(filename, ['.txt', '.json'])
        status = "✓" if result == expected else "✗"
        print(f"  {status} {filename}: {result}")
    
    print("\n=== ALLE TESTER FULLFØRT ===")
