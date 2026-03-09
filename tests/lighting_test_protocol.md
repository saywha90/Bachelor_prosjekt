# Testprotokoll for Lysforhold
## Krav T3 og NF1

### Oversikt
Denne testprotokollen dekker:
- **T3**: Test av systemet under tre nivåer av belysning
- **NF1**: Systemet skal fungere under belysning mellom 300-800 lux

### Utstyr
- **Lysmåler**: Måling av belysningsstyrke (lux)
- **Dimbare LED-paneler**: Kontrollert justering av lysnivå
- **Testsett**: 10 røde baller, 10 blå baller
- **Kamera**: Samme kamera som brukes i produksjonssystemet
- **Loggingssystem**: For registrering av alle testresultater

---

## Testoppsett

### Lysnivåer
Systemet skal testes under følgende belysningsstyrker:

| Nivå | Lysstyrke | Beskrivelse |
|------|-----------|-------------|
| 1    | 300 lux   | Minimum (NF1 nedre grense) |
| 2    | 500 lux   | Middels (normal kontorbelysning) |
| 3    | 800 lux   | Maksimum (NF1 øvre grense) |

### Miljøkontroll
- Test utføres i kontrollert miljø
- Ekstern belysning elimineres (blendere, gardiner)
- Lyskildene plasseres **over** og **på siden** av arbeidsområdet
- Lysmåler brukes for å verifisere belysningsstyrke på:
  - Arbeidsoverflaten
  - Ball-posisjoner
  - Kamerasynsfeltet

---

## Testprosedyre

### For hvert lysnivå (300, 500, 800 lux):

#### Steg 1: Kalibrering
1. Juster LED-paneler til ønsket lysstyrke
2. Verifiser med lysmåler på minst 3 posisjoner i arbeidsområdet
3. Dokumenter lysstyrke:
   ```
   Posisjon 1 (venstre): ___ lux
   Posisjon 2 (senter):  ___ lux
   Posisjon 3 (høyre):   ___ lux
   Gjennomsnitt:         ___ lux
   ```
4. Ta referansebilde for dokumentasjon

#### Steg 2: Deteksjonstest
1. Plasser 5 røde og 5 blå baller tilfeldig i arbeidsområdet
2. Kjør `test_requirements.py` for deteksjonsrate (F1)
3. Registrer:
   - Antall korrekt detekterte baller
   - Antall false positives
   - Antall false negatives
   - Gjennomsnittlig konfidensverdi

**Akseptansekriterie F1**: ≥95% deteksjonsrate

#### Steg 3: Klassifiseringstest
1. Test 10 røde baller individuelt
2. Test 10 blå baller individuelt
3. Kjør `test_requirements.py` for klassifisering (F2)
4. Registrer:
   - Antall korrekt klassifiserte
   - Gjennomsnittlig konfidensverdi
   - Feiltolkninger (rød → blå, blå → rød)

**Akseptansekriterie F2**: ≥90% klassifiseringsnøyaktighet

#### Steg 4: Ytelsestest
1. Kjør 10 komplette sorteringssykluser
2. Mål inferenstid for hvert deteksjonskall
3. Beregn p95 inferenstid

**Akseptansekriterie ML4**: p95 ≤ 1.0 sekund

#### Steg 5: Integrasjonstest
1. Kjør `end_to_end_test.py --cycles 10 --output lighting_test_{lux}_lux.json`
2. Registrer:
   - Totalt vellykkede sykluser
   - Feilrate per stage (deteksjon, plukk, plassering)
   - Gjennomsnittlig syklustid

**Akseptansekriterie F5**: 10/10 sykluser uten manuell reset

---

## Datainnsamling

### For hvert testnivå, lagre:

```json
{
  "test_id": "lighting_300_lux",
  "timestamp": "2026-03-09T12:00:00",
  "light_level_lux": 300,
  "measurements": {
    "light_positions": [295, 302, 298],
    "average_lux": 298.3
  },
  "detection_test": {
    "total_balls": 10,
    "detected": 10,
    "false_positives": 0,
    "detection_rate": 100.0,
    "avg_confidence": 0.93
  },
  "classification_test": {
    "total_classifications": 20,
    "correct": 19,
    "accuracy": 95.0,
    "confusion_matrix": {
      "red_as_red": 10,
      "red_as_blue": 0,
      "blue_as_blue": 9,
      "blue_as_red": 1
    }
  },
  "performance_test": {
    "inference_times_ms": [120, 115, 130, ...],
    "p95_ms": 145,
    "passes_ml4": true
  },
  "integration_test": {
    "total_cycles": 10,
    "successful": 10,
    "success_rate": 100.0,
    "passes_f5": true
  },
  "notes": "Lav belysning ga noe redusert konfidens, men alle krav oppfylt."
}
```

---

## Rapportering

### Testrapport
Etter fullført testing for alle tre lysnivåer, lag sammendragsrapport:

```markdown
# Testrapport: Lysforhold (T3, NF1)

## Dato: [dato]
## Tester: [navn]

### Oppsummering
| Lysnivå | F1 (≥95%) | F2 (≥90%) | ML4 (≤1s) | F5 (10/10) | Status |
|---------|-----------|-----------|-----------|------------|--------|
| 300 lux |    %      |    %      |    ms     |   /10      | ✅/❌  |
| 500 lux |    %      |    %      |    ms     |   /10      | ✅/❌  |
| 800 lux |    %      |    %      |    ms     |   /10      | ✅/❌  |

### Kravvurdering
- **T3 BESTÅTT**: Systemet testet under 3 belysningsnivåer (300/500/800 lux)
- **NF1 BESTÅTT**: Systemet fungerer innenfor 300-800 lux-området

### Observasjoner
- [Liste observerte mønstre, utfordringer, eller interessante funn]

### Konklusjon
[Oppsummering om systemet er robust nok for varierende lysforhold]
```

---

## Feilhåndtering

### Hvis deteksjonsrate < 95% ved et lysnivå:
1. Sjekk HSV-terskler i `config.py`
2. Vurder dynamisk HSV-justering basert på lysnivå
3. Test ML-modellen med treningsdata fra samme lysnivå
4. Dokumenter observasjoner og foreslåtte tiltak

### Hvis klassifisering < 90% ved et lysnivå:
1. Undersøk hvilke feilklassifiseringer som skjer
2. Vurder data augmentation i treningsdatasettet
3. Test om eksponering/gain-justering på kamera hjelper
4. Dokumenter observasjoner og foreslåtte tiltak

### Hvis inferenstid > 1.0s (p95):
1. Verifiser at TensorFlow Lite brukes (ikke full TensorFlow)
2. Test med redusert oppløsning på input
3. Profiler koden for flaskehalser
4. Dokumenter observasjoner og foreslåtte tiltak

---

## Vedlegg

### A: Kamerainnstillinger
Dokumenter kamerainnstillinger brukt under testing:
```python
# OpenCV kamerainnstillinger
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, ...)  # [dokumenter verdi]
cap.set(cv2.CAP_PROP_GAIN, ...)           # [dokumenter verdi]
```

### B: HSV-konfigurasjoner
Dokumenter HSV-terskler brukt:
```python
# Fra config.py
COLOR_RANGES = {
    'red_lower1': (0, 150, 100),
    'red_upper1': (10, 255, 255),
    ...
}
```

### C: ML-modellinfo
- **Modell**: MobileNetV2 (transfer learning)
- **Input-størrelse**: 224x224
- **Format**: TensorFlow Lite (.tflite)
- **Treningsdatasett**: [antall] bilder
- **Treningsdato**: [dato]

---

## Signatur

**Testet av**: ________________________  
**Dato**: _______________  
**Godkjent av**: ________________________  
**Dato**: _______________

---

**Versjon**: 1.0  
**Sist oppdatert**: 2026-03-09  
**Bachelor Project 2026 - Autonomia**
