# Bachelor_prosjekt - Autonomia Robotarm

Dette repoet inneholder kontrollsystemet for robotarmen utviklet av "Autonomia" (Bachelor 2026).
Systemet er designet for å være modulært, og støtter både 3-akse og 6-akse konfigurasjoner med integrert vision for balldeteksjon og sortering.

## Systemarkitektur

*   **Høynivå (Raspberry Pi):** Python-kode som håndterer kinematikk (IK), brukerinput, seriell kommunikasjon og balldeteksjon.
*   **Lavnivå (Arduino Mega):** C++ firmware basert på FreeRTOS som styrer servoer og sikrer sanntidsytelse.
*   **Vision System:** HSV-basert fargedeteksjon for identifikasjon og lokalisering av røde og blåe baller.

## Struktur

```
.
├── firmware/
│   └── motor_controller.ino  # Arduino-kode (FreeRTOS)
├── src/
│   ├── config.py             # Systemkonfigurasjon (Antall akser, dimensjoner)
│   ├── kinematics.py         # Kinematikk-løser (IK/FK)
│   ├── comms_manager.py      # Seriell kommunikasjon
│   ├── main_rpi.py           # Hovedprogram (CLI)
│   ├── requirements.txt      # Python-avhengigheter
│   └── vision/
│       ├── ball_detection.py   # Balldeteksjonssystem
│       ├── test_ball_detection.py  # Interaktiv testprogramvare
│       ├── hsv_tuner.py        # HSV-kalibreringverktøy
│       └── README.md           # Detaljert dokumentasjon for vision
└── README.md
```

## Komme i gang

### 1. Python (Raspberry Pi / PC)

Installer avhengigheter:
```bash
pip install -r src/requirements.txt
```

Kjør programmet (standard er Mock Mode for testing uten hardware):
```bash
python3 src/main_rpi.py
```

### 2. Arduino

1.  Åpne `firmware/motor_controller.ino` i Arduino IDE.
2.  Installer bibliotekene **FreeRTOS** (av Richard Barry) og **Servo**.
3.  Last opp til Arduino Mega.

### Konfigurasjon

For å endre fra 3 til 6 akser, eller endre fysiske mål på armen:
1.  Rediger `NUM_JOINTS` og `LINK_LENGTHS` i `src/config.py`.
2.  Rediger `NUM_JOINTS` i `firmware/motor_controller.ino`.

## 🎯 Balldeteksjonssystem

Systemet inkluderer et avansert vision-system for deteksjon og lokalisering av røde og blåe baller.

### Rask Start - Vision System

**Test deteksjonen:**
```bash
python src/vision/test_ball_detection.py
```

**Tune HSV-verdier for dine lysforhold:**
```bash
python src/vision/hsv_tuner.py
```

**Bruk i kode:**
```python
from vision.ball_detection import create_default_detector
import cv2

detector = create_default_detector()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    balls = detector.detect_balls(frame)
    
    for ball in balls:
        print(f"{ball.color.value}: {ball.center}, confidence={ball.confidence:.2f}")
```

### Hovedfunksjoner - Vision

✅ **HSV-basert fargedeteksjon** - Robust under varierende lysforhold  
✅ **Morfologiske operasjoner** - Støyreduksjon og objektforbedring  
✅ **Sirkulærhetskontroll** - Filtrerer ut ikke-sirkulære objekter  
✅ **Avstandsestimering** - Beregner avstand basert på ballstørrelse  
✅ **Sanntidsytelse** - Optimalisert for Raspberry Pi  
✅ **Interaktiv tuning** - HSV-verktøy for kalibrering  

**Se [src/vision/README.md](src/vision/README.md) for fullstendig dokumentasjon.**

### Teoretisk Bakgrunn

Systemet bruker flere etablerte computer vision-teknikker:

- **HSV Color Space**: Separerer fargekomponent fra lysstyrke for robust fargedeteksjon
- **Morphological Operations**: Opening og Closing for støyreduksjon
- **Contour Analysis**: Finner og validerer sammenhengende områder
- **Circularity Check**: Sirkulærhetsberegning (4πA/P²) for formvalidering
- **Pinhole Camera Model**: Avstandsestimering basert på perspektivprojeksjon


## Test Status

### Robotarm System
Systemet er testet i simuleringsmodus (Mock Mode).
- [x] Invers Kinematikk (3-akse geometrisk)
- [x] Sjekk av rekkevidde (Range check)
- [x] Pakkegenerering for seriell protokoll

### Vision System
- [x] HSV-basert fargedeteksjon (rød og blå)
- [x] Morfologiske operasjoner for støyreduksjon
- [x] Konturdeteksjon og validering
- [x] Sirkulærhetskontroll
- [x] Kamerakalibrering og avstandsestimering
- [x] Interaktive testverktøy
- [x] Omfattende dokumentasjon

## 📚 Dokumentasjon

- **Robotarm**: Se denne README
- **Vision System**: Se [src/vision/README.md](src/vision/README.md) for detaljert teknisk dokumentasjon
- **Kode**: Alle moduler har extensive inline-kommentarer på norsk

## 👥 Team Autonomia - Bachelor 2026

Dette prosjektet er utviklet som en del av bacheloroppgaven ved [Din institusjon].

**Prosjektmål**: Utvikle en autonom robotarm med vision-basert deteksjon for sortering av fargede baller.
