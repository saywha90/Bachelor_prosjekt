# 🚀 Hurtigstart Guide - Balldeteksjon

Dette er en rask guide for å komme i gang med balldeteksjonssystemet.

## 📦 Installasjon (5 minutter)

### Steg 1: Installer avhengigheter

```bash
cd c:\Users\oleha\Documents\Bachelorprosjekt\Kameradeteksjon\Bachelor_prosjekt
pip install -r src/requirements.txt
```

Dette installerer:
- `numpy` - Numeriske operasjoner
- `opencv-python` - Computer vision bibliotek
- `pyserial` - Seriell kommunikasjon

### Steg 2: Test at kameraet fungerer

```bash
python src/vision/test_ball_detection.py
```

Du skal nå se et vindu med live video fra kameraet.

---

## 🎯 Grunnleggende Bruk

### Test 1: Enkel Deteksjon

1. Kjør testprogrammet: `python src/vision/test_ball_detection.py`
2. Hold en rød eller blå ball foran kameraet
3. Du skal se:
   - Sirkel rundt ballen
   - Fargenavn (RØD/BLÅ)
   - Konfidensverdi (0-1)

**Tastatur:**
- `q` - Avslutt
- `h` - Vis hjelpemeny
- `1-4` - Bytt visning (normal/masker)

### Test 2: HSV-Tuning (hvis fargene ikke detekteres godt)

```bash
python src/vision/hsv_tuner.py
```

1. Hold en ball foran kameraet
2. Juster trackbars til du ser en ren hvit silhuett i "Maske"-vinduet
3. Trykk `p` for å se verdiene
4. Trykk `s` for å lagre til fil

---

## 🔧 Vanlige Problemer

### Problem: "Kunne ikke åpne kamera"

**Løsning:**
```bash
# Prøv en annen kameraindeks
python src/vision/test_ball_detection.py 1
python src/vision/test_ball_detection.py 2
```

### Problem: Ingen baller detekteres

**Sjekkliste:**
1. ✅ Er ballen rødt eller blå? (Ikke rosa, oransje, lilla)
2. ✅ Er det nok lys?
3. ✅ Er ballen i fokus (ikke for nære)?
4. ✅ Prøv HSV-tuneren for å justere fargeverdier

### Problem: Mange falske deteksjoner

**Løsning:**
```python
# I koden, øk strengheten:
detector = BallDetector(
    min_circularity=0.8,  # Øk fra 0.7 til 0.8
    min_radius=15         # Øk minimum radius
)
```

---

## 💡 Neste Steg

### 1. Kalibrer for avstandsestimering

```python
# I testprogrammet:
# 1. Trykk 'c'
# 2. Følg instruksjonene
# 3. Skriv inn faktisk avstand i cm
```

### 2. Integrer i robotarm-koden

```python
# Eksempel: main_rpi.py
from vision.ball_detection import create_default_detector
import cv2

detector = create_default_detector()
cap = cv2.VideoCapture(0)

def find_ball_to_pick():
    """Finn beste ball å plukke"""
    ret, frame = cap.read()
    balls = detector.detect_balls(frame)
    
    # Filtrer høy konfidens
    good_balls = [b for b in balls if b.confidence > 0.85]
    
    if not good_balls:
        return None
    
    # Returner nærmeste
    good_balls.sort(key=lambda b: b.distance_cm or 999)
    return good_balls[0]

def get_ball_color(ball):
    """Sjekk farge for sortering"""
    if ball.color == BallColor.RED:
        return "red_container"
    else:
        return "blue_container"
```

### 3. Les full dokumentasjon

Se [src/vision/README.md](README.md) for:
- Teoretisk bakgrunn
- Detaljert API-dokumentasjon
- Evalueringsmetoder
- Avanserte brukstilfeller

---

## 📊 Forventet Ytelse

| Plattform | FPS | Latens |
|-----------|-----|--------|
| Desktop PC | 60+ | ~16ms |
| Raspberry Pi 4 | 15-25 | ~40-65ms |
| Raspberry Pi 3 | 10-15 | ~65-100ms |

**Nøyaktighet:**
- Posisjon: ±2-5 piksler
- Avstand: ±2-5 cm (innenfor 30-100 cm)
- Deteksjonsrate: >95% under gode lysforhold

---

## 🎓 For Presentasjon/Eksamen

### Nøkkelpoeng å fremheve:

1. **HSV vs RGB**: Beskriv hvorfor HSV er bedre for fargedeteksjon
2. **Morfologiske operasjoner**: Forklar opening og closing
3. **Sirkulærhet**: Vis formelen og hvorfor den er viktig
4. **Pinhole-modell**: Forklar avstandsestimering

### Demo-sekvens:

1. Kjør `test_ball_detection.py` - Vis live deteksjon
2. Trykk `2` - Vis rød maske (forklar fargefiltrering)
3. Trykk `3` - Vis blå maske
4. Trykk `1` - Tilbake til normal (vis konfidensverdi)
5. Vis kode med forklaringer

### Styrker ved løsningen:

✅ Robust (HSV håndterer varierende lys)  
✅ Teoretisk fundert (etablerte CV-teknikker)  
✅ Godt dokumentert (lett å forsvare)  
✅ Modulær (lett å utvide)  
✅ Testbar (interaktive verktøy)  

---

**Lykke til! 🎯**

*Hvis du har spørsmål, se [README.md](README.md) eller kjør `python script.py -h` for hjelp.*
