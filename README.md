# 🤖 Autonomia Robotarm - Ballsortering med Vision

**Bachelor 2026 - Intelligent balldeteksjon for robotarm**

Dette prosjektet løser problemet med å detektere og klassifisere røde og blå baller under varierende lysforhold for en autonom robotarm. Systemet kombinerer Raspberry Pi (high-level kontroll), Arduino Mega (low-level servo-kontroll), og et kamera-basert vision-system.

### Siste testresultater

> ⚠️ **Merk:** All testing er gjort på Mac (utviklingsmaskin med webcam). Raspberry Pi 5 er valgt som målplatform, men er ikke satt opp ennå. Ytelsestall for Pi 5 oppdateres etter hardware-integrasjon.

| Metrikk | Resultat | Testet på |
|---------|----------|----------|
| Rød ball deteksjon | ~98–100 % | Mac (webcam) |
| Blå ball deteksjon | ~95–100 % | Mac (webcam) |
| Gjennomsnitt baller/frame | ~2.00 (nøyaktig 1 rød + 1 blå) | Mac (webcam) |
| FPS | ~18–19 | Mac (webcam) |
| FPS på Raspberry Pi 5 | Ikke målt ennå | — |
| Vurdering | ✅ EXCELLENT | Mac |

---

## 📖 Prosessen - Hva vi prøvde og hvorfor

### ❌ Forsøk 1: ML-basert klassifisering (FEILET)

**Hva vi gjorde:**
- Trente en CNN (Convolutional Neural Network) med MobileNetV2 transfer learning
- Samlet 202 treningsbilder (97 røde, 105 blå)
- Brukte data augmentation (rotation, zoom, brightness)
- Konverterte til TensorFlow Lite for Raspberry Pi

**Hvorfor det feilet:**
1. **For lite treningsdata** - CNN krever tusenvis av bilder for å generalisere godt
2. **For lik bakgrunn** - Alle bilder tatt i samme miljø → nettverket lærte bakgrunnen, ikke ballen
3. **Dårlig ytelse** - Inference tid var for høy (~200-500ms per frame)
4. **Lav nøyaktighet** - Modellen klassifiserte feil når lysforholdene endret seg
5. **Overfitting** - Fungerte bra på treningsdata, dårlig på nye bilder

**Konklusjon:** ML er overkill for dette problemet. Vi trenger ikke "intelligens", vi trenger pålitelig fargedeteksjon.

---

### ❌ Forsøk 2: Kompleks vision pipeline (OVERENGINEERED)

**Hva vi gjorde:**
- EnhancedBallDetector med ~800 linjer kode
- Hånddeteksjon (skin detection) for å unngå å detektere ballen når den holdes
- Bevegelsesdeteksjon (motion detection) for å skille statiske vs dynamiske objekter
- Kalman Filter tracking for å følge baller over tid
- Tung preprocessing (bilateral filter, morphological operations)

**Hvorfor det feilet:**
1. **Unødvendig kompleksitet** - Robotarmen trenger kun å vite: "Er det en rød eller blå ball her?"
2. **Falske negative** - Hånddeteksjon filtrerte ut baller vi skulle detektere
3. **Performance issues** - For mye prosessering → lav FPS
4. **Vanskelig å debugge** - For mange moving parts
5. **Fragil** - Mange edge cases der systemet feilet

**Feedback fra testing:** "Det detekterer mye på rødt men IKKE den røde ballen"

**Konklusjon:** KISS (Keep It Simple, Stupid). Vi kompliserte problemet unødvendig.

---

### ⚠️ Forsøk 3: SVM-klassifikator (tilgjengelig, men ikke primær)

**Hva vi bygde:**
- HSV-histogram + SVM (Support Vector Machine) som alternativ fargeklassifikator
- Trente på 63 bilder (33 røde, 30 blå) — webcam-bilder ~100×115 px
- `scikit-learn` med `StandardScaler + SVC(kernel=rbf, C=10)`
- 100-dimensjonal feature-vektor: HSV-histogram (H:36 bins, S:32 bins, V:32 bins)

**Resultat:** 95,1 % ± 6,6 % kryssvaliderings-nøyaktighet, 32 KB modell (`models/ball_color_classifier.pkl`)

**Begrensning:** `SimpleBallDetector` i `enhanced_detector.py` bruker direkte HSV-deteksjon, som er raskere og ikke krever noen ML-avhengigheter. SVM-modellen er tilgjengelig via `color_histogram_classifier.py` dersom man ønsker et alternativt klassifikasjonstrinn.

---

### ✅ Løsning: SimpleBallDetector (FUNGERER!)

**Hva vi gikk for:**
En forenklet, robust ensemble-basert detektor med kun de essensielle komponentene:

#### 1. **Kalibrert Multi-range HSV-deteksjon**
- Analyserte 18 HEIC-bilder av de faktiske ballene (34+ millioner piksler)
- Ekstraherte nøyaktige HSV-ranges spesifikt for DISSE ballene
- Ikke generiske verdier fra internett - data-drevet kalibrering

**Rød ball HSV-ranges:**
```python
# Bright (godt lys):   H: 0-11/170-179, S: 180-255, V: 149-255
# Medium (medium lys): H: 0-11/170-179, S: 150-255, V: 114-255
# Dark (dårlig lys):   H: 0-11/170-179, S: 130-255, V:  99-175
```

**Hvorfor dette fungerer:**
- HSV-fargerom separerer **farge (Hue)** fra **lysstyrke (Value)**
- Vi har 6 ranges for rød (3 lysnivåer × 2 hue-områder pga wraparound)
- Høy saturation threshold (147+) = kun mettede røde farger = færre falske positiver

#### 2. **Hough Circle Transform (geometrisk validering)**
- Detekterer sirkulære objekter uavhengig av farge
- Fungerer som en "second opinion" på HSV-resultatene
- Parameter-tuning: dp=1.2, minDist=30px, param1=50, param2=30

**Hvorfor dette fungerer:**
- Baller ER sirkler - perfekt match for Hough Transform
- Filtrerer ut ikke-sirkulære røde/blå objekter
- Robust mot lysforhold (geometri endrer seg ikke)

#### 3. **Ensemble Voting**
```python
# Hvis BÅDE HSV og Hough detekterer: confidence = 1.0 (boost)
# Hvis bare en metode detekterer: confidence = 0.6-0.7
# Threshold: 0.35 (konfigurerbar)
```

**Hvorfor dette fungerer:**
- Reduserer falske positiver dramatisk
- Høyere konfidens når metodene er enige
- Fallback til sterkeste metode ved uenighet

#### 4. **Adaptiv Lyshåndtering (300-700 lux)**

**Problem:** Lysforhold varierer (skrivebordslampe, innelys, dagslys, skygger)

**Løsning:**
```python
# Analyser hver frame:
mean_brightness = np.mean(grayscale_frame)
estimated_lux = 300 + (mean_brightness - 80) * 4.0

# LOW (300-400 lux): Aktiver CLAHE, videre HSV-ranges
# MEDIUM (400-550 lux): Standard HSV-ranges
# HIGH (550-700 lux): Strammere HSV-ranges for presisjon
```

**CLAHE (Contrast Limited Adaptive Histogram Equalization):**
- Kun aktivert ved lavt lys (300-400 lux)
- Forbedrer kontrast uten å overeksponere
- Gjør baller mer synlige i dårlig lys

**Hvorfor dette fungerer:**
- Intelligent tilpasning: kun prosesserer når nødvendig
- Dynamisk justering av HSV basert på faktisk lysforhold
- Visuell feedback (LOW/MEDIUM/HIGH) for debugging

---

## 🎯 Hvordan det fungerer nå

### Deteksjonspipeline (9 steg)

```
1. Capture frame      → Kamera (640×480 eller høyere)
2. Analyze lighting   → Estimerer lux, klassifiserer LOW/MEDIUM/HIGH
3. Apply compensation → CLAHE hvis LOW, ellers ingen preprocessing
4. Color conversion   → BGR → HSV + Grayscale
5. Gaussian blur      → Støyreduksjon (kernel 5×5)
6. HSV detection      → 6 ranges for rød, 3 for blå
7. Hough detection    → Finn sirkler geometrisk
8. Ensemble merge     → Union-Find transitiv clustering + confidence boost
9. Post-processing    → _post_merge_nms() + _limit_per_color(max=1)
```

**Nøkkeldetaljer om steg 8–9:**
- **Union-Find NMS:** Transitiv klynging — hvis A+B overlapper og B+C overlapper, merges alle tre (ikke bare nabopar)
- **`_post_merge_nms()`:** Sekundær NMS med 1,5× radius-terskel
- **`_limit_per_color()`:** Hard grense på 1 ball per farge → eliminerer gjenværende duplikater

### Kodeeksempel

```python
from src.vision.enhanced_detector import SimpleBallDetector, BallColor
import cv2

# Initialiser
detector = SimpleBallDetector(
    min_radius=10,              # Minimum ballradius (piksler)
    max_radius=150,             # Maksimum ballradius (piksler)
    confidence_threshold=0.35,  # Sikkerhet for deteksjon
    enable_adaptive_lighting=True  # Adaptiv lys (300-700 lux)
)

# Detekter
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
balls, stats = detector.detect_balls(frame)

# Resultater
for ball in balls:
    print(f"{ball.color.value} ball @ {ball.center}, "
          f"r={ball.radius:.1f}px, "
          f"conf={ball.confidence:.2f}, "
          f"method={ball.detection_method}")

# Visualiser
annotated = detector.draw_detections(frame, balls)
cv2.imshow('Detection', annotated)
```

---

## 🔧 Hvorfor denne løsningen fungerer

### 1. **Data-drevet kalibrering**
Vi analyserte DE FAKTISKE ballene vi skal detektere - ikke generiske røde/blå objekter.
- 18 HEIC-bilder → 34+ millioner piksler analysert
- Ekstraherte statistikk: mean, median, percentiler (P5, P95)
- Skapte ranges basert på faktisk data, ikke gjetninger

### 2. **Ensemble reduserer falske positiver**
```
HSV alene:  Mange falske positive (røde klær, blå gjenstander)
Hough alene: Mange falske positive (runde ikke-ball-objekter)  
Ensemble:    Kun objekter som er BÅDE røde/blå OG sirkulære = baller
```

### 3. **Adaptivitet gir robusthet**
- Systemet tilpasser seg lysforhold automatisk
- Samme kode fungerer i ulike miljøer uten rekonfigurering
- CLAHE aktiveres intelligent kun når nødvendig

### 4. **Enkelhet = pålitelighet**
- ~650 linjer kode (ned fra ~800)
- Færre moving parts = mindre som kan gå galt
- Lett å debugge: visuell feedback viser lysnivå, deteksjonsmetode, confidence

### 5. **Performance**
- ~18–19 FPS på Mac (utviklingsmaskin, webcam 1280×720)
- Raspberry Pi 5 er valgt som målplatform — ytelse ikke målt ennå
- Akseptabel for robotarm-applikasjon (ikke sanntidskritisk)

---

## 🚀 Komme i gang

### Installer avhengigheter

```bash
# Opprett virtuelt miljø
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Installer OpenCV og NumPy
pip install opencv-python numpy
```

### Test balldeteksjonen (raskeste måte)

```bash
cd Bachelor_prosjekt
source venv/bin/activate
python src/vision/test_enhanced_detector.py
```

**Du ser:**
- Live kamera feed
- Rød sirkel rundt røde baller 🔴
- Blå sirkel rundt blå baller 🔵  
- Lysnivå i toppen (🟠LOW / 🟢MEDIUM / 🟡HIGH)
- Statistikk (FPS, antall baller, conf, metode)

**Trykk Q for å avslutte**

### Kjør hele systemet (med robotarm)

```bash
# Mock mode (testing uten hardware)
python src/main_rpi.py

# Med Arduino tilkoblet
python src/main_rpi.py --port /dev/ttyUSB0
```

### Arduino-firmware

1. Åpne `firmware/motor_controller.ino` i Arduino IDE
2. Installer: **FreeRTOS** (Richard Barry), **Servo**
3. Last opp til Arduino Mega 2560
4. Verifiser: Serial Monitor @ 9600 baud

---

## 📁 Prosjektstruktur

```
Bachelor_prosjekt/
├── src/
│   ├── vision/
│   │   ├── enhanced_detector.py          ⭐ SimpleBallDetector (HOVEDFIL)
│   │   ├── test_enhanced_detector.py     🧪 Live kameratest
│   │   ├── color_histogram_classifier.py 🤖 SVM-inferens (alternativ klassifikator)
│   │   ├── train_color_classifier.py     🏋️ Trener HSV+SVM-modellen
│   │   ├── recalibrate_hsv.py            🔬 Analyserer treningsbilder for kalibrering
│   │   ├── hsv_tuner.py                  🎨 Interaktiv HSV-kalibrering
│   │   └── models/                       📦 Lagringsplass for .pkl-modeller
│   ├── main_rpi.py                       🤖 Hovedprogram (Raspberry Pi)
│   ├── kinematics.py                     📐 IK/FK for robotarm
│   ├── comms_manager.py                  📡 Seriell kommunikasjon mot Arduino
│   ├── config.py                         ⚙️ Systemkonfigurasjon
│   └── requirements.txt                  📋 Python-avhengigheter
│
├── firmware/
│   └── motor_controller.ino              🎛️ Arduino FreeRTOS firmware
│
├── models/
│   └── ball_color_classifier.pkl         ⭐ SVM-klassifikator (32 KB, 95,1 % nøyaktighet)
│
├── training_data/                        📸 Treningsdata for SVM
│   ├── red/    (33 bilder — webcam-bilder ~100×115 px)
│   └── blue/   (30 bilder — webcam-bilder ~100×115 px)
│
├── raw_iphone/                           📷 Råbilder av ballene (kalibrering)
└── README.md                             📖 Denne filen
```

---

## ✅ Testing og validering

### Hva vi har validert

| Test | Resultat | Kommentar |
|------|---------|-----------|
| Rød ball deteksjon | ✅ Fungerer | Kalibrert fra 34M piksler |
| Blå ball deteksjon | ✅ Fungerer | Multi-range HSV |
| Falske positiver | ✅ Minimale | Ensemble voting filtrerer effektivt |
| Lysforhold 300 lux | ✅ Fungerer | CLAHE preprocessing aktiveres |
| Lysforhold 500 lux | ✅ Fungerer | Standard HSV ranges |
| Lysforhold 700 lux | ✅ Fungerer | Strammere ranges for presisjon |
| FPS på Mac (webcam) | ✅ ~18–19 FPS | Testet på utviklingsmaskin |
| FPS på Raspberry Pi 5 | ⏳ Ikke målt | Hardware ikke satt opp ennå |
| Overlappende baller | ⚠️ Delvis | Ensemble hjelper, men ikke perfekt |

### Kjente begrensninger

1. **Lav FPS ved ekstrem lav lux** - CLAHE preprocessing tar tid
2. **Overlappende baller** - Kan detektere som én stor ball hvis helt overlappende
3. **Ikke-standardballer** - HSV-verdiene er kalibrert for DISSE ballene - andre røde/blå baller kan ha andre HSV-profiler
4. **Ekstreme lysforhold** - Under 300 lux eller over 700 lux kan deteksjon feile

### Performance-måling

**Mac (utviklingsmaskin, webcam 1280×720):**
- Deteksjon: ~18–19 FPS
- Tid per frame: <55ms

**Raspberry Pi 5 (planlagt målplatform — ikke målt ennå):**
- Forventes raskere enn Pi 4 pga. kraftigere CPU
- Ytelse oppdateres etter hardware-integrasjon

**Optimalisering gjort:
- CLAHE kun aktivert ved LOW lux (300-400)
- Effektiv morfologisk prosessering
- Minimal Hough overhead (kun på relevante områder)

---

## 🔧 Konfigurering og tuning

### Juster confidence threshold

Hvis for mange falske deteksjoner:
```python
detector = SimpleBallDetector(confidence_threshold=0.45)  # Standard: 0.35
```

Hvis baller ikke detekteres:
```python
detector = SimpleBallDetector(confidence_threshold=0.25)
```

### Juster ballstørrelse

```python
detector = SimpleBallDetector(
    min_radius=15,    # Standard: 10 (piksler)
    max_radius=120    # Standard: 150 (piksler)
)
```

### Deaktiver adaptiv lys (for debugging)

```python
detector = SimpleBallDetector(enable_adaptive_lighting=False)
```

### Kalibrere HSV for nye baller

Hvis du bytter til andre røde/blå baller:

```bash
# Interaktiv HSV-tuning
python src/vision/hsv_tuner.py

# Eller analyser bilder av ballene
# (Krever PIL/pillow-heif for HEIC-bilder)
```

Oppdater ranges i `enhanced_detector.py`:
```python
self.red_ranges = [
    (np.array([H_min, S_min, V_min]), np.array([H_max, S_max, V_max])),
    # ... flere ranges
]
```

---

## 🐛 Feilsøking

### Problem: "Cannot open camera"

**Løsning:**
```bash
# Sjekk tilgjengelige kameraer
ls /dev/video*  # Linux
# Gi kameratillatelser (macOS: Systeminnstillinger → Kamera)
```

### Problem: "ModuleNotFoundError: No module named 'cv2'"

**Løsning:**
```bash
source venv/bin/activate
pip install opencv-python numpy
```

### Problem: Baller ikke detektert

**Debug-steg:**
1. Sjekk lysnivå-indikator på skjermen (LOW/MEDIUM/HIGH)
2. Hvis LOW og ikke detekterer → CLAHE fungerer ikke → sjekk at `enable_adaptive_lighting=True`
3. Hvis MEDIUM/HIGH og ikke detekterer → HSV-verdier feil for dine baller → kalibrer på nytt
4. Print debug-info:
```python
balls, stats = detector.detect_balls(frame)
print(f"HSV detections: {stats['hsv_detections']}")
print(f"Hough detections: {stats['hough_detections']}")
print(f"Lighting: {stats['lighting_level']}")
```

### Problem: For mange falske deteksjoner

**Løsning:**
1. Øk `confidence_threshold` (0.35 → 0.45)
2. Stram inn HSV-ranges (øk saturation minimum)
3. Juster `min_radius` og `max_radius` for å ekskludere for store/små objekter

### Problem: Lav FPS (<10)

**Løsning:**
1. Deaktiver CLAHE hvis ikke nødvendig: `enable_adaptive_lighting=False`
2. Reduser oppløsning:
```python
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

---

## 🏗️ Systemarkitektur

### Høynivå (Raspberry Pi - Python)
- **Kinematikk** - IK/FK for 3- eller 6-akse robotarm
- **Vision** - SimpleBallDetector for balldeteksjon
- **Kommunikasjon** - Seriell protokoll til Arduino
- **Logging** - Operasjonslogging og feilhåndtering

### Lavnivå (Arduino Mega - C++)
- **FreeRTOS** - Sanntids-OS for task-scheduling
- **Servo-kontroll** - PWM-styring av 3-6 servoer
- **Serial kommunikasjon** - Mottar kommandoer fra Raspberry Pi

### Dataflyt

```
1. Kamera → Raspberry Pi
2. SimpleBallDetector → "Rød ball @ (x, y)"
3. Kinematikk → Beregn joint angles
4. Serial → Send til Arduino: [θ1, θ2, θ3, ...]
5. Arduino → Styr servoer
6. Gripper → Plukk ball
7. Kinematikk → Beregn container-posisjon
8. Serial → Send til Arduino
9. Arduino → Flytt til container
10. Gripper → Slipp ball
```

---

## 📊 Oppsummering - Hva fungerer nå

### ✅ SimpleBallDetector v1.2.0  

| Komponent | Status | Kommentar |
|-----------|--------|-----------|
| Rød ball deteksjon | ✅ ~98–100 % | Kalibrert fra 34M piksler |
| Blå ball deteksjon | ✅ ~95–100 % | Multi-range HSV |
| Duplikater | ✅ Eliminert | Union-Find NMS + limit_per_color |
| Adaptiv lys (300-700 lux) | ✅ Fungerer | CLAHE + dynamisk HSV |
| Ensemble (HSV + Hough) | ✅ Aktivt | Minimale falske positiver |
| Gjennomsnitt baller/frame | ✅ ~2.00 | Nøyaktig 1 rød + 1 blå |
| FPS på Mac (webcam) | ✅ ~18–19 | Testet på utviklingsmaskin |
| FPS på Raspberry Pi 5 | ⏳ Ikke målt | Hardware ikke satt opp ennå |
| Visuell overlay | ✅ Oppdatert | Hvit tekst, mørk boks, sortkontur |

### 🔑 Nøkkeluttak fra utviklingen

1. **ML er overkill** for dette problemet — fargedeteksjon er enklere og mer pålitelig
2. **Kompleksitet dreper** — EnhancedBallDetector (v1.1) var for kompleks og fragil
3. **Kalibrering er kritisk** — Generiske HSV-verdier fungerer ikke, må kalibreres for dine baller
4. **Ensemble reduserer falske positiver** — To metoder bedre enn én
5. **Transitiv NMS er nødvendig** — Nabopar-NMS er ikke-transitiv og gir gjenværende duplikater
6. **Hard cap per farge** — `max_balls_per_color=1` eliminerer duplikater som NMS ikke fanger
7. **Adaptivitet gir robusthet** — Samme kode i ulike lysforhold

### 🎓 Lærdommer

| Lærdom | Forklaring |
|---------|-----------|
| **KISS-prinsippet** | Keep It Simple, Stupid - ikke overengineere |
| **Valider tidlig** | Vi skulle testet ML med mindre datasett først |
| **Målinger over antakelser** | Kalibrer fra faktiske data, ikke internett-verdier |
| **Debugging-verktøy** | Visuell feedback (lysnivå, confidence) spart oss mye tid |
| **Robusthet > Accuracy** | 95% pålitelighet bedre enn 99% som feiler ved edge cases |

---

## 📄 Lisens

MIT License - se [LICENSE](LICENSE) for detaljer.

---

**Utviklet av Team Autonomia - Bachelor 2026**

*"From machine learning to simple color detection - sometimes the simplest solution is the best."*
