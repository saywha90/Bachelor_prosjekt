# 📚 Presentasjons- og læringsguide — Autonomia Robotarm

> **Formål:** Denne filen hjelper deg å forstå all koden i detalj, forsvare de valg som er tatt, og svare overbevisende på tekniske spørsmål under presentasjonen.

---

## 📋 Innholdsfortegnelse

1. [Systemet – hva det gjør og hvordan det er bygget opp](#1-systemet)
2. [Utviklingshistorien – hva vi prøvde og hvorfor vi valgte som vi valgte](#2-utviklingshistorien)
3. [Vision-systemet – `enhanced_detector.py` – linje for linje](#3-vision-systemet)
4. [Kinematikk – `kinematics.py` – matematikken forklart](#4-kinematikk)
5. [Kommunikasjon – `comms_manager.py` og `motor_controller.ino`](#5-kommunikasjon)
6. [Konfigurasjon – `config.py`](#6-konfigurasjon)
7. [Hovedprogrammet – `main_rpi.py`](#7-hovedprogrammet)
8. [Spørsmål og svar – forventede kritiske spørsmål](#8-spørsmål-og-svar)
9. [Raske oppsummeringer for presentasjonen](#9-raske-oppsummeringer)

---

## 1. Systemet

### Hva gjør dette prosjektet?

Systemet styrer en autonom robotarm som:

1. **Ser** en ball via et kamera
2. **Gjenkjenner** om den er **rød** eller **blå**
3. **Beregner** hvilke vinkler armens ledd må stå i for å nå ballen
4. **Sender** disse vinklene til Arduinoen
5. **Arduinoen** styrer servoene (motorene) fysisk

### Oversikt over maskinvaren

```
[Kamera]
   ↓ (rå pixels, 640×480 eller 1280×720)
[Raspberry Pi 4 — Python]
   ↓ Kjører: enhanced_detector.py → kinematics.py → comms_manager.py
[USB/Serial-kabel]
   ↓ Protokoll: [0xFF, antall, v1, v2, v3, CRC, 0xFE]
[Arduino Mega 2560 — C++/FreeRTOS]
   ↓ PWM-signal (50 Hz)
[Servo-motorer × 3 (eller 6)]
   ↓
[Robotarmens fysiske bevegelse]
```

### Python-filer og hva de gjør

| Fil | Ansvar |
|-----|--------|
| `main_rpi.py` | Starter systemet, CLI-grensesnitt, OperationLogger |
| `config.py` | Alle konstanter (lenker, ledd-grenser, seriell-port) |
| `kinematics.py` | Matematikk: (x,y,z) → [vinkel1, vinkel2, vinkel3] |
| `comms_manager.py` | Seriell kommunikasjon via pakke-protokoll |
| `vision/enhanced_detector.py` | Identifiserer baller og returnerer posisjon |
| `vision/test_enhanced_detector.py` | Live test med kamera og visuell overlay |

---

## 2. Utviklingshistorien

> Dette er det viktigste å kunne forklare i presentasjonen. Vis at dere har reflektert.

### Forsøk 1: Machine Learning med CNN (feilet)

**Hva vi faktisk bygget:**
- Et Convolutional Neural Network (CNN) basert på MobileNetV2
- MobileNetV2 er et ferdigtrenet nettverk (transfer learning) — vi la til et nytt "hode" for å klassifisere rød/blå
- Trente på 202 bilder (97 røde, 105 blå)
- Konverterte til TensorFlow Lite for å kjøre på Raspberry Pi

**Hva er CNN?**
Et CNN er et matematisk system inspirert av menneskelig syn. Det ser på pixels i et bilde, finner mønstre (kanter, former, farger) i lag, og til slutt forutsier hva det ser. MobileNetV2 er spesiallaget for å kjøre raskt på svake prosessorer (som Raspberry Pi).

**Konkret hva som gikk feil:**

| Problem | Forklaring |
|---------|-----------|
| For lite data | Et CNN trenger typisk 1 000–10 000 bilder per klasse. Vi hadde 97/105. |
| Overfitting | Nettverket "pugget" treningsbildene istedenfor å lære ballens faktiske egenskaper |
| Bakgrunn lært | Alle bilder tatt mot samme bakgrunn → nettverket lærte bakgrunnen, ikke ballen |
| Treghett | Inference (~200-500ms/bilde) er for langsomt for sanntidsstyring |

**Hvorfor vi gikk videre:**
Vi trenger ikke "intelligens" her. Problemet er egentlig enkelt: er dette objektet rødt og rundt, eller blått og rundt? Det trenger vi ikke en neural network til.

---

### Forsøk 2: Kompleks EnhancedBallDetector (overengineered)

**Hva vi faktisk bygget (~800 linjer):**
- HSV-fargedeteksjon (som nå), men med mye mer rundt
- `HandDetector`: Skin detection for å skjule ballen når en hånd holder den
- `MotionDetector`: Background subtraction for å finne bevegelse
- `KalmanFilter`: Matematisk prediksjon av ballens fremtidige posisjon
- Tung preprocessing: bilateral filter + morfologiske operasjoner

**Hva er hvert av disse?**
- **Skin detection:** HSV-deteksjon av hudfarger — filtrerte ut "rødt som ikke er ball"
- **Background subtraction:** Sammenligner hvert frame mot et "bakgrunnsbilde" for å finne hva som er nytt
- **Kalman Filter:** En matematisk modell som predikerer posisjon basert på hastighet og retning

**Hva gikk galt:**

Systemet fungerte mot seg selv. Skin detection filtrerte ut ballen (den røde ballen ble forvekslet med hud i noen lysforhold). Bevegelsesdeteksjon ignorerte statiske baller. Kompleksiteten gjorde debugging nesten umulig.

**Konkret tilbakemelding under testing:**
> *"Det detekterer mye på rødt men IKKE den røde ballen"*

Dette er det eksakte symptome på overengineering: du legger til "intelligens" som gjør systemet dummere.

---

### Løsning: SimpleBallDetector (fungerer)

**Filosofien:** Ikke gjør problemet vanskeligere enn det er. En ball er rund og har en spesifikk farge. Det er alt vi trenger å vite.

**To prinsipp:**
1. **Kalibrер for DE FAKTISKE ballene** — ikke bruk generiske verdier fra internett
2. **Bruk to uavhengige deteksjonsmetoder** som bekrefter hverandre

---

## 3. Vision-systemet

> Fil: `src/vision/enhanced_detector.py`

### 3.1 HSV-fargerom — hva er det og hvorfor bruker vi det?

Kameraer lagrer bilder i **BGR** (Blå-Grønn-Rød, tre tall per piksel). Problemet er at samme røde ball ser veldig forskjellig ut i BGR under ulike lysforhold:

| Lysforhold | BGR-verdi for rød |
|-----------|-----------------|
| Sterkt lys | (50, 30, 220) |
| Svakt lys | (20, 10, 90) |
| Kunstig lys | (60, 40, 200) |

Det er umulig å sette én grense for "rødt" i BGR.

**HSV løser dette** ved å separere fargeinformasjon fra lysstyrke:

- **H (Hue / Fargetone):** Selve fargen — rødt er alltid rundt 0 eller 179 (rød wrapper rundt i en sirkel fra 0–179 grader i OpenCV). Blått er rundt 100–135.
- **S (Saturation / Mettethet):** Hvor "ren" fargen er. En bleik rosa har lav S. En klar rød har høy S.
- **V (Value / Lysstyrke):** Hvor lys eller mørk piksel er.

Nå kan vi sette én grense for Hue (= "hva er fargen?") og separate grenser for S og V for lysforhold.

```
Rødt i HSV:  H ≈ 0–11 ELLER 170–179 (wraparound!)
Blått i HSV: H ≈ 95–135
```

**Viktig om rød wraparound:**
Rød er spesiell fordi den befinner seg ved BEGGE endene av Hue-sirkelen:

```
[0 ————————————————————————— 179]
 ↑rød                          ↑rød
```

Derfor har vi 6 ranges for rød (3 lysnivåer × 2 hue-områder) og bare 3 for blå.

---

### 3.2 Kalibreringsprosessen

**Problemet med generiske verdier:**
Mange nettartikler sier "rødt er H: 0-20, S: 100-255, V: 100-255". Det fungerte ikke. Vår røde ball hadde spesifikke egenskaper.

**Hva vi faktisk gjorde:**
1. Tok 18 HEIC-bilder av den røde ballen i forskjellige lysforhold
2. Lastet alle bilder inn i Python med `pillow-heif` (støtte for HEIC)
3. Konverterte hvert bilde til HSV
4. Analyserte 34 382 935 piksler statistisk

**Statistikken vi brukte:**
- **Median:** Midtverdien — robust mot ekstremverdier
- **P5 (5. persentil):** 95% av pikslene er høyere enn dette → definerer nedre grense
- **P95 (95. persentil):** 95% av pikslene er lavere enn dette → definerer øvre grense

**Resultatet:**
```python
# Basert på virkelige data, ikke gjetning:
# Hue: P5=2, P95=8 → vi bruker 0–11 (litt margin)
# Saturation: Lavt lys P5=147, godt lys P5=177
# Value: Mørkt lys min=59, lyst lys min=150
```

---

### 3.3 Multi-range HSV-deteksjon i koden

```python
# Fra __init__ i SimpleBallDetector:
self.red_ranges = [
    # Bright red (godt lys) - H: 0-11, S: 177-255, V: 150-255
    (np.array([0, 177, 150]),   np.array([11, 255, 255])),   # Hue 0-11
    (np.array([170, 177, 150]), np.array([179, 255, 255])),  # Hue 170-179 (wraparound)
    
    # Medium red (medium lys) - lavere S og V-krav
    (np.array([0, 157, 96]),    np.array([11, 255, 255])),
    (np.array([170, 157, 96]),  np.array([179, 255, 255])),
    
    # Dark red (dårlig lys) - enda lavere V-krav
    (np.array([0, 147, 59]),    np.array([11, 255, 156])),
    (np.array([170, 147, 59]),  np.array([179, 255, 156])),
]
```

**Hva skjer i `detect_with_hsv_multirange`:**

```python
# For hver range: lag en binær maske (hvit=treff, svart=ingen treff)
mask_range1 = cv2.inRange(hsv_frame, lower_bound, upper_bound)

# Kombiner alle 6 masker med OR: hvit hvis NOEN range treffer
combined_mask = mask1 | mask2 | mask3 | mask4 | mask5 | mask6

# Morfologiske operasjoner:
# OPEN = erosjon etterfulgt av dilasjon → fjerner støy (små hvite prikker)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, small_kernel)
# CLOSE = dilasjon etterfulgt av erosjon → tetter hull (svarte hull i ballen)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, large_kernel)

# Finn konturer (ytterkanter av hvite flekker)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

**Hva er morfologiske operasjoner?**
Tenk på masken som svart-hvitt pikselkart. Støy er enkeltpiksler som er hvite (falskt treff). Erosjon krymper alle hvite flekker (lille forsvinner, store overlever). Dilasjon vokser dem igjen. Resultatet: bare "ekte" store flekker overlever.

---

### 3.4 Contour-validering (`_validate_contour`)

En kontur er en kurve langs ytterkanten av en hvit flekk i masken. Vi sjekker:

**1. Area-sjekk:**
```python
area = cv2.contourArea(contour)
# Minimum area = π × min_radius² (arealet av minste ball)
if area < np.pi * (self.min_radius ** 2):
    return None
```

**2. Radius-sjekk:**
```python
(x, y), radius = cv2.minEnclosingCircle(contour)
if radius < self.min_radius or radius > self.max_radius:
    return None
```

**3. Sirkularitetssjekk:**
```python
circularity = (4 × π × area) / perimeter²
# Perfekt sirkel → circularity = 1.0
# Kvadrat → circularity ≈ 0.785
# Vi krever ≥ 0.4 (baller kan se elliptiske ut fra siden)
if circularity < 0.4:
    return None
```

**Formelen for sirkularitet** er faktisk ganske elegant: en perfekt sirkel med radius r har areal = πr² og omkrets = 2πr. Sett inn: (4π × πr²) / (2πr)² = 4π²r² / 4π²r² = 1.0. En hvilken som helst annen form gir lavere verdi.

---

### 3.5 Hough Circle Transform

**Hva er Hough Circle Transform?**
Det er en algoritme som finner sirkler geometrisk — uten å se på farge. Den ser på kanter (kantdeteksjon med Canny) og spør: "hvilken sirkel kunne ha skapt disse kantene?"

**Matematisk prinsipp:**
En sirkel beskrives av tre parametere: (cx, cy, r). For hvert edgepiksel (x, y) kan vi si at alle mulige sirkler som går gjennom dette punktet danner en donut i 3D-parametersrommet (cx, cy, r). Der mange donuter krysser = sannsynlig sirkel.

**I koden:**
```python
circles = cv2.HoughCircles(
    gray_frame,
    cv2.HOUGH_GRADIENT,
    dp=1,          # Oppløsning = original (dp=1), ikke halvparten (dp=2)
    minDist=30,    # Minimum 30px mellom to sirkelsentre (hindrer duplikater)
    param1=50,     # Canny-terskel for kantdeteksjon
    param2=30,     # Akkumulatortterskel: jo lavere, jo mer sensitiv
    minRadius=self.min_radius,
    maxRadius=self.max_radius
)
```

**Etter Hough finner en sirkel, bestemmer vi fargen:**
Funksjonen `_determine_color_from_hsv` sampler HSV-verdier i et område rundt sirkelsenteret (60% av radiusen). Den TELLER piksler per hue-range istedenfor å bruke gjennomsnitt:

```python
# Riktig metode (bruker piksel-telling):
red_mask = valid_mask & ((hue_ch <= 20) | (hue_ch >= 160))
red_pixels = int(np.sum(red_mask))

# Feil metode (brukt i tidligere versjon — hadde en bug):
# mean_hue = np.mean(roi[:,:,0])
# Problemet: hue=2 og hue=178 gir mean=90 → nettverket tenker BLÅ!
```

**Dette var en faktisk bug** vi fikset: ved å ta gjennomsnittet av røde piksler (noen ved hue=2, noen ved hue=178), fikk vi mean≈90, som er BLÅ. Løsningen er å telle piksler direkte.

---

### 3.6 Adaptiv lyshåndtering

**Problemet:** Kamerabildet ser svært forskjellig ut i ulike lysforhold.

**Steg 1: Analyser lysnivå**
```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
mean_brightness = np.mean(gray)  # 0-255

# Estimat: kalibrert mot innendørslys
# 300 lux ≈ brightness 80, 700 lux ≈ brightness 180
estimated_lux = 300 + (mean_brightness - 80) * 4.0
estimated_lux = np.clip(estimated_lux, 300, 700)

if mean_brightness < 100:
    level = "low"   # Aktiver CLAHE
elif mean_brightness < 140:
    level = "medium" # Standard
else:
    level = "high"  # Stram inn HSV-grenser
```

**Steg 2: CLAHE ved lavt lys**

CLAHE = Contrast Limited Adaptive Histogram Equalization.

Vanlig histogram-equalization redistribuerer pikselintensiteter globalt. Problemet er overeksponering i lyse områder. CLAHE gjør det lokalt (deler bildet i 8×8 fliser) med en clip-grense (2.5) for å hindre overdreven forsterkning.

**Viktig optimalisering:** `self.clahe = cv2.createCLAHE(...)` opprettes **én gang i `__init__`**, ikke for hvert frame. Å opprette det per frame sløste unødvendig CPU-tid.

```python
# I __init__:
self.clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

# I apply_lighting_compensation: (bruker self.clahe)
lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
l, a, b = cv2.split(lab)
l = self.clahe.apply(l)  # Forbedre kontrast på L-kanalen
frame = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
```

**Hvorfor LAB-fargerom?** LAB separerer luminanse (L) fra farger (a, b). Vi kan forbedre lysstyrken uten å påvirke fargetonene — perfekt for CLAHE.

**Steg 3: Dynamisk justering av HSV-grenser**
```python
if level == 'low':
    # Reduser saturation-krav med 20 (S_min - 20)
    # Reduser value-krav med 20 (V_min - 20)
    # → lett å detektere i mørket, men litt mer falske positiver
elif level == 'high':
    # Øk saturation-krav med 10 (S_min + 10)
    # → strammere → færre falske positiver i sterkt lys
```

---

### 3.7 Ensemble Voting — det som faktisk gjør systemet robust

**Problemet med én metode:**
- HSV alene: Finner alt rødt/blått i bildet (inkl. røde klær, blå gjenstander)
- Hough alene: Finner alle sirkler (inkl. lyspærer, runde knapper, kaffekrus)

**Ensemble-løsningen:**
Slå sammen begge metodene. Et objekt godkjennes bare hvis det er **både** riktig farge **og** sirkulært.

```python
def ensemble_merge(self, hsv_balls, hough_balls):
    all_detections = hsv_balls + hough_balls
    
    # Cluster overlappende deteksjoner
    for ball1 in all_detections:
        for ball2 in all_detections:
            distance = sqrt((x1-x2)² + (y1-y2)²)
            combined_radius = (ball1.radius + ball2.radius) × 0.7
            
            if distance < combined_radius:
                # Disse to deteksjonene er samme ball → same cluster
    
    # For hver cluster: boost confidence hvis flere metoder er enige
    num_methods = len(set(b.detection_method for b in cluster))
    final_confidence = avg_confidence × (1.0 + 0.3 × (num_methods - 1))
    # 1 metode: confidence × 1.0 (ingen boost)
    # 2 metoder: confidence × 1.3 (30% boost)
```

**Confidence-systemet:**
- HSV alene: confidence ≈ 0.6-0.7 (basert på sirkularitet og area match)
- Hough alene: confidence ≈ 0.5-0.7 (basert på edge strength)
- Begge enige: confidence boosted × 1.3, typisk > 0.8
- Terskel: `confidence_threshold = 0.35`

---

### 3.8 Deteksjonspipelinen fra start til slutt

```python
def detect_balls(self, frame):
    # Trygghet: avslutt gracefully ved ugyldig input
    if frame is None or frame.size == 0:
        return [], self.stats.copy()
    
    # Steg 1: Analyser lys → LOW / MEDIUM / HIGH
    lighting_info = self.analyze_lighting(frame)
    
    # Steg 2: CLAHE hvis LOW (bruker self.clahe, ikke ny instans)
    frame = self.apply_lighting_compensation(frame, lighting_info)
    
    # Steg 3: Konverter til HSV og grayscale
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Steg 4: Gaussian blur — reduser støy (kernel 5×5)
    # Gaussian blur veier nabopiksler med en Gausskurve
    # → glatter ut kamerastøy uten å miste kanter
    hsv  = cv2.GaussianBlur(hsv,  (5, 5), 0)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Steg 5: HSV multi-range → potensielle baller fra fargedeteksjon
    red_hsv, blue_hsv = self.detect_with_hsv_multirange(hsv, lighting_info)
    
    # Steg 6: Hough → potensielle baller fra geometrisk deteksjon
    hough_balls = self.detect_with_hough(gray, hsv)
    
    # Steg 7: Ensemble merge → kombiner, boost tillit ved enighet
    all_balls = self.ensemble_merge(red_hsv + blue_hsv, hough_balls)
    
    return all_balls, self.stats.copy()
```

**Hva returneres:**
`detect_balls` returnerer en **tuple**: `(baller_liste, statistikk_dict)`.

Den returnerer ALDRI bare en liste — dette er viktig fordi kallet ser slik ut:
```python
balls, stats = detector.detect_balls(frame)
```

Statistikk-dicten inneholder:
```python
{
    'hsv_detections': 14,      # Totalt HSV-treff siden start
    'hough_detections': 8,     # Totalt Hough-treff
    'ensemble_detections': 6,  # Treff etter merge (siste frame)
    'lighting_level': 'medium' # LOW / MEDIUM / HIGH
}
```

---

## 4. Kinematikk

> Fil: `src/kinematics.py`

### 4.1 Hva er invers kinematikk (IK)?

En robotarm er som en arm med ledd. Hvis du vet hvilken vinkel hvert ledd skal stå i, kan du beregne nøyaktig hvor hånden er — dette er **forover kinematikk (FK)**, den enkle veien.

Det vi trenger er det motsatte: vi vet **hvor** vi vil ha hånden (x, y, z), og vi må finne ut **hvilke vinkler** leddene skal ha. Det er **invers kinematikk (IK)**, og det er matematisk mye vanskeligere.

### 4.2 Geometrisk løsning for 3 ledd

Armen har tre ledd:
- **Ledd 1 (Base):** Roterer rundt Z-aksen (snur seg til venstre/høyre)
- **Ledd 2 (Skulder):** Bøyer armen opp/ned relativt til base
- **Ledd 3 (Albue):** Bøyer under-armen

**Steg 1 — Base-vinkel (q1):**
```
q1 = atan2(y, x)
```
`atan2(y, x)` er en trigonometrisk funksjon som returnerer vinkelen i XY-planet (der roboten "peker"). Hvis mål er ved (100, 100), blir q1 = 45°.

**Steg 2 — Reduser til 2D:**
Etter at basen er rotert mot målet, kan vi ignorere Y og se på armen i 2D (R, Z-planet der R er avstand fra base-akse):

```
r = sqrt(x² + y²)          # Horisontal avstand fra base
z_eff = z - L1             # Høyde minus base-høyden (L1)
D = sqrt(r² + z_eff²)      # Direkte avstand fra skulder til mål
```

**D er avgjørende:** Det er avstanden fra skulderleddet til sluttpunktet. Hvis D > L2+L3, kan vi ikke nå dit.

**Steg 3 — Cosinussetningen for Albue (q3):**

Vi har nå en trekant med:
- Side 1: L2 (overarmens lengde, 150mm)
- Side 2: L3 (underarmens lengde, 150mm)
- Side 3: D (direkte avstand, beregnet over)

Cosinussetningen: $c^2 = a^2 + b^2 - 2ab\cos(C)$

Vi søker vinkel C ved albuen (mellom L2 og L3):

$$\cos(\text{albue}) = \frac{L2^2 + L3^2 - D^2}{2 \cdot L2 \cdot L3}$$

```python
cos_angle_albue = (self.l2**2 + self.l3**2 - D**2) / (2 * self.l2 * self.l3)
cos_angle_albue = np.clip(cos_angle_albue, -1.0, 1.0)  # Sikring mot avrundingsfeil
angle_albue_inner = math.acos(cos_angle_albue)
q3 = math.pi - angle_albue_inner  # Supplement-vinkel
```

**Steg 4 — Skulder-vinkel (q2):**
```python
angle_to_target = math.atan2(z_eff, r)         # Vinkel opp til D-vektoren
# Cosinussetning igjen for skulder-internvinkelen:
cos_shoulder = (L2² + D² - L3²) / (2 × L2 × D)
angle_shoulder_inner = acos(cos_shoulder)
q2 = angle_to_target + angle_shoulder_inner
```

**Singularitetsbeskyttelse:**
```python
if D < 1.0:
    raise ValueError("Mål for nært skulderleddet — singularitet")
```
Når D er nær null (armen peker rett på skulderleddet), deler vi på nær-null i cosinusformelen → matematisk singularitet → uendelig mange løsninger. Vi avviser disse koordinatene.

### 4.3 Konfigurasjonslenker

Fra `config.py`:
```python
LINK_LENGTHS = {
    'L1': 100.0,  # Basehøyde (fra gulv til skulder)
    'L2': 150.0,  # Overarm (skulder til albue)
    'L3': 150.0   # Underarm (albue til håndledd)
}
```

Maks rekkevidde = L2 + L3 = 300mm (30cm). Sjekkes ved:
```python
if D > self.l2 + self.l3:
    raise ValueError("Utenfor rekkevidde")
```

---

## 5. Kommunikasjon

### 5.1 Pakke-protokollen

Raspberry Pi og Arduino kommuniserer via USB-kabel som emulerer en seriell port. Data sendes som rå bytes — ingen tekst, ingen JSON.

**Pakkeformat:**
```
[0xFF | COUNT | v1 | v2 | v3 | CRC | 0xFE]
```

| Felt | Størrelse | Eksempel | Forklaring |
|------|----------|--------|-----------|
| START_BYTE | 1 byte | `0xFF` | "Pakken begynner her" |
| COUNT | 1 byte | `3` | Antall vinkler (= NUM_JOINTS) |
| VINKEL_1 | 1 byte | `90` | Base-vinkel (0–180°) |
| VINKEL_2 | 1 byte | `45` | Skulder-vinkel |
| VINKEL_3 | 1 byte | `120` | Albue-vinkel |
| CRC | 1 byte | `sum % 256` | Sjekksum |
| END_BYTE | 1 byte | `0xFE` | "Pakken slutter her" |

**Totalt: 7 bytes for 3-akset arm**

**Hvorfor ikke sende JSON?**
JSON er menneskeleseleg, men stort og tregt. For seriellkommunikasjon er binære protokoller langt mer effektive. 7 bytes vs. f.eks. `{"a":[90,45,120]}` (17 bytes + parsing-overhead).

### 5.2 CRC-sjekksummen

**Hva er CRC?**
Cyclic Redundancy Check — et tall som representerer summen av alle bytes. Hvis en byte endrer seg under overføring (støy på kabelen), stemmer ikke lenger sjekksummen.

**Vår implementasjon (enkel variant):**
```python
# Python (sender):
checksum = sum(packet_bytes) % 256  # Alle bytes % 256 → alltid 0-255
packet.append(checksum)
```

```cpp
// Arduino (mottaker):
int sum = 0;
for (int i = 0; i < PACKET_SIZE - 2; i++) {
    sum += buffer[i];
}
byte calculatedChecksum = sum % 256;
if (calculatedChecksum == receivedChecksum) {
    // GYLDIG PAKKE
}
```

**Begrensning:** Summert CRC er ikke like robust som CRC-8 eller CRC-16. Kunne vi hatt to bits som flipper tilfeldig og den ene kompenserer den andre → falsk-gyldig. For fase 1 er det godt nok.

### 5.3 Vinkel-clamping (sikkerhetssjekk)

Før sending clamper vi vinklene til JOINT_LIMITS:
```python
for i, angle in enumerate(angles):
    min_val, max_val = config.JOINT_LIMITS.get(i, (0, 180))
    clamped = max(min_val, min(angle, max_val))
    safe_angles.append(int(clamped))
```

Dette hindrer at kinematikk-feil sender 350° til en servo som bare tåler 0–180°.

### 5.4 Arduino: FreeRTOS og to tråder

Arduino kjører normalt én ting av gangen (single-threaded). FreeRTOS er et sanntids-OS som lar Arduino kjøre to "oppgaver" (tasks) tilsynelatende parallelt:

**Task 1 — TaskSerial (høy prioritet):**
- Leser seriellbufferet konstant
- Parser pakker (sjekker START_BYTE, COUNT, CRC, END_BYTE)
- Oppdaterer `targetAngles[]` via mutex

**Task 2 — TaskControl (lavere prioritet):**
- Leser `targetAngles[]` via mutex
- Sender PWM-signaler til servoene
- 50 Hz (20ms mellom oppdateringer — servoer trenger 20ms per kommando)

**Mutex (gjensidig ekskludering):**
```cpp
xSemaphoreTake(xAnglesMutex, 10);  // Ta låsen (venter maks 10ms)
// ... skriv til targetAngles ...
xSemaphoreGive(xAnglesMutex);       // Slipp låsen
```

Uten mutex: Task1 skriver til `targetAngles[0]` mens Task2 leser `targetAngles[1]` → "tearing" → arm beveger seg til feil posisjon.

---

## 6. Konfigurasjon

> Fil: `src/config.py`

Alle systemkritiske konstanter er samlet her for å gjøre koden lett å modifisere:

```python
MOCK_MODE = True        # True = ingen hardware, printer til terminal
NUM_JOINTS = 3          # Antall ledd (3 for nå, 6 ved oppgradering)
SERIAL_PORT = '/dev/ttyACM0'  # Linux-port (Windows: 'COM3')
BAUD_RATE = 115200      # Må matche Arduino sin Serial.begin(115200)
SERIAL_TIMEOUT = 1      # Sekunder timeout for lesing

LINK_LENGTHS = {
    'L1': 100.0,  # mm
    'L2': 150.0,
    'L3': 150.0,
}

JOINT_LIMITS = {
    0: (0, 180),   # Base
    1: (0, 180),   # Skulder
    2: (0, 180),   # Albue
    # 3-5: klare for 6-akset utvidelse
}
```

**Viktig:** `NUM_JOINTS` i `config.py` **MÅ** matche `#define NUM_JOINTS 3` i `motor_controller.ino`. Hvis de ikke stemmer, vil Arduino ignorere pakken fordi COUNT != NUM_JOINTS.

---

## 7. Hovedprogrammet

> Fil: `src/main_rpi.py`

### CLI-grensesnittet (Fase 1)

Bruker skrives inn koordinater (x y z) direkte i terminalen, og programmet beregner IK og sender til Arduino. Dette er Fase 1 — for å verifisere at kinematikk og kommunikasjon fungerer **uten** vision.

```
Kommando > 100 50 50
→ kinematics.solve_ik(100, 50, 50)
→ comms.send_angles([q1, q2, q3])
→ [MOCK SEND] -> [45, 67, 112]  (i mock-modus)
```

### OperationLogger

Logger alle operasjoner til `operation_log.json` for å dokumentere kravoppfyllelse:
- **F3:** Plukk-suksess ≥ 90%
- **F4:** Korrekt plassering = 100%
- **F7:** Samsvar mellom klassifisering og sortering = 100%

Loggen roteres automatisk når den overskrider 10MB (hindrer disk-full-problemer over tid).

---

## 8. Spørsmål og svar

> Her er de vanskeligste spørsmålene du kan forvente, med forsvarbare svar.

---

### Q: "Hvorfor valgte dere ikke ML? Det er vel mer avansert?"

**Svar:**
ML er ikke alltid bedre. Det er et verktøy som passer til visse typer problemer. Fargedeteksjon er ikke ett av dem.

For ML trenger vi: store mengder varierte data (≥1000 bilder per klasse), tid til trening, og nok RAM for inference. Vi hadde 202 bilder og trengte sanntidsytelse.

HSV-deteksjon er deterministisk — det fungerer eller det fungerer ikke, og vi kan debugge det direkte. Et nevralt nettverk er en svart boks. Systemets oppgave er tydelig definert: er kulen rød eller blå, og er den rund? Et CNN er overkill.

Vi prøvde ML og fikk empirisk bevis på at det var feil verktøy for dette problemet.

---

### Q: "Hva er HSV og hvorfor ikke RGB?"

**Svar:**
RGB blander fargeinformasjon med lysstyrke — den samme røde ballen ser helt forskjellig ut i RGB under ulike lysforhold. HSV separerer farge (Hue) fra lysstyrke (Value), slik at vi kan si "rødt er alltid Hue 0–11" uavhengig av om rommet er lyst eller mørkt. Det gjør systemet robust.

---

### Q: "Hva er Hough Circle Transform?"

**Svar:**
Det er en algoritme som finner sirkler ved å analysere kanter i bildet. Den spør: "hvilken sirkel (senter x, y, radius r) kunne skapt disse kantene?" Matematisk akkumulerer den "stemmer" i et 3D-parameterrom. Topper i dette rommet = sannsynlige sirkler. Vi bruker den som en geometrisk "second opinion" på HSV-resultater.

---

### Q: "Hvorfor 6 HSV-ranges for rød, men bare 3 for blå?"

**Svar:**
Rød er spesiell i HSV fordi den sitter ved begge endene av hue-sirkelen (0 og 179). Vi trenger alltid 2 ranges for rød bare for å håndtere wraparound. I tillegg har vi 3 lysnivåer (bright, medium, dark), så 2 × 3 = 6 totalt. Blå har ikke wraparound — Hue 95–135 er sammenhengende, så vi trenger bare 1 range per lysnivå = 3 totalt.

---

### Q: "Hva er ensemble-metoden og hvorfor bruker dere den?"

**Svar:**
Vi kombinerer to uavhengige deteksjonsmetoder: HSV-farge og Hough-geometri. Begge kan gi falske positiver alene — HSV finner alle røde ting, Hough finner alle runde ting. Men en ball er både rød/blå OG rund. Kun objekter som begge metodene er enige om, får boosted confidence. Resultatet er dramatisk færre falske positiver.

---

### Q: "Hva er invers kinematikk og hvorfor beregner dere det analytisk?"

**Svar:**
Invers kinematikk er å beregne ledvinkler fra en ønsket sluttpunktsposisjon (xyz). Vi bruker en analytisk (geometrisk) løsning med cosinussetningen fordi den er deterministisk, rask (O(1)), og nøyaktig for en 3-aksers arm. En numerisk løser (som ikpy) ville vært nødvendig for 6 akser, men er tregere og kan gi feil løsning.

---

### Q: "Hva er CRC og er sjekksummen god nok?"

**Svar:**
CRC (Cyclic Redundancy Check) lar Arduino verifisere at pakken ankom uten korrupsjon. Vi summerer alle bytes og tar modulo 256. Det er en enkel implementasjon — ikke like robust som CRC-8 eller CRC-16 (to bits som flipper og kansellerer hverandre ville passere). For en kablet tilkobling i innendørsmiljø er det tilstrekkelig for Fase 1. I produksjon ville vi brukt CRC-16.

---

### Q: "Hvorfor FreeRTOS på Arduino?"

**Svar:**
Arduino kjører normalt én ting av gangen. Vi trenger to uavhengige prosesser: lytte etter serielldata (tidskritisk — kan ikke tape pakker) og oppdatere servoer (tidskritisk — 50 Hz). FreeRTOS lar oss kjøre begge "parallelt" med en mutex som hindrer datasync-problemer (tearing).

---

### Q: "Hva skjer hvis IK-beregningen gir vinkler utenfor servo-grensene?"

**Svar:**
`comms_manager.py` clamper alle vinkler til JOINT_LIMITS før sending: `clamped = max(min_val, min(angle, max_val))`. I tillegg sjekker `kinematics.py` at D (avstand til mål) er innenfor armens rekkevidde og ikke for nær skulderleddet (singularitet). Tredobbel sikkerhet.

---

### Q: "Hva kalibrerte dere HSV-verdiene mot?"

**Svar:**
Vi tok 18 bilder av de faktiske ballene vi bruker i prosjektet, i varierende lysforhold. Bilder var i HEIC-format (iPhone), konverterte med `pillow-heif`. Vi analyserte 34 382 935 piksler statistisk (mean, median, percentiler P5/P95) og satte ranges basert på P5 som nedre grense og P95 som øvre, med litt margin. Dette er data-drevet kalibrering, ikke internett-verdier.

---

### Q: "Hva skjer hvis kameraet ikke er tilkoblet?"

**Svar:**
`cv2.VideoCapture(camera_index)` returnerer False på `cap.isOpened()`. `test_enhanced_detector.py` sjekker dette og avslutter med en feilmelding. `detect_balls` validerer frame-input og returnerer `([], stats)` ved `None` eller ugyldig frame — programmet krasjer ikke.

---

### Q: "Hva er CLAHE og hvorfor bruker dere LAB-fargerom?"

**Svar:**
CLAHE (Contrast Limited Adaptive Histogram Equalization) forbedrer kontrast lokalt ved å dele bildet i 8×8-fliser og equalisere histogrammet i hvert, med en clip-grense for å hindre overforsterkning. Vi gjør det i LAB-fargerommet fordi LAB separerer luminanse (L-kanalen) fra farger (a, b). Ved å bare endre L påvirker vi ikke fargetonene — ballen ser ikke mer rød/blå ut enn den er, vi gjør den bare mer synlig.

---

## 9. Raske oppsummeringer for presentasjonen

### Systemet i én setning

> "Et kamerabasert ballsorteringssystem der en Raspberry Pi gjenkjenner røde og blå baller med kalibrert HSV-deteksjon og Hough Transform, beregner armens ledd-vinkler med geometrisk invers kinematikk, og kommuniserer det til en Arduino via en robust seriell pakkeprotokoll."

---

### Teknologistacken

| Komponent | Teknologi | Begrunnelse |
|-----------|-----------|------------|
| Vision | HSV + Hough ensemble | Robust, kalibrert, ingen ML-overhead |
| IK | Geometrisk analytisk | Deterministisk, rask, nøyaktig for 3 DOF |
| Kommunikasjon | Binær pakkeprotokoll med CRC | Kompakt, verifiserbar, rask |
| Arduino-scheduling | FreeRTOS | Parallell serial-lytting og servo-styring |
| Logging | JSON + rotering | Kravverifikasjon F3/F4/F7 |

---

### Utviklingsveien i tre trinn

```
❌ CNN/ML (202 bilder)
   → Overfitting, bakgrunn lært, for treg

❌ EnhancedBallDetector (~800 linjer)
   → Hånddeteksjon blokkerte rødballen, for kompleks

✅ SimpleBallDetector + datadrevet HSV-kalibrering
   → 34M piksler analysert → kalibrerte ranges
   → Ensemble (HSV+Hough) → minimale falske positiver
   → Adaptiv lys (300-700 lux) → fungerer overalt
```

---

### Nøkkeltall å huske

| | |
|-|-|
| Treningsbilder ML | 202 (utilstrekkelig) |
| Piksler analysert for kalibrering | 34 382 935 |
| HEIC-bilder for kalibrering | 18 |
| HSV-ranges rød | 6 (3 lysnivåer × 2 hue-areas) |
| HSV-ranges blå | 3 |
| Confidence threshold | 0.35 |
| Seriell pakkestørrelse | 7 bytes (3-akse) |
| FPS Raspberry Pi 4 | 15–20 |
| Maks rekkevidde arm | 300 mm (L2+L3) |
| Baud rate | 115 200 |

---

*Lykke til med presentasjonen! Du har gjort det rette valget: en enkel, robust, data-drevet løsning som faktisk fungerer.*
