# Lysforhold og Robusthet i Balldeteksjon

Dette dokumentet forklarer hvordan balldeteksjonssystemet håndterer varierende lysforhold som innelys, dagslys, skygger og refleksjoner.

---

## 🌞 OVERSIKT: LYSHÅNDTERING

Balldeteksjonssystemet har **to nivåer** av lyshåndtering:

### Nivå 1: HSV Fargerom (Alltid Aktivt) ✅
- **Hue (fargetone)** er separert fra **Value (lysstyrke)**
- Gir grunnleggende robusthet mot lysvariasjoner
- Fungerer godt i **stabile lysforhold**

### Nivå 2: Adaptiv Lyshåndtering (Valgfritt, Anbefalt) ⭐
- Analyserer lysforhold i hver frame
- Justerer HSV-grenser dynamisk
- Optimaliserer for **varierende lysforhold**
- Aktiveres med `enable_adaptive_lighting=True` (standard)

---

## 📊 STØTTEDE LYSFORHOLD

| Lysforhold | Uten Adaptiv | Med Adaptiv | Anbefaling |
|------------|--------------|-------------|------------|
| **Normal innendørs** | ✅ Utmerket | ✅ Perfekt | Basic modus OK |
| **Sterkt dagslys** | ⚠️ Moderat | ✅ Meget god | Bruk adaptiv |
| **Svakt lys** | ❌ Dårlig | ✅ God | **KREV adaptiv** |
| **Mye skygger** | ⚠️ Varierer | ✅ God | Bruk adaptiv |
| **Blandet lys** | ⚠️ Ustabil | ✅ Stabil | Bruk adaptiv |
| **Refleksjoner** | ⚠️ Noe støy | ✅ Bedre | Bruk adaptiv + preprocessing |

---

## 🔬 HVORDAN DET FUNGERER

### Lysanalyse

Systemet beregner flere metriske for hver frame:

```python
1. Gjennomsnittlig lysstyrke (mean brightness)
   - Lav (<80): Mørkt rom
   - Normal (80-180): Normalt innelys
   - Høy (>180): Sterkt lys

2. Standardavvik (contrast)
   - Lav: Jevn belysning
   - Høy: Ujevn belysning, skygger

3. Kontrast-ratio (max/min)
   - Måler hvor ekstreme lysforskjeller er

4. Histogram-entropi
   - Måler kompleksitet i belysning
```

### Adaptiv Justering

Basert på detektert lysforhold justeres HSV-grenser:

```python
# Eksempel: Svakt lys
Normal:  V_min = 100  (for mørke baller ignoreres)
Adaptiv: V_min = 40   (aksepterer mørke baller)

# Eksempel: Sterkt lys
Normal:  S_min = 100  (overxposed områder forkastes)
Adaptiv: S_min = 70   (tolererer litt washet-out farger)
```

---

## 🚀 BRUK AV ADAPTIV LYSHÅNDTERING

### Grunnleggende Bruk (Anbefalt)

```python
from vision.ball_detection import create_default_detector

# Adaptiv lyshåndtering er aktivert som standard
detector = create_default_detector()

# Detekter normalt
balls = detector.detect_balls(frame)

# Sjekk lysforhold (valgfritt)
lighting_info = detector.get_lighting_info()
if lighting_info:
    print(f"Lysforhold: {lighting_info['condition'].value}")
```

### Avansert: Med Preprocessing for Dårlig Lys

```python
# For MEGET dårlige lysforhold eller ujevn belysning
detector = create_default_detector(
    enable_adaptive_lighting=True,   # Adaptiv HSV
    enable_preprocessing=True        # CLAHE enhancement
)

# Preprocessing bruker CLAHE (Contrast Limited Adaptive Histogram Equalization)
# Dette forbedrer lokal kontrast, spesielt nyttig i skygger
```

### Manuell Konfigurasjon

```python
from vision.ball_detection import BallDetector

detector = BallDetector(
    min_radius=10,
    max_radius=150,
    min_circularity=0.7,
    known_ball_diameter_cm=7.0,
    enable_adaptive_lighting=True,    # Aktiver adaptiv
    enable_preprocessing=False        # Av som standard
)
```

---

## 🧪 TESTING I FORSKJELLIGE LYSFORHOLD

### Test 1: Normal Innendørs Belysning
```python
# Adaptiv hjelper, men ikke kritisk
detector = create_default_detector(enable_adaptive_lighting=False)
# Bør fungere greit
```

### Test 2: Vindu med Dagslys (Blandet)
```python
# Adaptiv ANBEFALES sterkt
detector = create_default_detector(enable_adaptive_lighting=True)
# Vil håndtere både lyse og mørke områder
```

### Test 3: Mørkt Rom / Kveldslys
```python
# Adaptiv + Preprocessing PÅKREVD
detector = create_default_detector(
    enable_adaptive_lighting=True,
    enable_preprocessing=True
)
# CLAHE vil forsterke kontrast i mørke områder
```

### Test 4: Utendørs / Direkte Sollys
```python
# Adaptiv ANBEFALES
# OBS: Kan få problemer med ekstreme refleksjoner
detector = create_default_detector(enable_adaptive_lighting=True)

# Vurder å justere sirkulærhet for å filtre refleksjoner:
detector.min_circularity = 0.75  # Strengere
```

---

## 📈 YTELSESIMPAKT

| Modus | FPS Impact | Minnebruk | Anbefaling |
|-------|-----------|-----------|------------|
| **Basic (ingen adaptiv)** | 0% | Lavest | Kun for stabil innendørs bruk |
| **Adaptiv** | ~5-10% | +10 MB | **ANBEFALT** for alle |
| **Adaptiv + Preprocessing** | ~15-20% | +15 MB | For dårlig lys |

**Raspberry Pi 4:**
- Basic: ~25 FPS
- Adaptiv: ~22 FPS (fortsatt utmerket)
- Adaptiv + Preprocessing: ~18 FPS (akseptabelt)

---

## 🔧 FEILSØKING: LYSPROBLEMER

### Problem: Baller detekteres i normalt lys, men ikke i skygger

**Løsning 1: Aktiver adaptiv**
```python
detector = create_default_detector(enable_adaptive_lighting=True)
```

**Løsning 2: Senk V_min manuelt**
```python
detector.red_lower_1 = np.array([0, 100, 50])  # V_min = 50 (fra 100)
detector.blue_lower = np.array([100, 100, 50])
```

### Problem: Overexposed områder detekteres ikke

**Løsning: Aktiver adaptiv**
```python
# Adaptiv vil senke S_min for å fange washet-out farger
detector = create_default_detector(enable_adaptive_lighting=True)
```

### Problem: Ustabil deteksjon (flakser mellom rammer)

**Løsning: Aktiver adaptiv og NOT CLAHE**
```python
# CLAHE kan forårsake flimring
detector = create_default_detector(
    enable_adaptive_lighting=True,
    enable_preprocessing=False  # Av
)
```

### Problem: For mange falske positiver ved refleksjoner

**Løsning: Øk sirkulærhet og deaktiver preprocessing**
```python
detector = BallDetector(
    min_circularity=0.8,  # Strengere
    enable_preprocessing=False  # Ikke forsterk refleksjoner
)
```

---

## 🎓 FOR EKSAMEN: FORKLARING AV LYSHÅNDTERING

### Nøkkelpunkter

**1. Hvorfor HSV hjemper med lys?**
```
HSV separerer fargetone (Hue) fra lysstyrke (Value).
En rød ball har samme Hue i sollys og skygge.
RGB-verdier endres dramatisk med lys, HSV Hue er stabil.
```

**2. Hva gjør adaptiv lyshåndtering?**
```
Analyserer bildet:
  - Beregner gjennomsnittlig lysstyrke
  - Detekterer om det er mørkt, normalt eller sterkt lys
  - Justerer HSV-terskler dynamisk

Resultat:
  - Svakt lys → Senk V_min (aksepter mørkere baller)
  - Sterkt lys → Senk S_min (aksepter bleked farger)
  - Skygger → Bred V-range (dekk både lyst og mørkt)
```

**3. Hva er CLAHE?**
```
CLAHE = Contrast Limited Adaptive Histogram Equalization

Forbe drer lokal kontrast uten å overexpose lyse områder.
Nyttig for:
  - Mørke rom
  - Ujevn belysning
  - Skygger

OBS: Kan forsterke støy, bruk med forsiktighet
```

### Demo-Sekvens

1. **Vis basic deteksjon** under normalt lys
2. **Slukk deler av lyset** → viser at baller forsvinner
3. **Aktiver adaptiv** → viser at baller kommer tilbake
4. **Forklar:** "Adaptiv justerer V_min fra 100 til 50, som lar oss se mørkere baller"
5. **Aktiver preprocessing** → viser ytterligere forbedring i mørke
6. **Forklar:** "CLAHE forsterker lokal kontrast, gjør baller tydeligere"

---

## 📚 TEKNISKE DETALJER

### HSV-Justeringslogikk

```python
# Pseudokode for adaptiv justering
if lighting == BRIGHT_DAYLIGHT:
    V_min += 20  # Høyere terskel (lyse forhold)
    S_min -= 30  # Lavere terskel (bleked farger)

elif lighting == DIM_INDOOR:
    V_min -= 60  # Mye lavere terskel (mørke forhold)
    S_min -= 20  # Litt lavere (mindre levende farger)

elif lighting == SHADOW_HEAVY:
    V_min -= 50  # Aksepter skygger
    V_max = 255  # Men også lyse områder
    S_min -= 10  # Moderat S-justering
```

### CLAHE-Parametere

```python
clahe = cv2.createCLAHE(
    clipLimit=2.0,      # Begrens kontrastforsterkning
    tileGridSize=(8,8)  # 8x8 regioner (lokal adaptasjon)
)
```

- **clipLimit**: Hvor mye kontrast tillates (2.0 er moderat)
- **tileGridSize**: Størrelse på lokale regioner (mindre = finkornet)

---

## ✅ KONKLUSJON

| Scenario | Anbefaling |
|----------|------------|
| **Stabilt innendørs lys** | Basic modus (adaptiv av) er OK |
| **Varierende innendørs** | **Aktiver adaptiv** |
| **Vindu med dagslys** | **Aktiver adaptiv** |
| **Mørkt rom** | **Adaptiv + Preprocessing** |
| **Utendørs** | **Adaptiv** (obs: ekstreme forhold) |
| **Produksjon** | **Alltid adaptiv** for robusthet |

**Anbefalt Standard:**
```python
detector = create_default_detector()  # Adaptiv er on by default
```

---

**💡 Tips:** Kjør `python src/vision/lighting_adaptation.py` for å teste lysanalysemodulet!
