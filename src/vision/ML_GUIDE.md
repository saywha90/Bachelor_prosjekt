# Machine Learning Klassifisering - Guide

## 📋 Oversikt

Dette systemet bruker Machine Learning (CNN) for å klassifisere ballfarger i stedet for tradisjonell HSV-basert fargedeteksjon. Modellen er basert på transfer learning med MobileNetV2 og optimalisert for Raspberry Pi ved bruk av TensorFlow Lite.

## 🎯 Hvorfor ML?

### Fordeler med ML-klassifisering:
- ✅ **Mer robust mot lysvariasjoner** - Fungerer under ulike belysningsforhold
- ✅ **Bedre generalisering** - Lærer å gjenkjenne baller selv med refleksjoner eller skygger
- ✅ **Høyere nøyaktighet** - Reduserer falske positiver sammenlignet med HSV
- ✅ **Akademisk verdi** - Moderne ML-tilnærming passer godt for bachelor-prosjekt

### Sammenlignet med HSV:
| Metode | Fordeler | Ulemper |
|--------|----------|---------|
| **HSV** | Rask, ingen trening nødvendig | Sensitiv til lys, må justeres manuelt |
| **ML (CNN)** | Robust, generaliserer godt | Krever treningsdata og GPU for trening |

## 🚀 Kom i gang

### 1. Installer avhengigheter

```bash
pip install -r requirements.txt
```

Dette installerer:
- TensorFlow (for ML-modell)
- scikit-learn (for evaluering)
- matplotlib & seaborn (for visualisering)

### 2. Samle treningsdata

Bruk det medfølgende verktøyet for å samle bilder av baller:

```bash
python src/vision/collect_training_data.py
```

**Kontroller:**
- `SPACE` - Ta bilde og lagre
- `C` - Bytt mellom klasser (rød → blå → grønn)
- `Q` - Avslutt

**Tips for datai nnsamling:**
- Samle minst **50-100 bilder per klasse**
- Variere lysforhold (sollys, kunstig lys, skygge)
- Variere vinkler og avstander
- Inkluder refleksjoner og bakgrunner
- Ta bilder både av enkeltballer og flere samtidig

### 3. Tren modellen

Når du har samlet nok data, tren modellen:

```bash
python src/vision/train_model.py --data_dir training_data --epochs 20
```

**Argumenter:**
- `--data_dir` : Mappe med treningsdata (default: training_data)
- `--epochs` : Antall treningsepoker (default: 20)
- `--batch_size` : Batch-størrelse (default: 32)
- `--learning_rate` : Læringshastighet (default: 0.001)
- `--model_name` : Navn på modell (default: ball_classifier)

**Output:**
Modellen lagres i `models/` mappen:
- `ball_classifier.h5` - Full Keras modell (større fil)
- `ball_classifier.tflite` - Optimalisert for Raspberry Pi (mindre, raskere)
- `ball_classifier_training_history.png` - Graf over treningsforløp
- `ball_classifier_confusion_matrix.png` - Confusion matrix

### 4. Bruk modellen

Modellen brukes automatisk når du kjører balldeteksjonen:

```python
from vision.ball_detection import create_default_detector

# Opprett detektor med ML aktivert (default)
detector = create_default_detector(use_ml=True)

# Detekter baller
balls = detector.detect_balls(frame)
```

**For å deaktivere ML og bruke HSV:**
```python
detector = create_default_detector(use_ml=False)
```

## 📊 Modellarkitektur

### Transfer Learning med MobileNetV2

```
Input (224x224x3 RGB)
        ↓
MobileNetV2 Base (pre-trained på ImageNet)
        ↓
Global Average Pooling
        ↓
Dropout (0.2)
        ↓
Dense (3 klasser: rød, blå, grønn)
        ↓
Softmax (output: sannsynligheter)
```

**Hvorfor MobileNetV2?**
- Optimalisert for mobile devices (Raspberry Pi)
- Liten modell-størrelse (~9 MB)
- Rask inferens (~50ms per bilde på RPi)
- God balanse mellom nøyaktighet og hastighet

## 🔧 Avansert bruk

### Manuell bruk av ML-klassifiserer

```python
from vision.ml_classifier import MLBallClassifier
import cv2

# Last modell
classifier = MLBallClassifier('models/ball_classifier.tflite')

# Klassifiser et bilde
image = cv2.imread('ball.jpg')
color, confidence = classifier.predict(image)

print(f"Farge: {color.value}, Konfidens: {confidence:.2f}")

# Få alle sannsynligheter
probs = classifier.get_class_probabilities(image)
print(probs)  # {'red': 0.95, 'blue': 0.03, 'green': 0.02}
```

### Fine-tuning av eksisterende modell

Hvis du ønsker å forbedre en eksisterende modell med mer data:

1. Legg til nye bilder i `training_data/`
2. Last eksisterende modell og fortsett trening:

```python
from tensorflow import keras

# Last eksisterende modell
model = keras.models.load_model('models/ball_classifier.h5')

# Fortsett trening (implementer selv)
```

### Konverter modell til TFLite manuelt

```python
from vision.ml_classifier import convert_to_tflite

convert_to_tflite('models/ball_classifier.h5', 'models/optimized.tflite')
```

## 📈 Evaluering

### Treningsmetriske

Sjekk treningshistorikken i `models/ball_classifier_training_history.png`:
- **Accuracy**: Hvor ofte modellen klassifiserer riktig
- **Loss**: Modellens feil (lavere = bedre)
- **Val Accuracy**: Nøyaktighet på valideringsdata (indikerer generalisering)

**God modell:**
- Val Accuracy > 95%
- Ingen stor forskjell mellom Train og Val Accuracy (indikerer ikke overfitting)

### Confusion Matrix

Sjekk `models/ball_classifier_confusion_matrix.png` for å se hvor modellen feiler:
- Diagonal: Riktige klassifiseringer
- Off-diagonal: Feilklassifiseringer

**Eksempel:**
```
        Predicted
        R  B  G
Actual R [95  3  2]
       B [ 2 96  2]
       G [ 1  1 98]
```

## 🐛 Feilsøking

### Problem: "TensorFlow ikke installert"
**Løsning:**
```bash
pip install tensorflow
```

### Problem: "Modell-fil ikke funnet"
**Løsning:** Tren modellen først:
```bash
python src/vision/train_model.py
```

### Problem: "For lite treningsdata"
**Løsning:** Samle minst 50 bilder per klasse:
```bash
python src/vision/collect_training_data.py
```

### Problem: "Lav val_accuracy (<80%)"
**Mulige årsaker:**
- For lite treningsdata → Samle mer data
- For lite variasjon i data → Variere lys, vinkler, bakgrunner
- For mange epoker (overfitting) → Reduser epochs til 15-20

### Problem: "Treg inferens på Raspberry Pi"
**Løsning:** Bruk TFLite modell:
```python
classifier = MLBallClassifier('models/ball_classifier.tflite', use_tflite=True)
```

## 📚 Teoretisk bakgrunn (Bachelor-rapport)

### Convolutional Neural Networks (CNN)

CNN er spesialdesignet for bildegjenkjenning:
- **Convolutional Layers**: Ekstraherer features (kanter, former, farger)
- **Pooling Layers**: Reduserer dimensjonalitet
- **Dense Layers**: Klassifisering basert på ekstraherte features

### Transfer Learning

I stedet for å trene en modell fra scratch, bruker vi transfer learning:
1. Start med en modell pre-trent på ImageNet (1.4M bilder, 1000 klasser)
2. Fjern siste lag (klassifisering)
3. Legg til nye lag for vårt problem (3 klasser: rød, blå, grønn)
4. Tren kun de nye lagene

**Fordeler:**
- Raskere trening (timer i stedet for dager/uker)
- Krever mindre data (100 bilder vs 10,000+)
- Bedre generalisering

### Data Augmentation

For å øke variasjonen i treningsdata, bruker vi augmentation:
- Rotasjon (±20°)
- Zoom (±20%)
- Horisontal flipping
- Lysstyrkejustering (±20%)

Dette gjør modellen mer robust mot variasjoner i virkelige bilder.

## 🎓 For bachelor-rapporten

### Eksperimenter du kan kjøre

1. **Sammenligning HSV vs ML**
   - Mål nøyaktighet på testdata
   - Test under ulike lysforhold
   - Sammenlign inferens-hastighet

2. **Optimalisering for Raspberry Pi**
   - Sammenlign TFLite vs full modell
   - Mål FPS på RPi

3. **Robusthet-testing**
   - Test med nye bakgrunner
   - Test med refleksjoner
   - Test med delvis okkluderte baller

### Referanser for rapport

- MobileNetV2: [Sandler et al., 2018](https://arxiv.org/abs/1801.04381)
- Transfer Learning: [Pan & Yang, 2010](https://ieeexplore.ieee.org/document/5288526)
- TensorFlow Lite: [TensorFlow Documentation](https://www.tensorflow.org/lite)

---

**Lykke til med ML-klassifiseringen! 🚀**
