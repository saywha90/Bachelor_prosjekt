# Models Directory

Denne mappen inneholder trente ML-modeller for ballklassifisering.

## 📁 Struktur

Etter trening vil denne mappen inneholde:

```
models/
├── ball_classifier.h5              # Full Keras modell (større fil)
├── ball_classifier.tflite          # Optimalisert for Raspberry Pi
├── ball_classifier_best.h5         # Beste modell fra trening
├── training_history.png            # Graf over treningsforløp
└── confusion_matrix.png            # Confusion matrix
```

## 🚀 Hvordan trene en modell

1. Samle treningsdata først:
   ```bash
   python ../collect_training_data.py
   ```

2. Tren modellen:
   ```bash
   python ../train_model.py --data_dir ../../training_data
   ```

3. Modellene lagres automatisk her!

## 📊 Modellfilene

### ball_classifier.h5
- Full Keras modell
- Brukes hvis du trenger full nøyaktighet
- Størrelse: ~9 MB
- Inferens: ~100ms per bilde på RPi

### ball_classifier.tflite
- TensorFlow Lite (optimalisert)
- **Anbefalt for Raspberry Pi**
- Størrelse: ~3 MB
- Inferens: ~50ms per bilde på RPi

### ball_classifier_best.h5
- Beste modell fra treningsløpet
- Lagres automatisk ved høyest val_accuracy
- Backup hvis final modell ikke er optimal

## 🔍 Evaluering

### training_history.png
Viser:
- Training accuracy vs Validation accuracy
- Training loss vs Validation loss
- Hjelper med å identifisere overfitting

### confusion_matrix.png
Viser:
- Hvor mange baller som klassifiseres riktig
- Hvilke farger som forveksles
- Nyttig for å identifisere problemer

## 📥 Pre-trente modeller (valgfritt)

Hvis du har en pre-trent modell:
1. Kopier `.tflite` filen hit
2. Navn den `ball_classifier.tflite`
3. Den brukes automatisk av systemet

## ⚠️ Viktig

- **Ikke commit store modell-filer til Git**
- Legg til `.gitignore` for `*.h5` og `*.tflite`
- Del modeller via annen metode hvis nødvendig

---

**Start med å trene din første modell! 🚀**
