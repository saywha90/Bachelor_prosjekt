"""
Training Script for Ball Classifier
====================================

Dette scriptet trener en ML-modell for ballklassifisering ved bruk av
transfer learning med MobileNetV2.

Bruk:
    python train_model.py --data_dir ./training_data --epochs 20

Mappestruktur for treningsdata:
    training_data/
        red/
            ball_red_001.jpg
            ball_red_002.jpg
            ...
        blue/
            ball_blue_001.jpg
            ball_blue_002.jpg
            ...
        green/
            ball_green_001.jpg
            ball_green_002.jpg
            ...

Author: Bachelor Project 2026 - Autonomia
"""

import argparse
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

try:
    import tensorflow as tf
    from tensorflow import keras
    from tensorflow.keras import layers
    from tensorflow.keras.preprocessing.image import ImageDataGenerator
except ImportError:
    print("FEIL: TensorFlow er ikke installert.")
    print("Installer med: pip install tensorflow")
    sys.exit(1)

# Legg til parent directory i path
sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.ml_classifier import create_base_model, convert_to_tflite


def setup_data_augmentation():
    """
    Setter opp data augmentation for å øke variasjonen i treningsdata.
    
    Data augmentation hjelper modellen å generalisere bedre ved å lage
    modifiserte versjoner av treningsbildene (rotasjon, zoom, flip, etc.)
    """
    train_datagen = ImageDataGenerator(
        rescale=1./255,              # Normaliser pikselverdier
        rotation_range=20,           # Roter opp til 20 grader
        width_shift_range=0.2,       # Shift horisontalt
        height_shift_range=0.2,      # Shift vertikalt
        shear_range=0.2,             # Shear transformation
        zoom_range=0.2,              # Zoom inn/ut
        horizontal_flip=True,        # Flip horisontalt
        brightness_range=[0.8, 1.2], # Variere lysstyrke
        fill_mode='nearest',         # Fyll tomme områder
        validation_split=0.2         # 20% til validering
    )
    
    # Validering bruker bare rescaling (ingen augmentation)
    val_datagen = ImageDataGenerator(
        rescale=1./255,
        validation_split=0.2
    )
    
    return train_datagen, val_datagen


def load_data(data_dir: str, img_size=(224, 224), batch_size=32):
    """
    Laster trenings- og valideringsdata fra mapper.
    
    Args:
        data_dir: Sti til mappe med treningsdata
        img_size: Størrelse å resize bilder til
        batch_size: Antall bilder per batch
        
    Returns:
        Tuple med (train_dataset, val_dataset, class_names)
    """
    data_path = Path(data_dir)
    
    if not data_path.exists():
        raise FileNotFoundError(f"Data-mappe ikke funnet: {data_dir}")
    
    # Finn klasser (sub-mapper)
    class_names = sorted([d.name for d in data_path.iterdir() if d.is_dir()])
    
    if len(class_names) == 0:
        raise ValueError(f"Ingen klasse-mapper funnet i {data_dir}")
    
    print(f"Funnet {len(class_names)} klasser: {class_names}")
    
    # Tell bilder per klasse
    for class_name in class_names:
        class_path = data_path / class_name
        num_images = len(list(class_path.glob('*.jpg'))) + len(list(class_path.glob('*.png')))
        print(f"  {class_name}: {num_images} bilder")
    
    # Setup data generators
    train_datagen, val_datagen = setup_data_augmentation()
    
    # Last treningsdata
    train_generator = train_datagen.flow_from_directory(
        data_dir,
        target_size=img_size,
        batch_size=batch_size,
        class_mode='categorical',
        subset='training',
        shuffle=True
    )
    
    # Last valideringsdata
    val_generator = val_datagen.flow_from_directory(
        data_dir,
        target_size=img_size,
        batch_size=batch_size,
        class_mode='categorical',
        subset='validation',
        shuffle=False
    )
    
    return train_generator, val_generator, class_names


def plot_training_history(history, save_path=None):
    """
    Plotter trenings- og valideringsmetrikker.
    
    Args:
        history: Keras history-objekt fra model.fit()
        save_path: Sti for å lagre plottet (None = vis bare)
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    
    # Accuracy
    ax1.plot(history.history['accuracy'], label='Train Accuracy')
    ax1.plot(history.history['val_accuracy'], label='Val Accuracy')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Accuracy')
    ax1.set_title('Model Accuracy')
    ax1.legend()
    ax1.grid(True)
    
    # Loss
    ax2.plot(history.history['loss'], label='Train Loss')
    ax2.plot(history.history['val_loss'], label='Val Loss')
    ax2.set_xlabel('Epoch')
    ax2.set_ylabel('Loss')
    ax2.set_title('Model Loss')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"✓ Treningshistorikk lagret: {save_path}")
    else:
        plt.show()
    
    plt.close()


def train_model(data_dir: str, 
                epochs: int = 20,
                batch_size: int = 32,
                learning_rate: float = 0.001,
                model_name: str = "ball_classifier"):
    """
    Trener ballklassifikasjonsmodellen.
    
    Args:
        data_dir: Sti til treningsdata
        epochs: Antall treningsepoker
        batch_size: Batch-størrelse
        learning_rate: Læringshastighet
        model_name: Navn på output-modell
    """
    print("="*70)
    print("BALL CLASSIFIER - MODEL TRAINING")
    print("="*70)
    
    # 1. Last data
    print("\n1. Laster treningsdata...")
    train_gen, val_gen, class_names = load_data(data_dir, batch_size=batch_size)
    num_classes = len(class_names)
    
    print(f"\n  Total treningsbilder: {train_gen.samples}")
    print(f"  Total valideringsbilder: {val_gen.samples}")
    
    # 2. Lag modell
    print("\n2. Bygger modell...")
    model = create_base_model(num_classes=num_classes)
    
    # 3. Kompiler modell
    print("3. Kompilerer modell...")
    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=learning_rate),
        loss='categorical_crossentropy',
        metrics=['accuracy']
    )
    
    # Vis modellstruktur
    model.summary()
    
    # 4. Setup callbacks
    callbacks = [
        # Early stopping hvis val_loss ikke forbedres
        keras.callbacks.EarlyStopping(
            monitor='val_loss',
            patience=5,
            restore_best_weights=True,
            verbose=1
        ),
        # Reduser learning rate hvis plateauer
        keras.callbacks.ReduceLROnPlateau(
            monitor='val_loss',
            factor=0.5,
            patience=3,
            verbose=1,
            min_lr=1e-7
        ),
        # Lagre beste modell underveis
        keras.callbacks.ModelCheckpoint(
            filepath=f'models/{model_name}_best.h5',
            monitor='val_accuracy',
            save_best_only=True,
            verbose=1
        )
    ]
    
    # 5. Tren modell
    print("\n4. Trener modell...")
    print(f"  Epoker: {epochs}")
    print(f"  Batch size: {batch_size}")
    print(f"  Learning rate: {learning_rate}")
    print()
    
    # Opprett models-mappe hvis den ikke finnes
    Path('models').mkdir(exist_ok=True)
    
    history = model.fit(
        train_gen,
        validation_data=val_gen,
        epochs=epochs,
        callbacks=callbacks,
        verbose=1
    )
    
    # 6. Evaluer modell
    print("\n5. Evaluerer modell...")
    val_loss, val_accuracy = model.evaluate(val_gen, verbose=0)
    print(f"  Validerings-nøyaktighet: {val_accuracy*100:.2f}%")
    print(f"  Validerings-tap: {val_loss:.4f}")
    
    # 7. Lagre final modell
    model_path = f"models/{model_name}.h5"
    model.save(model_path)
    print(f"\n✓ Modell lagret: {model_path}")
    
    # 8. Konverter til TensorFlow Lite
    print("\n6. Konverterer til TensorFlow Lite...")
    tflite_path = f"models/{model_name}.tflite"
    convert_to_tflite(model_path, tflite_path)
    
    # 9. Plot treningshistorikk
    print("\n7. Genererer treningsplot...")
    plot_path = f"models/{model_name}_training_history.png"
    plot_training_history(history, save_path=plot_path)
    
    # 10. Lag klassifiseringsrapport
    print("\n8. Genererer klassifiseringsrapport...")
    generate_classification_report(model, val_gen, class_names, model_name)
    
    print("\n" + "="*70)
    print("TRENING FULLFØRT!")
    print("="*70)
    print(f"\nModeller lagret i 'models/' mappen:")
    print(f"  • {model_name}.h5 (Full Keras modell)")
    print(f"  • {model_name}.tflite (Optimalisert for Raspberry Pi)")
    print(f"  • {model_name}_best.h5 (Beste modell fra trening)")
    print(f"\nFor å bruke modellen:")
    print(f"  from vision.ml_classifier import MLBallClassifier")
    print(f"  classifier = MLBallClassifier('models/{model_name}.tflite')")
    print()


def generate_classification_report(model, val_generator, class_names, model_name):
    """
    Genererer en detaljert klassifiseringsrapport med confusion matrix.
    """
    from sklearn.metrics import classification_report, confusion_matrix
    import seaborn as sns
    
    # Prediker på valideringsdata
    val_generator.reset()
    predictions = model.predict(val_generator, verbose=0)
    y_pred = np.argmax(predictions, axis=1)
    y_true = val_generator.classes
    
    # Skriv ut klassifiseringsrapport
    print("\nKlassifiseringsrapport:")
    print("-" * 70)
    report = classification_report(y_true, y_pred, target_names=class_names)
    print(report)
    
    # Lag confusion matrix
    cm = confusion_matrix(y_true, y_pred)
    
    # Plot confusion matrix
    plt.figure(figsize=(8, 6))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=class_names,
                yticklabels=class_names)
    plt.title('Confusion Matrix')
    plt.ylabel('True Label')
    plt.xlabel('Predicted Label')
    
    cm_path = f"models/{model_name}_confusion_matrix.png"
    plt.savefig(cm_path, dpi=150, bbox_inches='tight')
    print(f"✓ Confusion matrix lagret: {cm_path}")
    plt.close()


def main():
    """Hovedfunksjon"""
    parser = argparse.ArgumentParser(
        description='Tren ML-modell for ballklassifisering'
    )
    
    parser.add_argument(
        '--data_dir',
        type=str,
        default='training_data',
        help='Sti til mappe med treningsdata (default: training_data)'
    )
    
    parser.add_argument(
        '--epochs',
        type=int,
        default=20,
        help='Antall treningsepoker (default: 20)'
    )
    
    parser.add_argument(
        '--batch_size',
        type=int,
        default=32,
        help='Batch-størrelse (default: 32)'
    )
    
    parser.add_argument(
        '--learning_rate',
        type=float,
        default=0.001,
        help='Læringshastighet (default: 0.001)'
    )
    
    parser.add_argument(
        '--model_name',
        type=str,
        default='ball_classifier',
        help='Navn på output-modell (default: ball_classifier)'
    )
    
    args = parser.parse_args()
    
    try:
        train_model(
            data_dir=args.data_dir,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            model_name=args.model_name
        )
    except KeyboardInterrupt:
        print("\n\nTrening avbrutt av bruker.")
    except Exception as e:
        print(f"\n❌ FEIL: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
