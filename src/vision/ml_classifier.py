"""
Machine Learning Ball Classifier
=================================

Dette modulet implementerer en CNN-basert klassifiseringsmodell for å
identifisere ballfarger ved bruk av transfer learning med MobileNetV2.

Modellen er optimalisert for Raspberry Pi ved bruk av TensorFlow Lite.

Author: Bachelor Project 2026 - Autonomia
"""

import numpy as np
import cv2
from pathlib import Path
from typing import Optional, Tuple
from enum import Enum

try:
    import tensorflow as tf
    from tensorflow import keras
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False
    print("ADVARSEL: TensorFlow ikke tilgjengelig. ML-klassifisering deaktivert.")


class BallColorML(Enum):
    """Enum for ML-klassifiserte ballfarger"""
    RED = 0
    BLUE = 1
    GREEN = 2
    UNKNOWN = 3


class MLBallClassifier:
    """
    ML-basert ballklassifiserer som bruker CNN for fargedeteksjon.
    
    Denne klassen erstatter HSV-basert fargedeteksjon med en trent
    neural network modell som er mer robust overfor lysvariasjoner.
    
    Teknisk tilnærming:
    - Transfer Learning med MobileNetV2 (pre-trained på ImageNet)
    - Fine-tuning på ball-dataset
    - TensorFlow Lite for effektiv inferens på Raspberry Pi
    - Input: 224x224 RGB-bilde av ball
    - Output: Sannsynlighet for hver farge-klasse
    """
    
    def __init__(self, 
                 model_path: Optional[str] = None,
                 confidence_threshold: float = 0.7,
                 use_tflite: bool = True):
        """
        Initialiserer ML-klassifisereren.
        
        Args:
            model_path: Sti til trent modell (.tflite eller .h5)
            confidence_threshold: Minimum konfidensverdi for klassifisering
            use_tflite: Bruk TensorFlow Lite for raskere inferens på RPi
        """
        if not TF_AVAILABLE:
            raise ImportError("TensorFlow er ikke installert. Kjør: pip install tensorflow")
        
        self.confidence_threshold = confidence_threshold
        self.use_tflite = use_tflite
        self.model = None
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        
        # Input-størrelse for modellen (MobileNetV2 standard)
        self.input_size = (224, 224)
        
        # Klasse-mapping
        self.class_names = ['red', 'blue', 'green']
        
        # Last modell hvis oppgitt
        if model_path:
            self.load_model(model_path)
        else:
            # Se etter standard modell-fil i models-mappen
            default_model = Path(__file__).parent / "models" / "ball_classifier.tflite"
            if default_model.exists():
                self.load_model(str(default_model))
            else:
                print("INFO: Ingen modell lastet. Tren en modell med train_model.py først.")
    
    def load_model(self, model_path: str):
        """
        Laster en trent modell fra fil.
        
        Args:
            model_path: Sti til modell-fil (.tflite eller .h5)
        """
        model_path = Path(model_path)
        
        if not model_path.exists():
            raise FileNotFoundError(f"Modell-fil ikke funnet: {model_path}")
        
        if model_path.suffix == '.tflite' and self.use_tflite:
            # Last TensorFlow Lite modell (optimalisert for RPi)
            self.interpreter = tf.lite.Interpreter(model_path=str(model_path))
            self.interpreter.allocate_tensors()
            
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            print(f"✓ TFLite modell lastet: {model_path.name}")
            print(f"  Input shape: {self.input_details[0]['shape']}")
            print(f"  Output shape: {self.output_details[0]['shape']}")
        
        elif model_path.suffix in ['.h5', '.keras']:
            # Last full Keras modell med custom objects for preprocessing layers
            custom_objects = {
                'TrueDivide': keras.layers.Lambda(lambda x: x / 127.5),
                'Subtract': keras.layers.Lambda(lambda x: x - 1.0)
            }
            try:
                self.model = keras.models.load_model(str(model_path), custom_objects=custom_objects)
            except Exception:
                # Fallback: prøv uten custom objects
                self.model = keras.models.load_model(str(model_path))
            self.use_tflite = False
            print(f"✓ Keras modell lastet: {model_path.name}")
        
        else:
            raise ValueError(f"Ukjent modell-format: {model_path.suffix}")
    
    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """
        Forbereder bildet for modell-inferens.
        
        Args:
            image: Input-bilde i BGR-format (OpenCV standard)
            
        Returns:
            Preprocessert bilde klar for modellen
        """
        # Konverter BGR til RGB (modellen er trent på RGB)
        if len(image.shape) == 3 and image.shape[2] == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = image
        
        # Resize til modellens input-størrelse
        resized = cv2.resize(image_rgb, self.input_size, interpolation=cv2.INTER_AREA)
        
        # Konverter til float32 (Rescaling layer gjør resten)
        batched = np.expand_dims(resized, axis=0).astype(np.float32)
        
        return batched
    
    def predict(self, image: np.ndarray) -> Tuple[BallColorML, float]:
        """
        Klassifiserer en ball i bildet.
        
        Args:
            image: Bilde av ball (crop rundt ballen)
            
        Returns:
            Tuple med (BallColorML, confidence_score)
        """
        if self.model is None and self.interpreter is None:
            raise RuntimeError("Ingen modell lastet. Kall load_model() først.")
        
        # Preprosesser bildet
        preprocessed = self.preprocess_image(image)
        
        # Kjør inferens
        if self.use_tflite and self.interpreter is not None:
            # TensorFlow Lite inferens
            self.interpreter.set_tensor(self.input_details[0]['index'], preprocessed)
            self.interpreter.invoke()
            predictions = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        else:
            # Full Keras modell inferens
            predictions = self.model.predict(preprocessed, verbose=0)[0]
        
        # Finn klassen med høyest sannsynlighet
        predicted_class = np.argmax(predictions)
        confidence = float(predictions[predicted_class])
        
        # Sjekk om konfidensen er høy nok
        if confidence < self.confidence_threshold:
            return BallColorML.UNKNOWN, confidence
        
        # Map til BallColorML enum
        if predicted_class == 0:
            return BallColorML.RED, confidence
        elif predicted_class == 1:
            return BallColorML.BLUE, confidence
        elif predicted_class == 2:
            return BallColorML.GREEN, confidence
        else:
            return BallColorML.UNKNOWN, confidence
    
    def predict_batch(self, images: list) -> list:
        """
        Klassifiserer flere baller på en gang (batch-processing).
        
        Args:
            images: Liste med bilder av baller
            
        Returns:
            Liste med (BallColorML, confidence) for hver ball
        """
        if not images:
            return []
        
        results = []
        for image in images:
            result = self.predict(image)
            results.append(result)
        
        return results
    
    def get_class_probabilities(self, image: np.ndarray) -> dict:
        """
        Returnerer sannsynligheter for alle klasser.
        
        Nyttig for debugging og visualisering.
        
        Args:
            image: Bilde av ball
            
        Returns:
            Dictionary med {klasse_navn: sannsynlighet}
        """
        if self.model is None and self.interpreter is None:
            raise RuntimeError("Ingen modell lastet.")
        
        preprocessed = self.preprocess_image(image)
        
        if self.use_tflite and self.interpreter is not None:
            self.interpreter.set_tensor(self.input_details[0]['index'], preprocessed)
            self.interpreter.invoke()
            predictions = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        else:
            predictions = self.model.predict(preprocessed, verbose=0)[0]
        
        return {
            name: float(prob) 
            for name, prob in zip(self.class_names, predictions)
        }


def create_base_model(input_shape=(224, 224, 3), num_classes=3):
    """
    Lager en base-modell for trening basert på MobileNetV2.
    
    Denne funksjonen brukes av treningsskriptet.
    
    Args:
        input_shape: Input-dimensjoner (høyde, bredde, kanaler)
        num_classes: Antall klasser å klassifisere (3: rød, blå, grønn)
        
    Returns:
        Ukompilert Keras modell
    """
    if not TF_AVAILABLE:
        raise ImportError("TensorFlow er ikke installert.")
    
    # Last pre-trained MobileNetV2 (uten topp-lag)
    base_model = keras.applications.MobileNetV2(
        input_shape=input_shape,
        include_top=False,
        weights='imagenet'
    )
    
    # Frys hele base model først
    base_model.trainable = False
    
    # Bygg komplett modell
    inputs = keras.Input(shape=input_shape)
    
    # Enkel normalisering (0-255 -> 0-1) uten problematiske layers
    x = keras.layers.Rescaling(1./255)(inputs)
    
    # Base model
    x = base_model(x, training=False)
    
    # Global average pooling
    x = keras.layers.GlobalAveragePooling2D()(x)
    
    # Bedre klassifiseringshodet med mer kapasitet
    x = keras.layers.Dense(256, activation='relu')(x)
    x = keras.layers.Dropout(0.5)(x)
    x = keras.layers.Dense(128, activation='relu')(x)
    x = keras.layers.Dropout(0.3)(x)
    
    # Output-lag
    outputs = keras.layers.Dense(num_classes, activation='softmax')(x)
    
    model = keras.Model(inputs, outputs)
    
    return model


def convert_to_tflite(model_path: str, output_path: Optional[str] = None):
    """
    Konverterer en Keras modell til TensorFlow Lite for Raspberry Pi.
    
    Args:
        model_path: Sti til Keras modell (.h5)
        output_path: Sti for output TFLite modell (default: samme sted med .tflite)
    """
    if not TF_AVAILABLE:
        raise ImportError("TensorFlow er ikke installert.")
    
    # Last modell med custom objects for preprocessing layers
    custom_objects = {
        'TrueDivide': tf.keras.layers.Lambda(lambda x: x / 127.5),
        'Subtract': tf.keras.layers.Lambda(lambda x: x - 1.0)
    }
    model = keras.models.load_model(model_path, custom_objects=custom_objects)
    
    # Konverter til TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    
    # Optimaliseringer for Raspberry Pi
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    
    tflite_model = converter.convert()
    
    # Bestem output-sti
    if output_path is None:
        output_path = Path(model_path).with_suffix('.tflite')
    
    # Lagre
    with open(output_path, 'wb') as f:
        f.write(tflite_model)
    
    print(f"✓ TFLite modell lagret: {output_path}")
    
    # Vis størrelse
    original_size = Path(model_path).stat().st_size / 1024 / 1024
    tflite_size = Path(output_path).stat().st_size / 1024 / 1024
    print(f"  Original: {original_size:.2f} MB")
    print(f"  TFLite: {tflite_size:.2f} MB")
    print(f"  Reduksjon: {(1 - tflite_size/original_size)*100:.1f}%")


if __name__ == "__main__":
    # Test-kode
    print("ML Ball Classifier - Test")
    print("=" * 50)
    
    # Sjekk TensorFlow installasjon
    if TF_AVAILABLE:
        print(f"✓ TensorFlow versjon: {tf.__version__}")
    else:
        print("✗ TensorFlow ikke installert")
        print("  Installer med: pip install tensorflow")
