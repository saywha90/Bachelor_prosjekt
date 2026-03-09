"""
Data Collection Tool for Ball Classifier Training
==================================================

Dette verktøyet hjelper deg med å samle treningsdata for ML-modellen.
Det tar bilder fra kameraet og lagrer dem i riktig mappestruktur.

Bruk:
    python collect_training_data.py --output_dir training_data --class red

Kontroller:
    SPACE   - Ta bilde og lagre
    Q       - Avslutt
    C       - Bytt klasse (rød/blå/grønn)

Author: Bachelor Project 2026 - Autonomia
"""

import cv2
import argparse
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional


class DataCollector:
    """
    Verktøy for å samle treningsdata fra kamera.
    
    Organiserer bilder i mapper per klasse og sørger for
    at filnavnene er unike og beskrivende.
    """
    
    def __init__(self, 
                 output_dir: str = "training_data",
                 camera_index: int = 0,
                 img_size: tuple = (224, 224)):
        """
        Initialiserer data collection tool.
        
        Args:
            output_dir: Mappe hvor bilder skal lagres
            camera_index: Kamera-indeks (0 = default kamera)
            img_size: Størrelse å lagre bilder i (224x224 for MobileNetV2)
        """
        self.output_dir = Path(output_dir)
        self.camera_index = camera_index
        self.img_size = img_size
        
        # Opprett output-mappe hvis den ikke finnes
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Klasser
        self.classes = ['red', 'blue', 'green']
        self.current_class_index = 0
        
        # Opprett mapper for hver klasse
        for class_name in self.classes:
            class_dir = self.output_dir / class_name
            class_dir.mkdir(exist_ok=True)
        
        # Statistikk
        self.images_captured = {cls: 0 for cls in self.classes}
        
        # Kamera
        self.cap = None
    
    @property
    def current_class(self) -> str:
        """Returnerer nåværende aktiv klasse"""
        return self.classes[self.current_class_index]
    
    def next_class(self):
        """Bytter til neste klasse"""
        self.current_class_index = (self.current_class_index + 1) % len(self.classes)
        print(f"\n→ Byttet til klasse: {self.current_class.upper()}")
    
    def get_class_dir(self, class_name: str) -> Path:
        """Returnerer mappe for en gitt klasse"""
        return self.output_dir / class_name
    
    def get_next_filename(self, class_name: str) -> Path:
        """
        Genererer neste unike filnavn for en klasse.
        
        Format: ball_{class}_{timestamp}_{counter}.jpg
        """
        class_dir = self.get_class_dir(class_name)
        
        # Tell eksisterende bilder
        existing = len(list(class_dir.glob('*.jpg')))
        
        # Generer filnavn
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"ball_{class_name}_{timestamp}_{existing+1:04d}.jpg"
        
        return class_dir / filename
    
    def save_image(self, image, class_name: Optional[str] = None):
        """
        Lagrer et bilde til riktig klasse-mappe.
        
        Args:
            image: Bilde å lagre
            class_name: Klasse-navn (None = bruk current_class)
        """
        if class_name is None:
            class_name = self.current_class
        
        # Resize til modell-størrelse
        resized = cv2.resize(image, self.img_size, interpolation=cv2.INTER_AREA)
        
        # Generer filnavn
        filepath = self.get_next_filename(class_name)
        
        # Lagre
        cv2.imwrite(str(filepath), resized)
        
        # Oppdater statistikk
        self.images_captured[class_name] += 1
        
        print(f"✓ Lagret: {filepath.name} (Total {class_name}: {self.images_captured[class_name]})")
    
    def draw_ui(self, frame):
        """
        Tegner UI-elementer på frame for å vise status.
        
        Args:
            frame: Frame å tegne på
            
        Returns:
            Frame med UI
        """
        overlay = frame.copy()
        h, w = frame.shape[:2]
        
        # Bakgrunn for header
        cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
        
        # Tittel
        cv2.putText(overlay, "DATA COLLECTION TOOL", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Nåværende klasse (farget)
        class_colors = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0)
        }
        color = class_colors.get(self.current_class, (255, 255, 255))
        
        cv2.putText(overlay, f"Klasse: {self.current_class.upper()}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Statistikk (nederst)
        stats_y = h - 60
        cv2.rectangle(overlay, (0, stats_y), (w, h), (0, 0, 0), -1)
        
        stats_text = "  |  ".join([
            f"{cls.upper()}: {count}"
            for cls, count in self.images_captured.items()
        ])
        cv2.putText(overlay, f"Lagret: {stats_text}", (10, stats_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Kontroller
        controls = "SPACE: Ta bilde  |  C: Bytt klasse  |  Q: Avslutt"
        cv2.putText(overlay, controls, (10, stats_y + 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Blend overlay med original
        alpha = 0.8
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
        
        return frame
    
    def run(self):
        """
        Kjører data collection loop.
        """
        print("="*70)
        print("BALL CLASSIFIER - DATA COLLECTION")
        print("="*70)
        print(f"\nOutput mappe: {self.output_dir.absolute()}")
        print(f"Bilde-størrelse: {self.img_size}")
        print(f"Klasser: {', '.join(self.classes)}")
        print(f"\nKontroller:")
        print("  SPACE - Ta bilde og lagre")
        print("  C     - Bytt klasse")
        print("  Q     - Avslutt")
        print("\n" + "="*70)
        print(f"Starter med klasse: {self.current_class.upper()}")
        print("="*70)
        
        # Åpne kamera
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"\n❌ FEIL: Kunne ikke åpne kamera {self.camera_index}")
            return
        
        # Sett oppløsning
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("\n✓ Kamera åpnet. Starter capture...\n")
        
        try:
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("❌ Kunne ikke lese fra kamera")
                    break
                
                # Tegn UI
                display_frame = self.draw_ui(frame)
                
                # Vis frame
                cv2.imshow('Data Collection', display_frame)
                
                # Håndter input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    # Avslutt
                    print("\nAvslutter...")
                    break
                
                elif key == ord(' '):
                    # Ta bilde
                    self.save_image(frame)
                
                elif key == ord('c'):
                    # Bytt klasse
                    self.next_class()
        
        except KeyboardInterrupt:
            print("\n\nAvbrutt av bruker (Ctrl+C)")
        
        finally:
            # Rydd opp
            self.cap.release()
            cv2.destroyAllWindows()
            
            # Vis sluttstatistikk
            print("\n" + "="*70)
            print("DATA COLLECTION FULLFØRT")
            print("="*70)
            print("\nBilder samlet:")
            total = 0
            for class_name, count in self.images_captured.items():
                print(f"  {class_name.upper()}: {count} bilder")
                total += count
            print(f"\nTotalt: {total} bilder")
            
            # Sjekk om vi har nok data
            print("\n" + "-"*70)
            min_images = 50
            if all(count >= min_images for count in self.images_captured.values()):
                print(f"✅ Du har nok data til å trene modellen!")
                print(f"   Kjør: python train_model.py --data_dir {self.output_dir}")
            else:
                print(f"⚠️  Tips: Samle minst {min_images} bilder per klasse for best resultat")
                for class_name, count in self.images_captured.items():
                    if count < min_images:
                        print(f"   → {class_name.upper()}: trenger {min_images - count} til")
            print("-"*70)


def main():
    """Hovedfunksjon"""
    parser = argparse.ArgumentParser(
        description='Samle treningsdata for ballklassifisering'
    )
    
    parser.add_argument(
        '--output_dir',
        type=str,
        default='training_data',
        help='Mappe hvor bilder skal lagres (default: training_data)'
    )
    
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Kamera-indeks (default: 0)'
    )
    
    parser.add_argument(
        '--size',
        type=int,
        default=224,
        help='Bilde-størrelse (kvadratisk, default: 224)'
    )
    
    args = parser.parse_args()
    
    try:
        collector = DataCollector(
            output_dir=args.output_dir,
            camera_index=args.camera,
            img_size=(args.size, args.size)
        )
        collector.run()
    
    except Exception as e:
        print(f"\n❌ FEIL: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
