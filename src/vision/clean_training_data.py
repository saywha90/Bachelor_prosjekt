"""
Script for å rense treningsdata - cropper bilder til kun ballen.

Dette forbedrer ML-modellens ytelse ved å fjerne distraksjoner fra bakgrunnen.
"""

import cv2
import numpy as np
from pathlib import Path
import shutil
from typing import Optional, Tuple

def find_ball_bbox(image: np.ndarray, color: str) -> Optional[Tuple[int, int, int, int]]:
    """
    Finner bounding box rundt ballen i bildet.
    
    Returns:
        (x, y, w, h) eller None hvis ikke funnet
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # HSV-grenser for farger
    if color == 'red':
        # Rød krever to masker pga wrap-around
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == 'blue':
        mask = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))
    else:
        return None
    
    # Morfologiske operasjoner
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Finn konturer
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Finn største kontur
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Sjekk om stor nok
    area = cv2.contourArea(largest_contour)
    if area < 100:  # Minimum størrelse
        return None
    
    # Få bounding box
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    return (x, y, w, h)


def crop_ball(image: np.ndarray, bbox: Tuple[int, int, int, int], padding: int = 20) -> np.ndarray:
    """
    Cropper bildet rundt ballen med padding.
    
    Args:
        image: Original bilde
        bbox: (x, y, w, h) bounding box
        padding: Ekstra piksler rundt ballen
        
    Returns:
        Cropped bilde
    """
    h, w = image.shape[:2]
    x, y, bw, bh = bbox
    
    # Legg til padding
    x1 = max(0, x - padding)
    y1 = max(0, y - padding)
    x2 = min(w, x + bw + padding)
    y2 = min(h, y + bh + padding)
    
    return image[y1:y2, x1:x2]


def clean_training_data(data_dir: str = "training_data", backup: bool = True):
    """
    Renser alle treningsbilder ved å croppe til kun ballen.
    
    Args:
        data_dir: Mappe med treningsdata
        backup: Lag backup av originale bilder
    """
    data_path = Path(data_dir)
    
    if not data_path.exists():
        print(f"❌ Finner ikke: {data_dir}")
        return
    
    # Backup først
    if backup:
        backup_path = Path(f"{data_dir}_original_backup")
        if not backup_path.exists():
            print(f"📦 Lager backup: {backup_path}")
            shutil.copytree(data_path, backup_path)
        else:
            print(f"⚠️  Backup finnes allerede: {backup_path}")
    
    print(f"\n🔍 Renser treningsbilder i: {data_dir}\n")
    
    stats = {
        'total': 0,
        'cleaned': 0,
        'failed': 0,
        'skipped': 0
    }
    
    # Prosesser hver klasse-mappe
    for class_dir in data_path.iterdir():
        if not class_dir.is_dir():
            continue
        
        class_name = class_dir.name
        print(f"📁 Prosesserer: {class_name}/")
        
        # Finn alle bilder
        images = list(class_dir.glob('*.jpg')) + list(class_dir.glob('*.png'))
        
        for img_path in images:
            stats['total'] += 1
            
            # Last bilde
            image = cv2.imread(str(img_path))
            if image is None:
                print(f"  ❌ Kunne ikke laste: {img_path.name}")
                stats['failed'] += 1
                continue
            
            # Finn ball
            bbox = find_ball_bbox(image, class_name)
            
            if bbox is None:
                print(f"  ⚠️  Fant ikke ball: {img_path.name}")
                stats['skipped'] += 1
                continue
            
            # Crop til ball
            cropped = crop_ball(image, bbox, padding=30)
            
            # Sjekk at cropped image er gyldig
            if cropped.size == 0 or cropped.shape[0] < 50 or cropped.shape[1] < 50:
                print(f"  ⚠️  For lite crop: {img_path.name}")
                stats['skipped'] += 1
                continue
            
            # Lagre tilbake
            cv2.imwrite(str(img_path), cropped)
            stats['cleaned'] += 1
            
            if stats['cleaned'] % 20 == 0:
                print(f"  ✓ Renset {stats['cleaned']} bilder...")
        
        print(f"  ✓ Ferdig med {class_name}: {stats['cleaned']} renset\n")
    
    # Oppsummering
    print("="*60)
    print("📊 OPPSUMMERING")
    print("="*60)
    print(f"Totalt bilder:    {stats['total']}")
    print(f"✅ Renset:        {stats['cleaned']}")
    print(f"⚠️  Hoppet over:   {stats['skipped']}")
    print(f"❌ Feilet:        {stats['failed']}")
    print("="*60)
    
    if stats['cleaned'] > 0:
        print(f"\n✅ Treningsdata er renset!")
        print(f"💡 Tren modellen på nytt for bedre ytelse:")
        print(f"   python src/vision/train_model.py --epochs 30")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Rens treningsbilder")
    parser.add_argument('--data-dir', type=str, default='training_data',
                        help='Mappe med treningsdata')
    parser.add_argument('--no-backup', action='store_true',
                        help='Ikke lag backup (ikke anbefalt)')
    
    args = parser.parse_args()
    
    clean_training_data(
        data_dir=args.data_dir,
        backup=not args.no_backup
    )
