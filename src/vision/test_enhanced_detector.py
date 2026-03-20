"""
Test Simple Ball Detector with Adaptive Lighting
=================================================

Tester den forenklede detektoren med:
- Multi-range HSV (6 red, 3 blue ranges)
- Hough Circle Transform
- Ensemble voting
- Adaptive lighting (300-700 lux)
- Statistics

DESIGNET FOR ENKEL OG PÅLITELIG DETEKSJON AV STATISKE BALLER.
Håndterer lysforhold fra 300-700 lux automatisk.

Bruk: python src/vision/test_enhanced_detector.py [camera_index]
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.enhanced_detector import EnhancedBallDetector, BallColor


def main():
    """Kjør live test av enhanced detector."""
    
    # Camera index
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    
    print("=" * 60)
    print("SIMPLE BALL DETECTOR - LIVE TEST (with Adaptive Lighting)")
    print("=" * 60)
    print()
    print("Initializing simple detector...")
    
    # Initialiser detector - FORENKLET og PÅLITELIG med adaptiv lyshåndtering
    detector = EnhancedBallDetector(
        min_radius=10,
        max_radius=150,
        confidence_threshold=0.35,  # Balansert terskel
        enable_adaptive_lighting=True  # Håndterer 300-700 lux
    )
    
    print("✓ Detector initialized")
    print()
    print("Features enabled:")
    print("  ✓ Multi-range HSV (6 red ranges, 3 blue ranges)")
    print("  ✓ Hough Circle Transform (geometric validation)")
    print("  ✓ Ensemble voting (combines both methods)")
    print("  ✓ Adaptive lighting (300-700 lux range)")
    print()
    
    # Åpne kamera
    print(f"Opening camera {camera_index}...")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ ERROR: Could not open camera {camera_index}")
        return
    
    # Sett oppløsning
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"✓ Camera opened: {actual_width}x{actual_height} @ {fps:.1f} FPS")
    print()
    print("=" * 60)
    print("CONTROLS:")
    print("  'q' - Quit")
    print("  's' - Show statistics")
    print("  'r' - Reset statistics")
    print("=" * 60)
    print()
    
    # Statistics
    frame_count = 0
    red_count = 0
    blue_count = 0
    start_time = time.time()
    
    # FPS calculation
    fps_time = time.time()
    fps_frame_count = 0
    current_fps = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read frame")
            break
        
        frame_count += 1
        fps_frame_count += 1
        
        # FPS-beregning
        if time.time() - fps_time >= 1.0:
            current_fps = fps_frame_count
            fps_frame_count = 0
            fps_time = time.time()
        
        # Detect balls
        detected_balls, _ = detector.detect_balls(frame)
        
        # Count colors
        for ball in detected_balls:
            if ball.color == BallColor.RED:
                red_count += 1
            elif ball.color == BallColor.BLUE:
                blue_count += 1
        
        # Draw detections + overlay panel
        red_pct  = (red_count  / frame_count * 100) if frame_count > 0 else 0
        blue_pct = (blue_count / frame_count * 100) if frame_count > 0 else 0
        overlay = {
            f"FPS":         current_fps,
            f"Frame":       frame_count,
            f"Detections":  len(detected_balls),
            f"RED":         f"{red_count} ({red_pct:.1f}%)",
            f"BLUE":        f"{blue_count} ({blue_pct:.1f}%)",
        }
        output = detector.draw_detections(frame, detected_balls, show_info=True, overlay=overlay)
        
        # Show frame
        cv2.imshow('Enhanced Ball Detector - TEST', output)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('s'):
            # Show statistics
            stats = detector.get_statistics()
            elapsed = time.time() - start_time
            
            print("\n" + "=" * 60)
            print("STATISTICS")
            print("=" * 60)
            print(f"Time: {elapsed:.1f}s")
            print(f"Frames: {frame_count}")
            print(f"Average FPS: {frame_count / elapsed:.1f}")
            print()
            print("Detection Methods:")
            print(f"  HSV detections: {stats['hsv_detections']}")
            print(f"  Hough detections: {stats['hough_detections']}")
            print(f"  Ensemble detections: {stats['ensemble_detections']}")
            print()
            print("Ball Counts:")
            print(f"  Red: {red_count} ({red_pct:.1f}%)")
            print(f"  Blue: {blue_count} ({blue_pct:.1f}%)")
            print(f"  Total: {red_count + blue_count}")
            print(f"  Average per frame: {(red_count + blue_count) / frame_count:.2f}")
            print("=" * 60)
            print()
        elif key == ord('r'):
            # Reset statistics
            frame_count = 0
            red_count = 0
            blue_count = 0
            start_time = time.time()
            detector.stats = {
                'hsv_detections': 0,
                'hough_detections': 0,
                'ensemble_detections': 0
            }
            print("\n✓ Statistics reset\n")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    # Final report
    elapsed = time.time() - start_time
    stats = detector.get_statistics()
    
    print("\n" + "=" * 60)
    print("FINAL REPORT")
    print("=" * 60)
    print(f"Test duration: {elapsed:.1f}s")
    print(f"Total frames: {frame_count}")
    print(f"Average FPS: {frame_count / elapsed:.1f}")
    print()
    print("Detection Methods:")
    print(f"  HSV detections: {stats['hsv_detections']}")
    print(f"  Hough detections: {stats['hough_detections']}")
    print(f"  Ensemble detections: {stats['ensemble_detections']}")
    print()
    print("Ball Counts:")
    print(f"  Red: {red_count} ({red_pct:.1f}% detection rate)")
    print(f"  Blue: {blue_count} ({blue_pct:.1f}% detection rate)")
    print(f"  Total: {red_count + blue_count}")
    print(f"  Average per frame: {(red_count + blue_count) / frame_count:.2f}")
    print()
    
    # Performance assessment
    print("PERFORMANCE ASSESSMENT:")
    if red_pct >= 90 and blue_pct >= 90:
        print("  ✓ EXCELLENT - Detection rates meet 90%+ target!")
    elif red_pct >= 70 and blue_pct >= 70:
        print("  ⚠ GOOD - Detection rates above 70% but below target")
    elif red_pct >= 50 and blue_pct >= 50:
        print("  ⚠ MODERATE - Detection rates above 50% but need improvement")
    else:
        print("  ❌ POOR - Detection rates below 50%, further tuning needed")
    
    print("=" * 60)


if __name__ == "__main__":
    main()
