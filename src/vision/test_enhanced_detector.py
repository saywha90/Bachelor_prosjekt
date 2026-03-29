"""
Test Balldetektoren mot OAK Series 2 kamera
============================================

Tester SimpleBallDetector (ensemble HSV + Hough) med live video fra
Luxonis OAK Series 2 kamera via depthai v3 API.

Detekterer rode og bla baller med:
- Multi-range HSV (6 red-ranger, 3 blue-ranger)
- Hough Circle Transform (geometrisk validering)
- Ensemble-voting (kombinerer begge metodene)
- Adaptiv lyshandtering (300-700 lux)

Bruk: python src/vision/test_enhanced_detector.py
"""

import cv2
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.oak_camera import OAKCamera
from vision.enhanced_detector import SimpleBallDetector, BallColor
import config


def main():
    """Kjor live balldeteksjon med OAK-kameraet."""

    print("=" * 60)
    print("BALL DETECTOR - OAK Series 2")
    print("=" * 60)
    print()
    print("Initialiserer detektor...")

    detector = SimpleBallDetector(
        min_radius=10,
        max_radius=150,
        confidence_threshold=0.35,
        enable_adaptive_lighting=True,
    )

    print("OK Detektor klar")
    print()
    print("Funksjoner:")
    print("  OK Multi-range HSV (6 red, 3 blue ranger)")
    print("  OK Hough Circle Transform")
    print("  OK Ensemble-voting")
    print("  OK Adaptiv lyshandtering (300-700 lux)")
    print()

    print(f"Apner OAK-kamera ({config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]})...")
    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)

    if not cam.open():
        print("FEIL: Kunne ikke apne OAK-kameraet.")
        print("   Kontroller at kameraet er koblet til via USB.")
        return

    w, h = cam.get_resolution()
    print(f"OK Kamera apnet: {w}x{h}")
    print()
    print("=" * 60)
    print("KONTROLLER:")
    print("  'q' - Avslutt")
    print("  's' - Vis statistikk")
    print("  'r' - Nullstill statistikk")
    print("=" * 60)
    print()

    frame_count = 0
    red_count = 0
    blue_count = 0
    start_time = time.time()

    fps_time = time.time()
    fps_frame_count = 0
    current_fps = 0

    while cam.isOpened():
        ret, frame = cam.read()
        if not ret:
            print("FEIL: Klarte ikke lese frame")
            break

        frame_count += 1
        fps_frame_count += 1

        if time.time() - fps_time >= 1.0:
            current_fps = fps_frame_count
            fps_frame_count = 0
            fps_time = time.time()

        detected_balls, _ = detector.detect_balls(frame)

        for ball in detected_balls:
            if ball.color == BallColor.RED:
                red_count += 1
            elif ball.color == BallColor.BLUE:
                blue_count += 1

        red_pct  = (red_count  / frame_count * 100) if frame_count > 0 else 0
        blue_pct = (blue_count / frame_count * 100) if frame_count > 0 else 0
        overlay = {
            "FPS":        current_fps,
            "Frame":      frame_count,
            "Detections": len(detected_balls),
            "ROD":        f"{red_count} ({red_pct:.1f}%)",
            "BLA":        f"{blue_count} ({blue_pct:.1f}%)",
        }
        output = detector.draw_detections(frame, detected_balls, show_info=True, overlay=overlay)

        cv2.imshow("Ball Detector - OAK Series 2", output)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("\nAvslutter...")
            break
        elif key == ord("s"):
            stats = detector.get_statistics()
            elapsed = time.time() - start_time
            print("\n" + "=" * 60)
            print("STATISTIKK")
            print("=" * 60)
            print(f"Tid: {elapsed:.1f}s  |  Frames: {frame_count}  |  Snitt FPS: {frame_count / elapsed:.1f}")
            print()
            print("Deteksjonsmetoder:")
            print(f"  HSV:      {stats['hsv_detections']}")
            print(f"  Hough:    {stats['hough_detections']}")
            print(f"  Ensemble: {stats['ensemble_detections']}")
            print()
            print("Baller:")
            print(f"  Rod:  {red_count} ({red_pct:.1f}%)")
            print(f"  Bla:  {blue_count} ({blue_pct:.1f}%)")
            print(f"  Totalt: {red_count + blue_count}")
            print("=" * 60)
        elif key == ord("r"):
            frame_count = 0
            red_count = 0
            blue_count = 0
            start_time = time.time()
            detector.stats = {
                "hsv_detections":      0,
                "hough_detections":    0,
                "ensemble_detections": 0,
                "lighting_level":      "unknown",
            }
            print("\nOK Statistikk nullstilt\n")

    cam.release()
    cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    stats = detector.get_statistics()

    print("\n" + "=" * 60)
    print("SLUTTRAPPORT")
    print("=" * 60)
    print(f"Varighet: {elapsed:.1f}s  |  Frames: {frame_count}  |  Snitt FPS: {(frame_count / elapsed):.1f}" if elapsed > 0 else "Varighet: 0s")
    print()
    print("Deteksjonsmetoder:")
    print(f"  HSV:      {stats['hsv_detections']}")
    print(f"  Hough:    {stats['hough_detections']}")
    print(f"  Ensemble: {stats['ensemble_detections']}")
    print()

    if frame_count > 0:
        red_pct  = (red_count  / frame_count) * 100
        blue_pct = (blue_count / frame_count) * 100
        print("Baller:")
        print(f"  Rod:    {red_count} ({red_pct:.1f}%)")
        print(f"  Bla:    {blue_count} ({blue_pct:.1f}%)")
        print(f"  Totalt: {red_count + blue_count}")
        print()
        print("YTELSE:")
        if red_pct >= 90 and blue_pct >= 90:
            print("  UTMERKET - Deteksjonsrate over 90%!")
        elif red_pct >= 70 and blue_pct >= 70:
            print("  BRA - Deteksjonsrate over 70%, under mal")
        else:
            print("  TRENGS FORBEDRING - Deteksjonsrate under 70%")
    else:
        print("  Ingen frames fanget - kamera ikke tilgjengelig.")

    print("=" * 60)


if __name__ == "__main__":
    main()
