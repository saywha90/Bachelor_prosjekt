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
    print(f"Apner OAK-kamera ({config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]})...")
    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)

    if not cam.open():
        print("FEIL: Kunne ikke apne OAK-kameraet.")
        print("   Kontroller at kameraet er koblet til via USB.")
        return

    focal_px = cam.get_focal_length_px(hfov_deg=config.CAMERA_HFOV_DEG)
    print(f"OK Kamera apnet: {cam.get_resolution()[0]}x{cam.get_resolution()[1]}")
    print(f"OK Brennvidde: {focal_px:.1f} px  (HFOV={config.CAMERA_HFOV_DEG}°)")

    print("Initialiserer detektor...")
    detector = SimpleBallDetector(
        min_radius=10,
        max_radius=150,
        confidence_threshold=0.35,
        enable_adaptive_lighting=True,
        max_balls_per_color=4,
        focal_length_px=focal_px,
    )
    print("OK Detektor klar")
    print()
    print("Funksjoner:")
    print("  OK Multi-range HSV (6 red, 3 blue ranger)")
    print("  OK Hough Circle Transform")
    print("  OK Ensemble-voting")
    print("  OK Adaptiv lyshandtering (300-700 lux)")
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

        # Per-frame tellung (for sluttrapport)
        red_now  = sum(1 for b in detected_balls if b.color == BallColor.RED)
        blue_now = sum(1 for b in detected_balls if b.color == BallColor.BLUE)
        red_count  += red_now
        blue_count += blue_now

        # Overlay: vis kun hva SOM DETEKTERES AKKURAT NÅ
        overlay = {
            "FPS":  current_fps,
            "Rod":  red_now,
            "Bla":  blue_now,
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
        red_avg  = red_count  / frame_count
        blue_avg = blue_count / frame_count
        print("Gjennomsnittlige deteksjoner per frame:")
        print(f"  Rød:  {red_avg:.2f}")
        print(f"  Blå:  {blue_avg:.2f}")
    else:
        print("  Ingen frames fanget - kamera ikke tilgjengelig.")

    print("=" * 60)


if __name__ == "__main__":
    main()
