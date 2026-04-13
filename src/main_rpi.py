"""
main_rpi.py — Hovedinngang for robotstyringsprogrammet på Raspberry Pi.

Starter kamera-debug-vindu og kjører ball-deteksjon.
Arduino-kommunikasjon og kinematikk merges inn senere fra en annen branch.
"""

import sys
import config
from vision.oak_camera import OAKCamera
from vision.stream_debug import DiagnosticDetector, camera_loop


def main():
    print("=== Autonomia Robot Control System ===")
    print("Åpner OAK-kamera ...")

    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)
    if not cam.open():
        print("FEIL: Kunne ikke åpne kameraet.")
        sys.exit(1)

    focal_px = cam.get_focal_length_px(hfov_deg=config.CAMERA_HFOV_DEG)
    print(f"Kamera OK   oppløsning={config.CAMERA_RESOLUTION}   f={focal_px:.1f}px")

    detector = DiagnosticDetector(
        min_radius=config.BALL_MIN_RADIUS,
        max_radius=config.BALL_MAX_RADIUS,
        confidence_threshold=config.BALL_CONFIDENCE_THRESHOLD,
        focal_length_px=focal_px,
        max_balls_per_color=4,
    )

    # Kjør kamera-loop — trykk 'q' i vinduet for å avslutte
    camera_loop(detector, cam)


if __name__ == "__main__":
    main()
