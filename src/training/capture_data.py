"""
capture_data.py
========================

Tar bilder av en ball til treningsdatasett med OAK Series 2 kamera.

Trykk SPACE for a lagre bildet - ALLTID, uavhengig av deteksjon.
Slik far du gode treningsbilder selv nar detektoren ikke ser ballen.

Bruk:
    python src/training/capture_data.py --color red
    python src/training/capture_data.py --color blue
    python src/training/capture_data.py          # standard: blue
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import argparse
import cv2
from pathlib import Path
from datetime import datetime
import time

from vision.camera import OAKCamera
from config import vision as config

IMAGES_TARGET = 100


def count_existing(save_dir: Path) -> int:
    return len(list(save_dir.glob("*.jpg")))


def main():
    parser = argparse.ArgumentParser(description="Ta treningsbilder med OAK kamera")
    parser.add_argument("--color", choices=["red", "blue"], default="blue",
                        help="Hvilken ball tas bilde av: red eller blue (default: blue)")
    parser.add_argument("--target", type=int, default=IMAGES_TARGET,
                        help=f"Antall bilder som skal tas (default: {IMAGES_TARGET})")
    args = parser.parse_args()

    color = args.color
    save_dir = Path(f"training_data/{color}")
    save_dir.mkdir(parents=True, exist_ok=True)

    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)
    if not cam.open():
        print("FEIL: Kunne ikke apne OAK-kameraet.")
        sys.exit(1)

    count = count_existing(save_dir)
    color_no = "ROD" if color == "red" else "BLA"
    print("=" * 60)
    print(f"BILDEINNSAMLING - {color_no} BALL")
    print("=" * 60)
    print(f"Mal:          {args.target} bilder")
    print(f"Eksisterende: {count}")
    print(f"Trenger:      {max(0, args.target - count)} til")
    print(f"Lagres i:     {save_dir.resolve()}")
    print()
    print("  SPACE  = ta bilde  (alltid, uansett deteksjon)")
    print("  q      = avslutt")
    print("=" * 60)

    flash_until = 0.0
    status = f"Klar - {max(0, args.target - count)} bilder igjen"

    while cam.isOpened():
        ret, frame = cam.read()
        if not ret:
            break

        now = time.time()
        display = frame.copy()
        h, w = display.shape[:2]

        # Bakgrunnspanel topp
        cv2.rectangle(display, (0, 0), (w, 50), (30, 30, 30), -1)

        # Progress
        done = count >= args.target
        bar_color = (80, 200, 80) if done else (80, 120, 220) if color == "blue" else (60, 60, 200)
        progress_text = f"{color_no}: {count}/{args.target}  {'FERDIG!' if done else ''}"
        cv2.putText(display, progress_text, (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, bar_color, 2)

        # Statusmelding bunn
        cv2.rectangle(display, (0, h - 45), (w, h), (30, 30, 30), -1)
        cv2.putText(display, status, (15, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Flash-ramme ved lagring
        if now < flash_until:
            cv2.rectangle(display, (4, 4), (w - 4, h - 4), (0, 80, 255), 6)

        if done:
            cv2.putText(display, "FERDIG! Trykk q", (w // 2 - 150, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (80, 200, 80), 3)

        cv2.imshow(f"Bildeinnsamling - {color_no} ball - SPACE = ta bilde", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord(" ") and not done:
            count += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:21]
            filename = save_dir / f"{color}_{count:03d}_{ts}.jpg"
            cv2.imwrite(str(filename), frame)
            flash_until = now + 0.25
            status = f"LAGRET {filename.name}  ({count}/{args.target})"
            print(f"  [{count:3d}/{args.target}] {filename.name}")

    cam.release()
    cv2.destroyAllWindows()
    print()
    print("=" * 60)
    print(f"Totalt {color_no} bilder: {count_existing(save_dir)}")
    print(f"Lagret i:    {save_dir.resolve()}")
    print("=" * 60)


if __name__ == "__main__":
    main()
