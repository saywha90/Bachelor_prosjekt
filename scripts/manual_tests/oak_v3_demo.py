"""
oak_v3_demo.py
==============

Minimal tilkoblingstest for OAK Series 2 kamera (depthai v3).
Brukes for a verifisere at kameraet er koblet til og sender video.

Bruk: python scripts/manual_tests/oak_v3_demo.py

Trykk 'q' for a avslutte.
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

import cv2
try:
    import depthai as dai
except ImportError:
    print("Error: depthai library not installed. Install with: pip install depthai")
    sys.exit(1)

print("OAK KAMERA - TILKOBLINGSTEST")
print("-" * 40)

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    queue = cam.requestOutput((1280, 720)).createOutputQueue()

    print("Starter pipeline...")
    pipeline.start()
    print("OK Kamera tilkoblet!")
    print("Trykk 'q' for a avslutte")
    print()

    while pipeline.isRunning():
        img = queue.get()
        frame = img.getCvFrame()
        cv2.imshow("OAK Series 2 - Tilkoblingstest", frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
print("OK Test fullfort!")
