"""
oak_camera.py
=============

Wrapper for Luxonis OAK Series 2 kamera via depthai v3 API.

Gir et enkelt grensesnitt som er kompatibelt med cv2.VideoCapture:
    cam = OAKCamera()
    cam.open()
    ret, frame = cam.read()   # frame er numpy BGR-array
    cam.release()

Eller som context manager:
    with OAKCamera() as cam:
        ret, frame = cam.read()

Kameraet: OAK Series 2 med Movidius MyriadX VPU (VID_03E7&PID_2485)
depthai versjon: 3.x (Pipeline-basert API)

Author: Bachelor Project 2026 - Autonomia
"""

import numpy as np
import depthai as dai
from typing import Optional, Tuple


class OAKCamera:
    """
    Wrapper for OAK Series 2 kamera via depthai v3.

    Bruker depthai Pipeline som context manager internt og eksponerer
    et enkelt read()/release()-grensesnitt tilsvarende cv2.VideoCapture.
    """

    def __init__(self, resolution: Tuple[int, int] = (1280, 720)):
        """
        Args:
            resolution: Ønsket oppløsning (bredde, høyde).
                        OAK RGB-kameraet støtter f.eks. (1280, 720) og (640, 400).
        """
        self._resolution = resolution
        self._pipeline: Optional[dai.Pipeline] = None
        self._queue = None
        self._opened = False

    # ------------------------------------------------------------------
    # Offentlig grensesnitt (tilsvarende cv2.VideoCapture)
    # ------------------------------------------------------------------

    def open(self) -> bool:
        """
        Åpner kameraet og starter pipeline.

        Returns:
            True hvis kameraet ble åpnet, False ved feil.
        """
        try:
            self._pipeline = dai.Pipeline()
            self._pipeline.__enter__()

            cam = self._pipeline.create(dai.node.Camera).build()
            self._queue = cam.requestOutput(self._resolution).createOutputQueue()
            self._pipeline.start()

            self._opened = True
            return True
        except Exception as e:
            print(f"❌ OAKCamera: Kunne ikke åpne kamera: {e}")
            self._cleanup()
            return False

    def isOpened(self) -> bool:
        """Returnerer True hvis kameraet er åpent og kjører."""
        return self._opened and self._pipeline is not None and self._pipeline.isRunning()

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Henter neste frame fra kameraet.

        Returns:
            (True, frame_bgr) ved suksess, (False, None) ved feil.
            frame_bgr er et numpy-array i BGR-format (same som cv2).
        """
        if not self.isOpened():
            return False, None
        try:
            img_frame = self._queue.get()
            if img_frame is None:
                return False, None
            return True, img_frame.getCvFrame()
        except Exception:
            return False, None

    def release(self):
        """Stopper pipeline og frigir kameraressurser."""
        self._cleanup()

    def get_resolution(self) -> Tuple[int, int]:
        """Returnerer konfigurert oppløsning (bredde, høyde)."""
        return self._resolution

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "OAKCamera":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    # ------------------------------------------------------------------
    # Intern
    # ------------------------------------------------------------------

    def _cleanup(self):
        if self._pipeline is not None:
            try:
                self._pipeline.__exit__(None, None, None)
            except Exception:
                pass
            self._pipeline = None
        self._queue = None
        self._opened = False
