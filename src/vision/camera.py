"""
camera.py
=========

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

from __future__ import annotations

import logging
import math
import time
import numpy as np
import depthai as dai
from types import TracebackType
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class OAKCamera:
    """
    Wrapper for OAK Series 2 kamera via depthai v3.

    Bruker depthai Pipeline som context manager internt og eksponerer
    et enkelt read()/release()-grensesnitt tilsvarende cv2.VideoCapture.
    """

    def __init__(self, resolution: Tuple[int, int] = (1280, 720)) -> None:
        """
        Args:
            resolution: Ønsket oppløsning (bredde, høyde).
                        OAK RGB-kameraet støtter f.eks. (1280, 720) og (640, 400).
        """
        self._resolution = resolution
        self._pipeline: Optional[dai.Pipeline] = None
        self._queue = None
        self._control_queue = None
        self._opened = False

    # ------------------------------------------------------------------
    # Offentlig grensesnitt (tilsvarende cv2.VideoCapture)
    # ------------------------------------------------------------------

    # Antall frames å forkaste etter oppstart for å la AE konvergere.
    # Uten dette er bildet nesten svart (mean brightness ~32/255) pga
    # at auto-eksponering ikke er ferdig ved første frame.
    _AE_WARMUP_FRAMES = 30

    def open(self) -> bool:
        """
        Åpner kameraet og starter pipeline.

        Venter til auto-eksponering (AE) har konvergert før kameraet
        regnes som klart. Dette er nødvendig ved USB 2.0-tilkobling
        (f.eks. via Dell USB-adapter) der kameraet trenger lenger tid
        på å stabilisere eksponering.

        Returns:
            True hvis kameraet ble åpnet, False ved feil.
        """
        try:
            self._pipeline = dai.Pipeline()
            # Intentional dunder call: depthai.Pipeline requires explicit
            # __enter__/__exit__ to manage the device connection lifecycle
            # when not using a `with` block directly on the pipeline object.
            self._pipeline.__enter__()

            cam = self._pipeline.create(dai.node.Camera).build()
            try:
                cam.inputControl.setBlocking(False)
                cam.inputControl.setMaxSize(4)
                self._control_queue = cam.inputControl.createInputQueue()
            except Exception as e:
                # Runtime camera controls are best-effort: normal streaming
                # should still work on DepthAI versions/devices that do not
                # expose an inputControl queue through the v3 Camera node.
                logger.warning("OAKCamera: runtime camera controls unavailable: %s", e)
                self._control_queue = None
            self._queue = cam.requestOutput(self._resolution).createOutputQueue()
            self._pipeline.start()

            # Kast de første N framene slik at AE/AWB rekker å konvergere.
            # Uten dette er bildet nesten svart ved USB 2.0-tilkobling.
            logger.info("Venter på AE (%d frames)...", self._AE_WARMUP_FRAMES)
            for _ in range(self._AE_WARMUP_FRAMES):
                self._queue.get()
            logger.info("AE warmup complete — kamera klart")

            self._opened = True
            return True
        except Exception as e:
            logger.error("OAKCamera: Kunne ikke åpne kamera: %s", e)
            self._cleanup()
            return False

    def discard_frames(self, count: int) -> int:
        """Discard up to ``count`` frames from the RGB stream.

        This is used after startup and after camera-control changes so the
        caller sees frames captured with settled exposure/white-balance state.

        Returns the number of frames actually discarded.
        """
        discarded = 0
        for _ in range(max(0, int(count))):
            ret, _frame = self.read()
            if ret:
                discarded += 1
            else:
                # Avoid a tight loop if the camera drops out temporarily.
                time.sleep(0.01)
        return discarded

    def enable_auto_exposure_white_balance(self, discard_frames: int = 0) -> bool:
        """Enable AE/AWB and unlock any previous exposure/WB locks.

        The calibration workflow calls this before settling on an empty desk,
        then locks the converged values before detection starts.
        """
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureEnable()
        ctrl.setAutoExposureLock(False)
        ctrl.setAutoWhiteBalanceLock(False)
        return self._send_camera_control(ctrl, "enable AE/AWB", discard_frames)

    def lock_auto_exposure_white_balance(self, discard_frames: int = 0) -> bool:
        """Lock the current auto-exposure and auto-white-balance values.

        DepthAI keeps the latest AE sensor configuration when AE lock is set.
        Locking after the desk has been visible for several frames prevents a
        later hand/forearm entering the image from shifting exposure or white
        balance during ball auto-detection.
        """
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureLock(True)
        ctrl.setAutoWhiteBalanceLock(True)
        return self._send_camera_control(ctrl, "lock AE/AWB", discard_frames)

    def set_manual_exposure_white_balance(
        self,
        exposure_us: int,
        iso: int,
        white_balance_k: Optional[int] = None,
        discard_frames: int = 0,
    ) -> bool:
        """Set deterministic manual exposure/ISO and optional white balance.

        This is available for calibration/diagnostics if the locked-auto values
        are not stable enough for a specific lighting setup.  The explicit
        AE/AWB locks make the command robust across DepthAI versions/devices
        where automatic controls may otherwise keep adapting after a manual
        exposure command has been sent.
        """
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(int(exposure_us), int(iso))
        if white_balance_k is not None:
            ctrl.setManualWhiteBalance(int(white_balance_k))
        ctrl.setAutoExposureLock(True)
        ctrl.setAutoWhiteBalanceLock(True)
        return self._send_camera_control(ctrl, "manual exposure/WB", discard_frames)

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
        except Exception as e:
            logger.debug("Frame read failed: %s", e)
            return False, None

    def release(self) -> None:
        """Stopper pipeline og frigir kameraressurser."""
        self._cleanup()

    def get_resolution(self) -> Tuple[int, int]:
        """Returnerer konfigurert oppløsning (bredde, høyde)."""
        return self._resolution

    def get_focal_length_px(self, hfov_deg: float = 81.0) -> float:
        """
        Returnerer kalibrert brennvidde i piksler for konfigurert oppløsning.

        Forsøker først å lese kalibrerte intrinsics fra kameraets EEPROM via
        depthai. Hvis det feiler (kamera ikke tilkoblet, kalibreringsdata mangler)
        beregnes brennvidden teoretisk fra oppgitt HFOV:
            f = (width / 2) / tan(HFOV_rad / 2)

        For OAK Series 2 / IMX378 ved 640×400 gir dette ~375 px.
        Sammenlignet med standard-gjettet 900 px gir dette korrekte
        avstandsestimater for baller ca. 50 cm fra kameraet.

        Args:
            hfov_deg: Horisontal synsfelt som fallback (standard: 81° for IMX378)

        Returns:
            Brennvidde i piksler (fx)
        """
        # Forsøk kalibrert verdi fra enhetens EEPROM
        try:
            calib = self._pipeline.defaultDevice.readCalibration()
            w, h = self._resolution
            M, _, _ = calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A, w, h
            )
            fx = float(M[0][0])
            if fx > 50:  # sanity-sjekk
                return fx
        except Exception as e:
            logger.debug("EEPROM focal length read failed: %s", e)

        # Teoretisk beregning fra HFOV
        w = self._resolution[0]
        return (w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "OAKCamera":
        if not self.open():
            raise RuntimeError("Failed to open OAK-D camera")
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.release()

    # ------------------------------------------------------------------
    # Intern
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        """Release camera resources on context exit."""
        self._control_queue = None
        if self._pipeline is not None:
            try:
                # Intentional dunder call: matches the explicit __enter__()
                # in open() to properly tear down the depthai device session.
                self._pipeline.__exit__(None, None, None)
            except Exception:
                pass
            self._pipeline = None
        self._queue = None
        self._opened = False

    def _send_camera_control(self, ctrl: dai.CameraControl, label: str,
                             discard_frames: int = 0) -> bool:
        """Send a DepthAI CameraControl message through the runtime queue."""
        if self._control_queue is None:
            logger.warning("OAKCamera: cannot %s; no runtime control queue", label)
            return False
        try:
            self._control_queue.send(ctrl)
            if discard_frames > 0:
                self.discard_frames(discard_frames)
            logger.info("OAKCamera: %s applied", label)
            return True
        except Exception as e:
            logger.warning("OAKCamera: failed to %s: %s", label, e)
            return False
