"""
mock_serial.py
==============
Drop-in replacement for ``serial.Serial`` that prints commands to the
terminal instead of sending them over USB.  Used for testing the full
sorting pipeline without the OpenRB-150 or Dynamixel motors connected.

When a ``visualizer`` is attached, motor movements are *interpolated*
over several frames so the 3-D plot shows smooth animation.

Usage
-----
Replace::

    import serial
    ser = serial.Serial('/dev/ttyACM0', 115200)

with::

    from mock_serial import MockSerial
    ser = MockSerial()

The rest of the code (``ser.write()``, ``ser.readline()``) works
identically.
"""

import json
import time


class MockSerial:
    """Fake serial port that simulates the OpenRB-150 response protocol."""

    def __init__(
        self,
        port: str = "/dev/MOCK",
        baudrate: int = 115200,
        timeout: float = 1.0,
        move_delay: float = 1.5,
        visualizer=None,
        anim_frames: int = 30,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.move_delay = move_delay  # total simulated motor travel time
        self.is_open = True
        self._last_command = None

        # ── Visualiser integration ────────────────────────────────────
        self.visualizer = visualizer
        self.anim_frames = anim_frames   # frames per movement

        # Current simulated motor positions (start at centre = 2048)
        self._current_steps = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 2048}

        print(f"[MOCK SERIAL] ✅  Opened fake port {self.port} @ {self.baudrate} baud")

    # ── Attach a visualiser after construction ────────────────────────
    def set_visualizer(self, viz):
        """Attach a live visualiser so motor movements are animated."""
        self.visualizer = viz

    # ── pyserial-compatible API ───────────────────────────────────────

    def write(self, data: bytes) -> int:
        """Receive a JSON command, pretty-print it, and simulate motor delay.

        If a visualiser is attached, the move is *interpolated* over
        ``anim_frames`` steps so the 3-D plot animates smoothly.

        Parameters
        ----------
        data : bytes
            The JSON string (with trailing newline) that would be sent
            to the OpenRB-150.

        Returns
        -------
        int
            Number of bytes "sent".
        """
        text = data.decode("utf-8", errors="replace").strip()
        self._last_command = text

        # Try to pretty-print the JSON; fall back to raw string
        try:
            parsed = json.loads(text)
            pretty = json.dumps(parsed, indent=2)
        except json.JSONDecodeError:
            parsed = None
            pretty = text

        print(f"\n[MOCK SERIAL TX] ──────────────────────────────")
        print(f"  {pretty}")

        # ── Animate or sleep ──────────────────────────────────────────
        if parsed and self.visualizer and all(k in parsed for k in ("m1", "m2", "m3", "m4")):
            target_steps = {k: parsed[k] for k in ("m1", "m2", "m3", "m4")}
            self._animate_move(target_steps)
        else:
            print(f"[MOCK SERIAL] ⏳  Simulating motor movement ({self.move_delay}s)...")
            time.sleep(self.move_delay)

        print(f"[MOCK SERIAL] ✅  Motors reached target position")
        return len(data)

    def _animate_move(self, target_steps: dict):
        """Interpolate from current steps to target over N frames."""
        start = dict(self._current_steps)
        n = self.anim_frames
        dt = self.move_delay / max(n, 1)

        print(f"[MOCK SERIAL] 🎬  Animating {n} frames ({self.move_delay:.1f}s)...")

        for i in range(1, n + 1):
            t = i / n  # 0→1
            # Smooth ease-in-out (sinusoidal)
            t_smooth = (1 - __import__("math").cos(t * __import__("math").pi)) / 2.0

            frame = {}
            for key in ("m1", "m2", "m3", "m4"):
                frame[key] = int(round(start[key] + (target_steps[key] - start[key]) * t_smooth))

            self.visualizer.update_plot(frame)
            time.sleep(dt)

        # Snap to exact target
        self._current_steps = dict(target_steps)
        self.visualizer.update_plot(self._current_steps)

    def readline(self) -> bytes:
        """Return a fake ``OK\\n`` response, mimicking the OpenRB-150."""
        return b"OK\n"

    def read(self, size: int = 1) -> bytes:
        """Read *size* bytes (always returns empty for the mock)."""
        return b""

    def flush(self):
        """No-op flush."""
        pass

    def close(self):
        """Close the fake port."""
        self.is_open = False
        print(f"[MOCK SERIAL] 🔌  Closed fake port {self.port}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        return f"MockSerial(port={self.port!r}, baudrate={self.baudrate})"
