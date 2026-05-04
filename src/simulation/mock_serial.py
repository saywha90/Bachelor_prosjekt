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


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import logging
import math
import time

logger = logging.getLogger(__name__)


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

        # Current simulated motor positions (start at centre; m4=1911 due to 3D-printed mount offset)
        self._current_steps = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": 2048}

        # Goal positions for simulating load during adaptive grip
        self._goal_steps = dict(self._current_steps)

        # Claw open/closed constants for load simulation
        self._claw_open = 2016
        self._claw_closed = 2675

        # Pending response for special commands (read_pos, set_profile, etc.)
        self._pending_response = None

        logger.debug("[MOCK SERIAL] Opened fake port %s @ %d baud", self.port, self.baudrate)

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

        # ── Handle special commands without animating ──────────────────
        if parsed and isinstance(parsed, dict) and "cmd" in parsed:
            cmd = parsed["cmd"]
            if cmd == "read_pos":
                # Store response for next readline()
                self._pending_response = json.dumps(self._current_steps) + "\n"
                return len(data)
            elif cmd == "set_profile":
                self._pending_response = '{"status":"profile_set"}\n'
                return len(data)
            elif cmd == "set_current_limit":
                motor_id = parsed.get("id", 0)
                value = parsed.get("value", 0)
                self._pending_response = json.dumps({
                    "status": "current_limit_set", "id": motor_id, "value": value
                }) + "\n"
                return len(data)
            elif cmd == "set_torque":
                motor_id = parsed.get("id", 0)
                enable = parsed.get("enable", True)
                self._pending_response = json.dumps({
                    "status": f"torque_{'on' if enable else 'off'}", "id": motor_id
                }) + "\n"
                return len(data)
            elif cmd == "read_current":
                # Return simulated zero current for all motors
                resp = {f"m{mid}": 0 for mid in [1, 2, 3, 4, 5]}
                self._pending_response = json.dumps(resp) + "\n"
                return len(data)
            elif cmd == "enable_torque":
                self._pending_response = "OK:TORQUE_ON\n"
                return len(data)
            elif cmd == "clear_errors":
                self._pending_response = '{"cleared":0}\n'
                return len(data)
            elif cmd == "read_load":
                resp = {f"m{mid}": 0 for mid in [1, 2, 3, 4, 5]}
                # Simulate load feedback when claw is closing toward an object
                m5_current = self._current_steps.get("m5", self._claw_open)
                m5_goal = self._goal_steps.get("m5", m5_current)
                if m5_goal > m5_current and m5_current > self._claw_open:
                    # Simulate increasing load as claw closes
                    travel = self._claw_closed - self._claw_open
                    progress = max(0, m5_current - self._claw_open)
                    resp["m5"] = min(100, progress * 100 // travel) if travel > 0 else 0
                self._pending_response = json.dumps(resp) + "\n"
                return len(data)
            elif cmd == "read_errors":
                resp = {f"m{mid}": 0 for mid in [1, 2, 3, 4, 5]}
                self._pending_response = json.dumps(resp) + "\n"
                return len(data)
            elif cmd == "diagnose":
                # Mock response matching the firmware diagnose command format.
                # Reports all 5 motors as found at the default baud rate with
                # their current simulated positions and realistic model numbers.
                model_numbers = {1: 1060, 2: 1120, 3: 1060, 4: 1060, 5: 1060}  # XM430=1060, XM540=1120
                diag = []
                for mid in [1, 2, 3, 4, 5]:
                    diag.append({
                        "id": mid,
                        "found": True,
                        "baud": 115200,
                        "position": self._current_steps.get(f"m{mid}", 2048),
                        "model": model_numbers.get(mid, 1060),
                    })
                self._pending_response = json.dumps({"diagnostics": diag}) + "\n"
                return len(data)
            else:
                self._pending_response = f"ERR:Unknown cmd: {cmd}\n"
                return len(data)

        logger.debug("[MOCK SERIAL TX] %s", pretty)

        # ── Track goal positions for load simulation ──────────────────
        if parsed and isinstance(parsed, dict):
            for k in ("m1", "m2", "m3", "m4", "m5"):
                if k in parsed:
                    self._goal_steps[k] = parsed[k]

        # ── Animate or sleep ──────────────────────────────────────────
        if parsed and self.visualizer and all(k in parsed for k in ("m1", "m2", "m3", "m4", "m5")):
            target_steps = {k: parsed[k] for k in ("m1", "m2", "m3", "m4", "m5")}
            if target_steps != self._current_steps:
                self._animate_move(target_steps)
            else:
                logger.debug("[MOCK SERIAL] Target pose already reached; skipping no-op animation")
        else:
            logger.debug("[MOCK SERIAL] Simulating motor movement (%.1fs)...", self.move_delay)
            time.sleep(self.move_delay)

        logger.debug("[MOCK SERIAL] Motors reached target position")
        self._pending_response = None  # clear; readline will return "OK\n"
        return len(data)

    def _animate_move(self, target_steps: dict):
        """Interpolate from current steps to target over N frames."""
        start = dict(self._current_steps)
        n = self.anim_frames
        dt = self.move_delay / max(n, 1)

        logger.debug("[MOCK SERIAL] Animating %d frames (%.1fs)...", n, self.move_delay)

        for i in range(1, n + 1):
            t = i / n  # 0→1
            # Smooth ease-in-out (sinusoidal)
            t_smooth = (1 - math.cos(t * math.pi)) / 2.0

            frame = {}
            for key in ("m1", "m2", "m3", "m4", "m5"):
                frame[key] = int(round(start[key] + (target_steps[key] - start[key]) * t_smooth))

            self.visualizer.update_plot(frame)
            time.sleep(dt)

        # Snap to exact target
        self._current_steps = dict(target_steps)
        self.visualizer.update_plot(self._current_steps)

    def readline(self) -> bytes:
        """Return a fake response, mimicking the OpenRB-150.

        If a special command (read_pos, set_profile) was received, return
        the appropriate JSON.  Otherwise return ``OK\\n``.
        """
        if self._pending_response is not None:
            resp = self._pending_response.encode()
            self._pending_response = None
            return resp
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
        logger.debug("[MOCK SERIAL] Closed fake port %s", self.port)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        return f"MockSerial(port={self.port!r}, baudrate={self.baudrate})"
