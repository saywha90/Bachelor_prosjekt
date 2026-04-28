"""Focused coverage for M3 scan-pose mitigation runtime helpers."""

import main as runtime_main


class StubSerial:
    """Minimal serial stub that captures JSON commands and replays responses."""

    def __init__(self, responses: list[bytes]):
        self.responses = list(responses)
        self.commands = []

    def write(self, data: bytes) -> int:
        self.commands.append(data.decode().strip())
        return len(data)

    def readline(self) -> bytes:
        if self.responses:
            return self.responses.pop(0)
        return b"OK\n"


class StubVision:
    """Tiny stand-in that records the pose passed to verify_pose()."""

    def __init__(self, result: bool):
        self.result = result
        self.received_positions = None

    def verify_pose(self, current_motor_positions: dict) -> bool:
        self.received_positions = current_motor_positions
        return self.result


def test_read_current_uses_existing_firmware_command():
    ser = StubSerial([b'{"m1": 0, "m2": 0, "m3": 287, "m4": 0, "m5": 0}\n'])

    currents = runtime_main.read_current(ser)

    assert currents == {"m1": 0, "m2": 0, "m3": 287, "m4": 0, "m5": 0}
    assert ser.commands == ['{"cmd": "read_current"}']


def test_sync_goal_to_present_pose_before_restore_recommands_actual_pose():
    ser = StubSerial([
        b'{"m1": 2010, "m2": 2020, "m3": 2030, "m4": 2040, "m5": 2050}\n',
        b"OK\n",
    ])

    pose = runtime_main.sync_goal_to_present_pose_before_restore(ser)

    assert pose == {"m1": 2010, "m2": 2020, "m3": 2030, "m4": 2040, "m5": 2050}
    assert ser.commands == [
        '{"cmd": "read_pos"}',
        '{"m1": 2010, "m2": 2020, "m3": 2030, "m4": 2040, "m5": 2050}',
    ]


def test_verify_scan_pose_before_scan_delegates_to_vision_bridge():
    ser = StubSerial([b'{"m1": 1, "m2": 2, "m3": 3, "m4": 4, "m5": 5}\n'])
    vision = StubVision(result=False)

    ok = runtime_main.verify_scan_pose_before_scan(ser, vision)

    assert ok is False
    assert vision.received_positions == {"m1": 1, "m2": 2, "m3": 3, "m4": 4, "m5": 5}
    assert ser.commands == ['{"cmd": "read_pos"}']


def test_m3_torque_relax_is_hard_gated_by_default():
    ser = StubSerial([])

    relaxed = runtime_main.m3_torque_relax(ser)

    assert relaxed is False
    assert ser.commands == []
