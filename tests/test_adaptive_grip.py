import json
from typing import Optional

import main as main_module
from config.arm import CLAW_CLOSED_POS, CLAW_OPEN_POS, GRIP_EXTRA_CLOSE, GRIP_LOAD_DETECT


class AdaptiveGripSerial:
    def __init__(self, contact_position: Optional[int] = None):
        self.positions = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": CLAW_OPEN_POS}
        self.contact_position = contact_position
        self.motor_goals: list[dict[str, int]] = []
        self._pending_response = "OK\n"

    def write(self, data: bytes) -> int:
        payload = json.loads(data.decode().strip())
        if "cmd" in payload:
            cmd = payload["cmd"]
            if cmd == "read_pos":
                self._pending_response = json.dumps(self.positions) + "\n"
            elif cmd == "read_load":
                load = 0
                if self.contact_position is not None and self.positions["m5"] == self.contact_position:
                    load = main_module.GRIP_LOAD_DETECT
                self._pending_response = json.dumps({"m1": 0, "m2": 0, "m3": 0, "m4": 0, "m5": load}) + "\n"
            elif cmd == "read_current":
                self._pending_response = json.dumps({"m1": 0, "m2": 0, "m3": 0, "m4": 0, "m5": 0}) + "\n"
            else:
                self._pending_response = '{"status":"ok"}\n'
            return len(data)

        goal = {key: int(value) for key, value in payload.items()}
        self.motor_goals.append(goal.copy())
        target_m5 = goal["m5"]
        if self.contact_position is not None and target_m5 >= self.contact_position:
            self.positions = {**goal, "m5": self.contact_position}
        else:
            self.positions = goal
        self._pending_response = "OK\n"
        return len(data)

    def readline(self) -> bytes:
        response = self._pending_response.encode()
        self._pending_response = "OK\n"
        return response


def test_adaptive_grip_steps_from_configured_open_to_closed_limit(monkeypatch):
    monkeypatch.setattr(main_module, "GRIP_POLL_INTERVAL", 0)
    ser = AdaptiveGripSerial(contact_position=None)
    last_solution = {"m1": 2100, "m2": 2200, "m3": 2300, "m4": 2400, "m5": CLAW_CLOSED_POS}

    gripped = main_module.adaptive_grip(ser, last_solution)

    assert gripped is False
    m5_goals = [goal["m5"] for goal in ser.motor_goals]
    assert m5_goals[0] == CLAW_OPEN_POS
    assert m5_goals[-1] == CLAW_CLOSED_POS
    assert all(CLAW_OPEN_POS <= pos <= CLAW_CLOSED_POS for pos in m5_goals)
    assert m5_goals[1:-1] == list(range(CLAW_OPEN_POS + GRIP_EXTRA_CLOSE, CLAW_CLOSED_POS, GRIP_EXTRA_CLOSE))


def test_adaptive_grip_stops_on_load_and_secures_without_passing_closed(monkeypatch):
    monkeypatch.setattr(main_module, "GRIP_POLL_INTERVAL", 0)
    contact_position = CLAW_OPEN_POS + GRIP_EXTRA_CLOSE + 15
    ser = AdaptiveGripSerial(contact_position=contact_position)
    last_solution = {"m1": 2100, "m2": 2200, "m3": 2300, "m4": 2400, "m5": CLAW_OPEN_POS}

    gripped = main_module.adaptive_grip(ser, last_solution)

    assert gripped is True
    m5_goals = [goal["m5"] for goal in ser.motor_goals]
    assert m5_goals[0] == CLAW_OPEN_POS
    assert m5_goals[-1] == contact_position + GRIP_EXTRA_CLOSE
    assert m5_goals[-1] <= CLAW_CLOSED_POS
    assert ser.positions["m5"] == contact_position


def test_adaptive_grip_uses_sensitive_light_load_threshold():
    assert GRIP_LOAD_DETECT == 15
