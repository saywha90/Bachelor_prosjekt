import json
from types import SimpleNamespace
from typing import Optional

import main as main_module
from config.arm import (
    CLAW_CLOSED_POS,
    CLAW_OPEN_POS,
    GRIP_EXTRA_CLOSE,
    GRIP_LOAD_DETECT,
    GRIP_MIN_BALL_BLOCKED_STEPS,
)


class AdaptiveGripSerial:
    def __init__(self, contact_position: Optional[int] = None, *, settling_reads: int = 0):
        self.positions = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": CLAW_OPEN_POS}
        self.contact_position = contact_position
        self.settling_reads = settling_reads
        self.motor_goals: list[dict[str, int]] = []
        self.read_pos_count = 0
        self.secure_target_read_pos_count = 0
        self._pending_response = "OK\n"
        self._blocked_position: Optional[int] = None
        self._secure_target: Optional[int] = None

    def write(self, data: bytes) -> int:
        payload = json.loads(data.decode().strip())
        if "cmd" in payload:
            cmd = payload["cmd"]
            if cmd == "read_pos":
                self.read_pos_count += 1
                if self._secure_target is not None:
                    self.secure_target_read_pos_count += 1
                    if self.secure_target_read_pos_count > self.settling_reads:
                        self.positions["m5"] = self._blocked_position or self.positions["m5"]
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
            self._blocked_position = self.contact_position
            if self._blocked_position is None:
                self.positions = goal
            elif self._secure_target is not None or self.motor_goals[:-1]:
                self.positions = {**goal, "m5": self._blocked_position}
            else:
                self.positions = {**goal, "m5": self._blocked_position}
            self._secure_target = target_m5
        else:
            self._secure_target = None
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


def test_adaptive_grip_waits_for_secure_close_feedback_before_success(monkeypatch):
    monkeypatch.setattr(main_module, "GRIP_POLL_INTERVAL", 0)
    contact_position = CLAW_CLOSED_POS - GRIP_MIN_BALL_BLOCKED_STEPS - 20
    ser = AdaptiveGripSerial(contact_position=contact_position, settling_reads=3)
    last_solution = {"m1": 2100, "m2": 2200, "m3": 2300, "m4": 2400, "m5": CLAW_OPEN_POS}

    gripped = main_module.adaptive_grip(ser, last_solution)

    assert gripped is True
    assert ser.secure_target_read_pos_count >= 3
    assert ser.motor_goals[-1]["m5"] == min(contact_position + GRIP_EXTRA_CLOSE, CLAW_CLOSED_POS)


def test_adaptive_grip_uses_sensitive_light_load_threshold():
    assert GRIP_LOAD_DETECT == 15


def _sorting_plan():
    def solution(name, m5=CLAW_CLOSED_POS):
        return {"commands": {"m1": 2000, "m2": 2100, "m3": 2200, "m4": 2300, "m5": m5}, "name": name}

    return [
        ("pickup", solution("pickup", CLAW_OPEN_POS)),
        ("pickup_recovery_clearance", solution("pickup_recovery_clearance", CLAW_OPEN_POS)),
        ("front_neutral", solution("front_neutral")),
        ("rear_transfer", solution("rear_transfer")),
        ("RED_BIN.drop", solution("RED_BIN.drop")),
        ("return_rear_return_lift", solution("return_rear_return_lift", CLAW_OPEN_POS)),
        ("return_front_neutral", solution("return_front_neutral", CLAW_OPEN_POS)),
    ]


def _patch_sorting_cycle_dependencies(monkeypatch, calls: list[tuple[str, str]]):
    monkeypatch.setattr(main_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(main_module, "load_height_calibration", lambda: None)
    monkeypatch.setattr(main_module, "compute_grab_height", lambda _x, _y: 3.0)
    monkeypatch.setattr(main_module, "compute_wrist_correction", lambda _x, _y: 0)
    monkeypatch.setattr(main_module, "prevalidate_transport_plan", lambda _arm, _colour, _pickup_pose: _sorting_plan())
    monkeypatch.setattr(main_module, "send_claw", lambda _ser, _last, _pos, label="": calls.append(("claw", label)))
    monkeypatch.setattr(main_module, "send_raw_command", lambda _ser, _cmd: '{"status":"ok"}')

    def fake_send_strict_solution(_ser, solution, label="", viz=None, settle_time=None):
        calls.append(("move", solution["name"]))
        return solution["commands"].copy()

    monkeypatch.setattr(main_module, "send_strict_solution", fake_send_strict_solution)


def test_sorting_cycle_aborts_before_sort_route_when_grip_fails(monkeypatch):
    calls: list[tuple[str, str]] = []
    _patch_sorting_cycle_dependencies(monkeypatch, calls)
    monkeypatch.setattr(main_module, "adaptive_grip", lambda _ser, _last_ik: False)
    monkeypatch.setattr(main_module, "verify_grip", lambda _ser, _closed_pos: False)

    success = main_module.run_sorting_cycle(
        ser=SimpleNamespace(),
        arm=SimpleNamespace(),
        detection={"colour": "red", "x": 20.0, "y": 0.0, "z": 0.0},
        vision=SimpleNamespace(),
    )

    assert success is False
    assert ("move", "pickup") in calls
    assert ("move", "pickup_recovery_clearance") in calls
    assert not any(name in {"front_neutral", "rear_transfer", "RED_BIN.drop"} for kind, name in calls if kind == "move")


def test_sorting_cycle_starts_sort_route_only_after_grip_verification(monkeypatch):
    calls: list[tuple[str, str]] = []
    _patch_sorting_cycle_dependencies(monkeypatch, calls)

    def fake_adaptive_grip(_ser, _last_ik):
        calls.append(("verify", "adaptive_grip_complete"))
        return True

    monkeypatch.setattr(main_module, "adaptive_grip", fake_adaptive_grip)

    success = main_module.run_sorting_cycle(
        ser=SimpleNamespace(),
        arm=SimpleNamespace(),
        detection={"colour": "red", "x": 20.0, "y": 0.0, "z": 0.0},
        vision=SimpleNamespace(),
    )

    assert success is True
    verify_index = calls.index(("verify", "adaptive_grip_complete"))
    first_sort_index = calls.index(("move", "front_neutral"))
    assert verify_index < first_sort_index
