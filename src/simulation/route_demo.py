"""Rear-placement route simulation demo helpers and CLI."""

from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import dataclass
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.arm import (  # noqa: E402
    CLAW_CLOSED_POS,
    CLAW_OPEN_POS,
    RouteCalibrationError,
    load_transport_route_calibration,
)
from ik.solver import ArmIK  # noqa: E402
from simulation.mock_serial import MockSerial  # noqa: E402

logger = logging.getLogger(__name__)

SAMPLE_ROUTE_CALIBRATION = Path(__file__).resolve().parent / "sample_route_calibration.json"

DEFAULT_PICKUP_POSE = {
    "x": 24.0,
    "y": 0.0,
    "z": 20.0,
    "m4_offset": 0,
    "m5": CLAW_OPEN_POS,
    "skip_sag": True,
}

DEFAULT_SORT_SEQUENCE = ("RED_BIN", "BLUE_BIN")


@dataclass(frozen=True)
class RouteDemoWaypoint:
    """A prevalidated strict IK waypoint used by the route simulation."""

    name: str
    intent: str
    pose: dict
    commands: dict
    validation: dict


def _normalise_bin_key(destination: str) -> str:
    key = destination.upper().strip()
    if not key.endswith("_BIN"):
        key += "_BIN"
    return key


def _solve_demo_waypoint(
    arm: ArmIK,
    name: str,
    pose: dict,
    intent: str,
) -> RouteDemoWaypoint:
    try:
        solution = arm.solve_strict(pose, intent=intent)
    except ValueError as exc:
        raise ValueError(
            f"Route demo strict prevalidation failed at waypoint {name!r} "
            f"with intent {intent!r}: {exc}"
        ) from exc

    return RouteDemoWaypoint(
        name=name,
        intent=intent,
        pose=solution["validation"]["pose"],
        commands=solution["commands"],
        validation=solution["validation"],
    )


def _prepared_pickup_pose(pickup_pose: dict | None = None) -> dict:
    pickup = DEFAULT_PICKUP_POSE.copy()
    if pickup_pose:
        pickup.update(pickup_pose)
    pickup.setdefault("m4_offset", 0)
    pickup.setdefault("m5", CLAW_OPEN_POS)
    pickup.setdefault("skip_sag", True)
    return pickup


def _pickup_sequence_specs(
    pickup_pose: dict | None,
    pickup_clearance_z: float,
    *,
    close_name: str = "pickup-gripped",
    loaded: bool = True,
) -> list[tuple[str, dict, str]]:
    pickup = _prepared_pickup_pose(pickup_pose)

    pickup_clearance = pickup.copy()
    pickup_clearance["z"] = max(float(pickup["z"]) + 4.0, pickup_clearance_z)
    pickup_clearance["m5"] = CLAW_OPEN_POS

    pickup_closed = pickup.copy()
    pickup_closed["m5"] = CLAW_CLOSED_POS

    pickup_clearance_after = pickup_clearance.copy()
    pickup_clearance_after["m5"] = CLAW_CLOSED_POS if loaded else CLAW_OPEN_POS

    return [
        ("pickup-clearance", pickup_clearance, "carry"),
        ("pickup", pickup, "pickup"),
        (close_name, pickup_closed, "pickup"),
        ("pickup-clearance-retreat", pickup_clearance_after, "carry"),
    ]


def scan_look_again_command() -> dict:
    """Return the raw motor scan pose used after simulated no-grip."""
    from config.arm import SCAN_POSE

    return dict(SCAN_POSE)


def build_rear_placement_demo_plan(
    arm: ArmIK,
    calibration_path: Path | str,
    *,
    destination: str = "RED_BIN",
    pickup_pose: dict | None = None,
    pickup_clearance_z: float = 24.0,
    include_retreat: bool = True,
) -> list[RouteDemoWaypoint]:
    """Load route calibration and strictly prevalidate a visual demo plan.

    The route calibration is loaded through ``load_transport_route_calibration``
    with ``require_route_schema=True`` so legacy single-target bin files fail
    closed instead of being converted into guessed rear routes.
    """
    route_cal = load_transport_route_calibration(
        Path(calibration_path),
        require_route_schema=True,
    )
    destination_key = _normalise_bin_key(destination)
    if destination_key not in route_cal.bins:
        raise RouteCalibrationError(f"No route calibration for destination bin {destination_key}")

    bin_route = route_cal.bins[destination_key]
    front_neutral_loaded = route_cal.shared_waypoints["front_neutral"].as_strict_pose(
        m5=CLAW_CLOSED_POS
    )
    rear_transfer_loaded = route_cal.shared_waypoints["rear_transfer"].as_strict_pose(
        m5=CLAW_CLOSED_POS
    )
    bin_approach_loaded = bin_route.approach.as_strict_pose(m5=CLAW_CLOSED_POS)
    bin_drop_loaded = bin_route.drop.as_strict_pose(m5=CLAW_CLOSED_POS)
    bin_drop_released = bin_route.drop.as_strict_pose(m5=CLAW_OPEN_POS)
    for rear_pose in (rear_transfer_loaded, bin_approach_loaded, bin_drop_loaded, bin_drop_released):
        rear_pose["rear_base_yaw_limit_deg"] = route_cal.rear_base_yaw_limit_deg

    pickup_specs = _pickup_sequence_specs(
        pickup_pose,
        pickup_clearance_z,
        close_name="pickup-gripped",
        loaded=True,
    )

    specs: list[tuple[str, dict, str]] = [
        *pickup_specs,
        ("front-neutral", front_neutral_loaded, "carry"),
        ("rear-transfer", rear_transfer_loaded, "rear_place"),
        ("bin-approach", bin_approach_loaded, "rear_place"),
        ("bin-drop", bin_drop_loaded, "rear_place"),
        ("bin-drop-release", bin_drop_released, "rear_place"),
    ]

    if include_retreat:
        bin_approach_released = bin_route.approach.as_strict_pose(m5=CLAW_OPEN_POS)
        rear_transfer_released = route_cal.shared_waypoints["rear_transfer"].as_strict_pose(m5=CLAW_OPEN_POS)
        for rear_pose in (bin_approach_released, rear_transfer_released):
            rear_pose["rear_base_yaw_limit_deg"] = route_cal.rear_base_yaw_limit_deg
        specs.extend(
            [
                ("bin-approach-retreat", bin_approach_released, "rear_place"),
                ("rear-transfer-retreat", rear_transfer_released, "rear_place"),
                ("front-neutral-retreat", route_cal.shared_waypoints["front_neutral"].as_strict_pose(m5=CLAW_OPEN_POS), "carry"),
                ("pickup-clearance-home", pickup_specs[0][1], "carry"),
            ]
        )

    return [_solve_demo_waypoint(arm, name, pose, intent) for name, pose, intent in specs]


def build_air_pick_scan_demo_plan(
    arm: ArmIK,
    *,
    pickup_pose: dict | None = None,
    pickup_clearance_z: float = 24.0,
) -> list[RouteDemoWaypoint]:
    """Strictly prevalidate the simulated no-grip path back to scan.

    This path intentionally contains no bin waypoints.  It demonstrates the
    real flow for a failed grip: close on air, reopen while retreating, return
    to scan position, and look again rather than using a reject bin.
    """
    specs = _pickup_sequence_specs(
        pickup_pose,
        pickup_clearance_z,
        close_name="pickup-no-grip-close",
        loaded=False,
    )
    return [_solve_demo_waypoint(arm, name, pose, intent) for name, pose, intent in specs]


def build_sort_sequence_demo_plans(
    arm: ArmIK,
    calibration_path: Path | str,
    *,
    destinations: tuple[str, ...] = DEFAULT_SORT_SEQUENCE,
    pickup_pose: dict | None = None,
    pickup_clearance_z: float = 24.0,
) -> list[tuple[str, str, list[RouteDemoWaypoint]]]:
    """Build one validated rear-placement route for each requested colour."""
    sequence = []
    for destination in destinations:
        destination_key = _normalise_bin_key(destination)
        colour = destination_key.removesuffix("_BIN")
        plan = build_rear_placement_demo_plan(
            arm,
            calibration_path,
            destination=destination_key,
            pickup_pose=pickup_pose,
            pickup_clearance_z=pickup_clearance_z,
        )
        sequence.append((colour, destination_key, plan))
    return sequence


def route_overlay_points(plan: list[RouteDemoWaypoint]) -> list[tuple[str, dict]]:
    """Return ordered, labelled Cartesian waypoints for the visual corridor."""
    visible_names = {
        "pickup-clearance",
        "pickup",
        "pickup-clearance-retreat",
        "front-neutral",
        "rear-transfer",
        "bin-approach",
        "bin-drop",
        "bin-approach-retreat",
        "rear-transfer-retreat",
        "front-neutral-retreat",
        "pickup-clearance-home",
    }
    return [(waypoint.name, waypoint.pose) for waypoint in plan if waypoint.name in visible_names]


def print_prevalidation_summary(plan: list[RouteDemoWaypoint]) -> None:
    """Print the ordered strict IK waypoint validation summary."""
    print("\nStrict route prevalidation complete before animation/execution:")
    for idx, waypoint in enumerate(plan, start=1):
        pose = waypoint.pose
        validation = waypoint.validation
        print(
            f"  {idx:02d}. {waypoint.name:<24} "
            f"intent={waypoint.intent:<10} "
            f"pose=({pose['x']:>5.1f}, {pose['y']:>5.1f}, {pose['z']:>5.1f}) cm  "
            f"m=[{waypoint.commands['m1']}, {waypoint.commands['m2']}, "
            f"{waypoint.commands['m3']}, {waypoint.commands['m4']}, {waypoint.commands['m5']}]  "
            f"yaw={validation['base_yaw_deg']:.1f}°  "
            f"shoulder={validation['theta_shoulder_deg']:.1f}°  "
            f"pitch={validation['theta_pitch_deg']:.1f}°"
        )


def print_air_pick_summary(plan: list[RouteDemoWaypoint]) -> None:
    """Print a no-grip prevalidation summary and explicit scan return."""
    print_prevalidation_summary(plan)
    scan = scan_look_again_command()
    print(
        "\nSimulated air-pick/no-grip: no object was gripped; "
        "returning to SCAN_POSE for scan/look-again instead of using any bin."
    )
    print(
        f"  scan/look-again m=[{scan['m1']}, {scan['m2']}, {scan['m3']}, {scan['m4']}, {scan['m5']}]"
    )


def execute_demo_plan(
    plan: list[RouteDemoWaypoint],
    *,
    move_delay: float = 0.8,
    frames: int = 24,
) -> None:
    """Animate the already prevalidated route plan through MockSerial."""
    from simulation.visualizer import ArmVisualizer

    viz = ArmVisualizer()
    viz.draw_route_waypoints(route_overlay_points(plan))
    ser = MockSerial(move_delay=move_delay, visualizer=viz, anim_frames=frames)

    print("\nAnimating from SCAN_POSE into the prevalidated rear-placement route...")
    ser.write((json.dumps(scan_look_again_command()) + "\n").encode())
    ser.readline()

    for idx, waypoint in enumerate(plan, start=1):
        print(f"  ▶ {idx:02d}. {waypoint.name} ({waypoint.intent})")
        ser.write((json.dumps(waypoint.commands) + "\n").encode())
        response = ser.readline().decode().strip()
        if response != "OK":
            logger.warning("Unexpected mock serial response at %s: %s", waypoint.name, response)

    viz.close()


def execute_sort_sequence(
    sequence: list[tuple[str, str, list[RouteDemoWaypoint]]],
    *,
    move_delay: float = 0.8,
    frames: int = 24,
) -> None:
    """Animate red/blue sorting routes in one visualizer session."""
    from simulation.visualizer import ArmVisualizer

    viz = ArmVisualizer()
    ser = MockSerial(move_delay=move_delay, visualizer=viz, anim_frames=frames)
    scan = scan_look_again_command()

    print("\nAnimating from SCAN_POSE into the rear two-bin sorting sequence...")
    ser.write((json.dumps(scan) + "\n").encode())
    ser.readline()

    for colour, destination, plan in sequence:
        print(f"\nSorting {colour} ball into {destination} (rear bin).")
        viz.draw_route_waypoints(route_overlay_points(plan), title=f"{colour} → {destination}")
        for idx, waypoint in enumerate(plan, start=1):
            print(f"  ▶ {idx:02d}. {waypoint.name} ({waypoint.intent})")
            ser.write((json.dumps(waypoint.commands) + "\n").encode())
            response = ser.readline().decode().strip()
            if response != "OK":
                logger.warning("Unexpected mock serial response at %s: %s", waypoint.name, response)

    viz.close()


def execute_air_pick_scan_demo(
    plan: list[RouteDemoWaypoint],
    *,
    move_delay: float = 0.8,
    frames: int = 24,
) -> None:
    """Animate a failed grip and return to scan/look-again."""
    from simulation.visualizer import ArmVisualizer

    viz = ArmVisualizer()
    viz.draw_route_waypoints(route_overlay_points(plan), title="No-grip pickup → scan/look-again")
    ser = MockSerial(move_delay=move_delay, visualizer=viz, anim_frames=frames)
    scan = scan_look_again_command()

    print("\nAnimating simulated air-pick/no-grip path from SCAN_POSE...")
    ser.write((json.dumps(scan) + "\n").encode())
    ser.readline()

    for idx, waypoint in enumerate(plan, start=1):
        print(f"  ▶ {idx:02d}. {waypoint.name} ({waypoint.intent})")
        ser.write((json.dumps(waypoint.commands) + "\n").encode())
        response = ser.readline().decode().strip()
        if response != "OK":
            logger.warning("Unexpected mock serial response at %s: %s", waypoint.name, response)

    print("  ✗ No object gripped; returning to SCAN_POSE to scan/look-again (no reject bin).")
    ser.write((json.dumps(scan) + "\n").encode())
    ser.readline()
    viz.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Visualize and strictly prevalidate the rear-placement transport route."
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        required=True,
        help="Route-schema calibration JSON to load explicitly. Use src/simulation/sample_route_calibration.json for the safe simulation sample.",
    )
    parser.add_argument("--destination", default="RED_BIN", help="Single destination bin or colour, e.g. RED_BIN or BLUE_BIN")
    parser.add_argument("--sequence", action="store_true", help="Sort a red ball and then a blue ball into the two rear bins")
    parser.add_argument("--air-pick", action="store_true", help="Simulate a failed/no-grip pickup and return to scan/look-again")
    parser.add_argument("--pickup-x", type=float, default=DEFAULT_PICKUP_POSE["x"], help="Demo pickup X coordinate in cm")
    parser.add_argument("--pickup-y", type=float, default=DEFAULT_PICKUP_POSE["y"], help="Demo pickup Y coordinate in cm")
    parser.add_argument("--pickup-z", type=float, default=DEFAULT_PICKUP_POSE["z"], help="Demo pickup Z coordinate in cm")
    parser.add_argument("--no-gui", action="store_true", help="Only load and prevalidate; do not open matplotlib")
    parser.add_argument("--move-delay", type=float, default=0.8, help="Seconds per simulated waypoint move")
    parser.add_argument("--frames", type=int, default=24, help="Animation frames per simulated waypoint move")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    pickup_pose = {
        "x": args.pickup_x,
        "y": args.pickup_y,
        "z": args.pickup_z,
        "m5": CLAW_OPEN_POS,
        "m4_offset": 0,
        "skip_sag": True,
    }

    try:
        arm = ArmIK()
        if args.air_pick:
            plan = build_air_pick_scan_demo_plan(arm, pickup_pose=pickup_pose)
            sequence = None
        elif args.sequence:
            sequence = build_sort_sequence_demo_plans(
                arm,
                args.calibration,
                pickup_pose=pickup_pose,
            )
            plan = None
        else:
            plan = build_rear_placement_demo_plan(
                arm,
                args.calibration,
                destination=args.destination,
                pickup_pose=pickup_pose,
            )
            sequence = None
    except (RouteCalibrationError, ValueError) as exc:
        logger.error("Rear-placement route simulation failed closed: %s", exc)
        return 2

    if args.air_pick:
        print_air_pick_summary(plan)
    elif args.sequence:
        for colour, destination, route_plan in sequence:
            print(f"\nSorting {colour} ball into {destination} (rear bin).")
            print_prevalidation_summary(route_plan)
    else:
        destination_key = _normalise_bin_key(args.destination)
        print(f"\nSorting {destination_key.removesuffix('_BIN')} ball into {destination_key} (rear bin).")
        print_prevalidation_summary(plan)

    if args.no_gui:
        print("\n--no-gui selected; route was validated but not animated.")
        return 0

    if args.air_pick:
        execute_air_pick_scan_demo(plan, move_delay=args.move_delay, frames=args.frames)
    elif args.sequence:
        execute_sort_sequence(sequence, move_delay=args.move_delay, frames=args.frames)
    else:
        execute_demo_plan(plan, move_delay=args.move_delay, frames=args.frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
