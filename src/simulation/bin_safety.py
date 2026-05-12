"""Shared bin geometry and claw/bin collision safety helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence

from ik.solver import ArmIK


SHOULDER_SERVO_CENTER_BELOW_SHOULDER_CM = 1.5
SHOULDER_SERVO_HEIGHT_CM = 3.5

# The static base cylinder in the visualizer reaches from the table plane to the
# bottom of the shoulder servo.  Bin collision models use the separately measured
# physical bin height so the safety boundary matches the desk-to-rim distance.
ROBOT_BASE_HEIGHT_CM = (
    ArmIK.shoulder_height
    - SHOULDER_SERVO_CENTER_BELOW_SHOULDER_CM
    - SHOULDER_SERVO_HEIGHT_CM / 2.0
)

PHYSICAL_BIN_HEIGHT_CM = 30.5
SIMULATION_BIN_FOOTPRINT_CM = 7.0
CLAW_BIN_COLLISION_TOLERANCE_CM = 1e-9
SAG_AWARE_BIN_CLEARANCE_CM = 5.0


@dataclass(frozen=True)
class BinVolume:
    """Axis-aligned bin collision volume in the arm workspace."""

    name: str
    x: float
    y: float
    height: float = PHYSICAL_BIN_HEIGHT_CM
    footprint_cm: float = SIMULATION_BIN_FOOTPRINT_CM

    @property
    def half_extent_cm(self) -> float:
        return self.footprint_cm / 2.0

    @property
    def bounds(self) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        half = self.half_extent_cm
        return (
            (self.x - half, self.x + half),
            (self.y - half, self.y + half),
            (0.0, self.height),
        )

    def point_clearance_cm(self, point: Sequence[float]) -> float:
        """Return Euclidean clearance from *point* to this bin volume."""

        px, py, pz = (float(point[0]), float(point[1]), float(point[2]))
        (min_x, max_x), (min_y, max_y), (min_z, max_z) = self.bounds
        dx = max(min_x - px, 0.0, px - max_x)
        dy = max(min_y - py, 0.0, py - max_y)
        dz = max(min_z - pz, 0.0, pz - max_z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def contains_point(
        self,
        point: Sequence[float],
        *,
        tolerance_cm: float = CLAW_BIN_COLLISION_TOLERANCE_CM,
    ) -> bool:
        """Return True when *point* touches or lies inside this bin volume."""

        px, py, pz = (float(point[0]), float(point[1]), float(point[2]))
        (min_x, max_x), (min_y, max_y), (min_z, max_z) = self.bounds
        return (
            min_x - tolerance_cm <= px <= max_x + tolerance_cm
            and min_y - tolerance_cm <= py <= max_y + tolerance_cm
            and min_z - tolerance_cm <= pz <= max_z + tolerance_cm
        )

    def segment_intersects(
        self,
        start: Sequence[float],
        end: Sequence[float],
        *,
        tolerance_cm: float = CLAW_BIN_COLLISION_TOLERANCE_CM,
    ) -> bool:
        """Return True when the closed segment touches/intersects the bin."""

        p0 = [float(start[0]), float(start[1]), float(start[2])]
        p1 = [float(end[0]), float(end[1]), float(end[2])]
        bounds = self.bounds
        t_min = 0.0
        t_max = 1.0

        for axis in range(3):
            axis_min = bounds[axis][0] - tolerance_cm
            axis_max = bounds[axis][1] + tolerance_cm
            delta = p1[axis] - p0[axis]
            if abs(delta) <= 1e-12:
                if p0[axis] < axis_min or p0[axis] > axis_max:
                    return False
                continue

            t1 = (axis_min - p0[axis]) / delta
            t2 = (axis_max - p0[axis]) / delta
            enter = min(t1, t2)
            exit_ = max(t1, t2)
            t_min = max(t_min, enter)
            t_max = min(t_max, exit_)
            if t_min - t_max > 1e-12:
                return False

        return t_max >= -1e-12 and t_min <= 1.0 + 1e-12

    def segment_clearance_cm(
        self,
        start: Sequence[float],
        end: Sequence[float],
    ) -> float:
        """Return minimum Euclidean clearance from a segment to this bin volume."""

        start_xyz = [float(start[0]), float(start[1]), float(start[2])]
        end_xyz = [float(end[0]), float(end[1]), float(end[2])]
        if self.segment_intersects(start_xyz, end_xyz, tolerance_cm=0.0):
            return 0.0

        delta = [end_xyz[axis] - start_xyz[axis] for axis in range(3)]

        def clearance_at(t: float) -> float:
            point = [start_xyz[axis] + delta[axis] * t for axis in range(3)]
            return self.point_clearance_cm(point)

        # The squared distance from a line segment to an axis-aligned box is a
        # convex one-dimensional function over t∈[0, 1].  Ternary search gives a
        # deterministic, geometry-independent minimum without adding heavy deps.
        low = 0.0
        high = 1.0
        for _ in range(80):
            left = low + (high - low) / 3.0
            right = high - (high - low) / 3.0
            if clearance_at(left) <= clearance_at(right):
                high = right
            else:
                low = left

        candidates = (0.0, 1.0, (low + high) / 2.0)
        return min(clearance_at(t) for t in candidates)


@dataclass(frozen=True)
class BinClearanceViolation:
    """Sag-aware minimum-clearance violation for a claw pose or segment."""

    bin_volume: BinVolume
    clearance_cm: float
    min_clearance_cm: float


SIMULATION_REAR_BINS = {
    "RED_BIN": (-28.0, -7.0, PHYSICAL_BIN_HEIGHT_CM),
    "BLUE_BIN": (-28.0, 7.0, PHYSICAL_BIN_HEIGHT_CM),
}


def pose_xyz(pose: Any) -> tuple[float, float, float]:
    """Extract an ``(x, y, z)`` tuple from a pose mapping/object/sequence."""

    if isinstance(pose, Mapping):
        return (float(pose["x"]), float(pose["y"]), float(pose["z"]))
    if isinstance(pose, Sequence) and not isinstance(pose, (str, bytes)):
        if len(pose) < 3:
            raise ValueError("pose sequence must contain at least x, y, z")
        return (float(pose[0]), float(pose[1]), float(pose[2]))
    return (float(pose.x), float(pose.y), float(pose.z))


def bin_volumes_from_centres(
    centres: Mapping[str, Any],
    *,
    height_cm: float = PHYSICAL_BIN_HEIGHT_CM,
    footprint_cm: float = SIMULATION_BIN_FOOTPRINT_CM,
) -> tuple[BinVolume, ...]:
    """Build bin volumes from calibrated/demonstration bin centre poses."""

    volumes: list[BinVolume] = []
    for name, centre in centres.items():
        x, y, _z = pose_xyz(centre)
        volumes.append(
            BinVolume(
                name=str(name),
                x=x,
                y=y,
                height=float(height_cm),
                footprint_cm=float(footprint_cm),
            )
        )
    return tuple(volumes)


def simulation_bin_volumes() -> tuple[BinVolume, ...]:
    """Return the fixed visual-demo rear bin volumes."""

    return bin_volumes_from_centres(
        {
            name: {"x": coords[0], "y": coords[1], "z": coords[2]}
            for name, coords in SIMULATION_REAR_BINS.items()
        }
    )


def find_claw_bin_point_collision(
    point: Any,
    bins: Iterable[BinVolume],
) -> BinVolume | None:
    """Return the first bin touched by a claw point, if any."""

    xyz = pose_xyz(point)
    for bin_volume in bins:
        if bin_volume.contains_point(xyz):
            return bin_volume
    return None


def find_claw_bin_segment_collision(
    start: Any,
    end: Any,
    bins: Iterable[BinVolume],
) -> BinVolume | None:
    """Return the first bin touched by a claw path segment, if any."""

    start_xyz = pose_xyz(start)
    end_xyz = pose_xyz(end)
    for bin_volume in bins:
        if bin_volume.segment_intersects(start_xyz, end_xyz):
            return bin_volume
    return None


def find_claw_bin_point_clearance_violation(
    point: Any,
    bins: Iterable[BinVolume],
    *,
    min_clearance_cm: float = SAG_AWARE_BIN_CLEARANCE_CM,
    tolerance_cm: float = CLAW_BIN_COLLISION_TOLERANCE_CM,
) -> BinClearanceViolation | None:
    """Return the first bin whose clearance to a claw point is too small."""

    xyz = pose_xyz(point)
    for bin_volume in bins:
        clearance_cm = bin_volume.point_clearance_cm(xyz)
        if clearance_cm + tolerance_cm < float(min_clearance_cm):
            return BinClearanceViolation(
                bin_volume=bin_volume,
                clearance_cm=clearance_cm,
                min_clearance_cm=float(min_clearance_cm),
            )
    return None


def find_claw_bin_segment_clearance_violation(
    start: Any,
    end: Any,
    bins: Iterable[BinVolume],
    *,
    min_clearance_cm: float = SAG_AWARE_BIN_CLEARANCE_CM,
    tolerance_cm: float = CLAW_BIN_COLLISION_TOLERANCE_CM,
) -> BinClearanceViolation | None:
    """Return the first bin whose clearance to a claw segment is too small."""

    start_xyz = pose_xyz(start)
    end_xyz = pose_xyz(end)
    for bin_volume in bins:
        clearance_cm = bin_volume.segment_clearance_cm(start_xyz, end_xyz)
        if clearance_cm + tolerance_cm < float(min_clearance_cm):
            return BinClearanceViolation(
                bin_volume=bin_volume,
                clearance_cm=clearance_cm,
                min_clearance_cm=float(min_clearance_cm),
            )
    return None


def format_claw_bin_collision_reason(bin_volume: BinVolume, *, context: str) -> str:
    """Return a human-readable fail-closed collision reason."""

    return (
        f"{context} touches/intersects {bin_volume.name} bin volume "
        f"(footprint {bin_volume.footprint_cm:.1f}×{bin_volume.footprint_cm:.1f} cm, "
        f"height {bin_volume.height:.2f} cm measured from desk); "
        "arm would sit in the bin and break"
    )


def format_claw_bin_clearance_reason(violation: BinClearanceViolation, *, context: str) -> str:
    """Return a human-readable sag-aware clearance rejection reason."""

    bin_volume = violation.bin_volume
    return (
        f"{context} clearance to {bin_volume.name} bin volume is only "
        f"{violation.clearance_cm:.2f} cm "
        f"(footprint {bin_volume.footprint_cm:.1f}×{bin_volume.footprint_cm:.1f} cm, "
        f"height {bin_volume.height:.2f} cm measured from desk); "
        f"requires at least {violation.min_clearance_cm:.2f} cm sag-aware clearance "
        "because real arm sag makes smaller margins unsafe; "
        "arm could sit in the bin and break"
    )
