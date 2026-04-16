"""
test_ik_virtual.py
==================
Virtual test harness for pi_kinematics.py.

Feeds fake camera coordinates into the IK solver, prints the JSON
output, and flags any suspicious jumps between nearby targets.

No motors or hardware required — pure math validation.
"""

import json
import math
import sys

from pi_kinematics import ArmIK


def step_to_deg(step: int) -> float:
    """Convert a Dynamixel step (0-4095) to degrees."""
    return step * (360.0 / 4096.0)


def print_divider(char: str = "─", width: int = 72):
    print(char * width)


def run_single_targets(arm: ArmIK):
    """Test a spread of individual target coordinates."""

    targets = [
        # (x,   y,    z)    — description
        (15,   10,    0),   # moderate reach, right side
        (20,   -5,    5),   # moderate reach, left side, elevated
        (30,    0,    0),   # far straight ahead
        (10,   10,   -5),   # close, right, below shoulder
        (25,   25,    0),   # far diagonal
        ( 5,    0,    0),   # very close, dead ahead
        (20,    0,   10),   # moderate reach, high up
        (15,  -15,    0),   # moderate reach, left side
        (10,    0,    0),   # close, dead ahead
        (35,    0,    0),   # near max reach — boundary test
    ]

    print("\n╔══════════════════════════════════════════════════════════════════╗")
    print("║           SINGLE-TARGET IK RESULTS                             ║")
    print("╚══════════════════════════════════════════════════════════════════╝\n")

    header = (
        f"{'Target (x,y,z)':>20s}  │  "
        f"{'m1':>5s}  {'m2':>5s}  {'m3':>5s}  {'m4':>5s}  {'m5':>5s}  │  "
        f"{'m1°':>6s} {'m2°':>6s} {'m3°':>6s} {'m4°':>6s} {'m5°':>6s}  │  Status"
    )
    print(header)
    print_divider()

    for t in targets:
        label = f"({t[0]:>3}, {t[1]:>3}, {t[2]:>3})"
        try:
            result = arm.solve(*t)
            m = [result["m1"], result["m2"], result["m3"], result["m4"], result["m5"]]
            degs = [step_to_deg(s) for s in m]

            # Basic sanity: all steps in [0, 4095]
            in_range = all(0 <= s <= 4095 for s in m)
            status = "✅" if in_range else "⚠️  OUT OF RANGE"

            print(
                f"{label:>20s}  │  "
                f"{m[0]:5d}  {m[1]:5d}  {m[2]:5d}  {m[3]:5d}  {m[4]:5d}  │  "
                f"{degs[0]:6.1f} {degs[1]:6.1f} {degs[2]:6.1f} {degs[3]:6.1f} {degs[4]:6.1f}  │  {status}"
            )
        except ValueError as e:
            print(f"{label:>20s}  │  {'---':>5s}  {'---':>5s}  {'---':>5s}  {'---':>5s}  {'---':>5s}  │  ❌ {e}")


def run_incremental_sweep(arm: ArmIK):
    """Sweep X from 10 to 35 in 1 cm steps (y=0, z=0).

    Flags any motor that jumps more than a threshold between 1 cm steps.
    """
    print("\n╔══════════════════════════════════════════════════════════════════╗")
    print("║           INCREMENTAL SWEEP  (x: 10→35, y=0, z=0)             ║")
    print("║           Flag if any motor jumps > 150 steps per 1 cm         ║")
    print("╚══════════════════════════════════════════════════════════════════╝\n")

    header = (
        f"{'x':>4s}  │  "
        f"{'m1':>5s}  {'m2':>5s}  {'m3':>5s}  {'m4':>5s}  {'m5':>5s}  │  "
        f"{'Δm1':>4s} {'Δm2':>4s} {'Δm3':>4s} {'Δm4':>4s} {'Δm5':>4s}  │  Status"
    )
    print(header)
    print_divider()

    prev = None
    jump_threshold = 150  # steps — suspicious for a 1 cm move

    for x in range(10, 36):
        try:
            result = arm.solve(float(x), 0.0, 0.0)
            m = [result["m1"], result["m2"], result["m3"], result["m4"], result["m5"]]

            if prev is not None:
                deltas = [abs(m[i] - prev[i]) for i in range(5)]
                flags = ["⚠️" if d > jump_threshold else "  " for d in deltas]
                any_flag = any(d > jump_threshold for d in deltas)
                status = "⚠️  BIG JUMP" if any_flag else "✅"

                print(
                    f"{x:4d}  │  "
                    f"{m[0]:5d}  {m[1]:5d}  {m[2]:5d}  {m[3]:5d}  {m[4]:5d}  │  "
                    f"{deltas[0]:4d} {deltas[1]:4d} {deltas[2]:4d} {deltas[3]:4d} {deltas[4]:4d}  │  {status}"
                )
            else:
                print(
                    f"{x:4d}  │  "
                    f"{m[0]:5d}  {m[1]:5d}  {m[2]:5d}  {m[3]:5d}  {m[4]:5d}  │  "
                    f"{'—':>4s} {'—':>4s} {'—':>4s} {'—':>4s} {'—':>4s}  │  (start)"
                )

            prev = m

        except ValueError as e:
            print(f"{x:4d}  │  {'UNREACHABLE':^30s}  │  ❌ {e}")
            prev = None


def run_partial_move_test(arm: ArmIK):
    """Test the two-step servoing at 0 %, 20 %, 40 %, …, 100 %."""

    target = (25.0, 10.0, 0.0)

    print("\n╔══════════════════════════════════════════════════════════════════╗")
    print(f"║   TWO-STEP SERVOING  target=({target[0]}, {target[1]}, {target[2]})              ║")
    print("║   Stepping from 0% to 100% in 20% increments                   ║")
    print("╚══════════════════════════════════════════════════════════════════╝\n")

    header = (
        f"{'%':>5s}  │  "
        f"{'m1':>5s}  {'m2':>5s}  {'m3':>5s}  {'m4':>5s}  {'m5':>5s}  │  JSON"
    )
    print(header)
    print_divider()

    for pct in [0.0, 0.20, 0.40, 0.60, 0.80, 1.00]:
        try:
            if pct == 0.0:
                # Origin / home — solver might fail if (0,0,0) is inside
                # the dead zone.  Just show the starting label.
                print(f"{int(pct*100):5d}  │  {'(home / origin)':^30s}  │  —")
                continue

            result = arm.calculate_partial_move(*target, percentage=pct)
            m = [result["m1"], result["m2"], result["m3"], result["m4"], result["m5"]]
            j = json.dumps(result)

            print(
                f"{int(pct*100):5d}  │  "
                f"{m[0]:5d}  {m[1]:5d}  {m[2]:5d}  {m[3]:5d}  {m[4]:5d}  │  {j}"
            )
        except ValueError as e:
            print(f"{int(pct*100):5d}  │  ❌ {e}")


def run_symmetry_test(arm: ArmIK):
    """Mirror targets across Y axis should only differ in m1 (base)."""

    pairs = [
        ((20,  10, 0), (20, -10, 0)),
        ((15,   5, 0), (15,  -5, 0)),
        ((30,  15, 5), (30, -15, 5)),
    ]

    print("\n╔══════════════════════════════════════════════════════════════════╗")
    print("║           SYMMETRY TEST  (mirror across Y=0)                   ║")
    print("║           m2, m3, m4, m5 should match; m1 should mirror         ║")
    print("╚══════════════════════════════════════════════════════════════════╝\n")

    for (a, b) in pairs:
        try:
            ra = arm.solve(*a)
            rb = arm.solve(*b)

            m2_match = ra["m2"] == rb["m2"]
            m3_match = ra["m3"] == rb["m3"]
            m4_match = ra["m4"] == rb["m4"]
            m5_match = ra["m5"] == rb["m5"]
            # m1 should be symmetric around 2048
            m1_sum = ra["m1"] + rb["m1"]
            m1_sym = abs(m1_sum - 4096) <= 2  # allow ±1 rounding

            ok = m2_match and m3_match and m4_match and m5_match and m1_sym
            status = "✅ symmetric" if ok else "⚠️  asymmetric"

            print(f"  {str(a):>18s}  →  m1={ra['m1']:4d}  m2={ra['m2']:4d}  m3={ra['m3']:4d}  m4={ra['m4']:4d}  m5={ra['m5']:4d}")
            print(f"  {str(b):>18s}  →  m1={rb['m1']:4d}  m2={rb['m2']:4d}  m3={rb['m3']:4d}  m4={rb['m4']:4d}  m5={rb['m5']:4d}")
            print(f"  {'':>18s}     m1 sum={m1_sum} (expect ≈4096)   {status}")
            print()

        except ValueError as e:
            print(f"  {a} / {b}  →  ❌ {e}\n")


# ── Entry point ───────────────────────────────────────────────────────
if __name__ == "__main__":
    arm = ArmIK()

    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║          pi_kinematics.py  —  VIRTUAL TEST SUITE               ║")
    print("╠══════════════════════════════════════════════════════════════════╣")
    print(f"║  L1 = {arm.L1:.1f} cm   L2 = {arm.L2:.1f} cm   L3 = {arm.L3:.1f} cm              ║")
    print(f"║  Sag multiplier = {arm.z_offset_multiplier}                                      ║")
    print(f"║  Max reach = {arm.L1 + arm.L2:.1f} cm   Min reach = {abs(arm.L1 - arm.L2):.1f} cm                ║")
    print("╚══════════════════════════════════════════════════════════════════╝")

    run_single_targets(arm)
    run_incremental_sweep(arm)
    run_partial_move_test(arm)
    run_symmetry_test(arm)

    print("\n" + "═" * 72)
    print("  All tests complete.  Review ⚠️  and ❌ flags above.")
    print("═" * 72)
