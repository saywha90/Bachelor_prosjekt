"""
visualizer.py
=============
Live 3D visualiser for the robotic arm.

Uses *forward kinematics* to convert Dynamixel motor steps back to
Cartesian joint positions and draws the arm in a live ``matplotlib``
3D plot.

The workspace bins from ``config/arm.py`` are drawn as coloured markers so
you can watch the arm move between them in real time.

Dependencies: matplotlib, numpy
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import logging
import math
import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)
from mpl_toolkits.mplot3d import Axes3D       # noqa: F401  (registers 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from config.arm import BINS, HOME_POSITION


# ─── Constants (must match solver.py) ──────────────────────────────
L1 = 25.5   # shoulder → elbow  (cm)  (must match solver.py)
L2 = 23.0   # elbow   → wrist   (cm)  (must match solver.py)
L3 = 16.5   # wrist   → claw tip (cm)  (must match solver.py)

STEP_CENTRE   = 2048
RAD_PER_STEP  = (2.0 * math.pi) / 4096.0

SHOULDER_HEIGHT = 33.0   # matches ArmIK default — base to shoulder (cm)


# ─── Forward Kinematics ──────────────────────────────────────────────
def steps_to_rad(steps: int) -> float:
    """Convert Dynamixel steps to radians relative to centre (2048)."""
    return (steps - STEP_CENTRE) * RAD_PER_STEP


def forward_kinematics(m1: int, m2: int, m3: int, m4: int, m5: int = 2048):
    """Return the 3-D positions of each joint given motor step values.

    Returns
    -------
    list of (x, y, z) tuples
        [base, shoulder, elbow, wrist, claw_tip]

    The coordinate system matches the IK solver:
        x = forward, y = left/right, z = up.
    """
    theta_base     =  steps_to_rad(m1)
    theta_shoulder =  steps_to_rad(m2)
    theta_elbow    = -steps_to_rad(m3)   # negative convention (see IK)
    theta_wrist    =  steps_to_rad(m4)

    # Base is at the origin
    base = (0.0, 0.0, 0.0)

    # Shoulder sits at the base, raised by shoulder_height
    shoulder = (0.0, 0.0, SHOULDER_HEIGHT)

    # --- Work in the arm's 2-D vertical plane, then rotate by base angle ---
    # Shoulder → Elbow
    # theta_shoulder is measured from horizontal in the arm plane
    elbow_r = L1 * math.cos(theta_shoulder)
    elbow_z = L1 * math.sin(theta_shoulder) + SHOULDER_HEIGHT

    # Elbow → Wrist
    # The elbow deflects *down* from the shoulder line
    wrist_angle_abs = theta_shoulder - theta_elbow
    wrist_r = elbow_r + L2 * math.cos(wrist_angle_abs)
    wrist_z = elbow_z + L2 * math.sin(wrist_angle_abs)

    # Wrist → Claw tip
    # theta_wrist from the IK solver already accounts for the claw pointing
    # downward, so no additional offset is needed here.
    tip_angle_abs = wrist_angle_abs + theta_wrist
    tip_r = wrist_r + L3 * math.cos(tip_angle_abs)
    tip_z = wrist_z + L3 * math.sin(tip_angle_abs)

    # Rotate planar (r) distances into (x, y) using the base angle
    cos_b = math.cos(theta_base)
    sin_b = math.sin(theta_base)

    elbow = (elbow_r * cos_b, elbow_r * sin_b, elbow_z)
    wrist = (wrist_r * cos_b, wrist_r * sin_b, wrist_z)
    tip   = (tip_r   * cos_b, tip_r   * sin_b, tip_z)

    return [base, shoulder, elbow, wrist, tip]


# ─── Bin box helper ──────────────────────────────────────────────────
def _box_vertices(cx, cy, cz, sx=6, sy=6, sz=6):
    """Return 6 faces (each 4 vertices) for a wireframe box centred at (cx, cy, cz)."""
    dx, dy, dz = sx / 2, sy / 2, sz / 2
    # 8 corners
    c = np.array([
        [cx - dx, cy - dy, cz - dz],
        [cx + dx, cy - dy, cz - dz],
        [cx + dx, cy + dy, cz - dz],
        [cx - dx, cy + dy, cz - dz],
        [cx - dx, cy - dy, cz + dz],
        [cx + dx, cy - dy, cz + dz],
        [cx + dx, cy + dy, cz + dz],
        [cx - dx, cy + dy, cz + dz],
    ])
    faces = [
        [c[0], c[1], c[2], c[3]],   # bottom
        [c[4], c[5], c[6], c[7]],   # top
        [c[0], c[1], c[5], c[4]],   # front
        [c[2], c[3], c[7], c[6]],   # back
        [c[0], c[3], c[7], c[4]],   # left
        [c[1], c[2], c[6], c[5]],   # right
    ]
    return faces


# ─── Visualiser class ────────────────────────────────────────────────
class ArmVisualizer:
    """Live 3-D visualiser for the 4-DOF robotic arm."""

    # Colour palette
    BIN_COLOURS = {
        "RED_BIN":    "#e74c3c",
        "BLUE_BIN":   "#3498db",
        "REJECT_BIN": "#95a5a6",
    }

    def __init__(self):
        plt.ion()   # interactive mode — non-blocking show

        self.fig = plt.figure(figsize=(10, 8))
        self.fig.patch.set_facecolor("#1a1a2e")
        self.ax = self.fig.add_subplot(111, projection="3d")

        self._style_axes()
        self._draw_environment()

        # Arm line + joint markers (initialise empty)
        self.arm_line, = self.ax.plot(
            [], [], [], "-o",
            color="#00e676", linewidth=3, markersize=8,
            markerfacecolor="#ffffff", markeredgecolor="#00e676",
            markeredgewidth=2, zorder=10,
        )
        # Claw tip gets a special marker
        self.tip_marker, = self.ax.plot(
            [], [], [], "v",
            color="#ff9100", markersize=14, markeredgecolor="#fff",
            markeredgewidth=1.5, zorder=11,
        )

        # Ghost trail (faint previous positions)
        self._trail_xs = []
        self._trail_ys = []
        self._trail_zs = []
        self.trail_line, = self.ax.plot(
            [], [], [], ".", color="#00e676", alpha=0.15, markersize=3,
        )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show(block=False)
        plt.pause(0.1)

    # ── Styling ──────────────────────────────────────────────────────
    def _style_axes(self):
        ax = self.ax
        ax.set_facecolor("#16213e")

        ax.set_xlim(-10, 60)
        ax.set_ylim(-50, 50)
        ax.set_zlim(0, 70)

        ax.set_xlabel("X  (forward) cm", color="#aaa", fontsize=9, labelpad=8)
        ax.set_ylabel("Y  (left/right) cm", color="#aaa", fontsize=9, labelpad=8)
        ax.set_zlabel("Z  (up) cm", color="#aaa", fontsize=9, labelpad=8)

        ax.tick_params(colors="#666", labelsize=7)
        ax.set_title(
            "AUTONOMIA — Live Arm Simulation",
            color="#e0e0e0", fontsize=14, fontweight="bold", pad=16,
        )

        # Grid styling
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("#333")
        ax.yaxis.pane.set_edgecolor("#333")
        ax.zaxis.pane.set_edgecolor("#333")
        ax.grid(True, alpha=0.2, color="#555")

    def _draw_environment(self):
        """Draw bins as semi-transparent boxes and the ground plane."""
        ax = self.ax

        # Ground plane (faint grid)
        gx = np.linspace(-10, 45, 12)
        gy = np.linspace(-35, 35, 14)
        gx, gy = np.meshgrid(gx, gy)
        gz = np.zeros_like(gx)
        ax.plot_surface(gx, gy, gz, alpha=0.06, color="#4fc3f7", zorder=0)

        # Draw each bin as a wireframe box
        for name, (bx, by, bz) in BINS.items():
            colour = self.BIN_COLOURS.get(name, "#ffffff")
            faces = _box_vertices(bx, by, bz / 2, sx=7, sy=7, sz=bz)
            poly = Poly3DCollection(
                faces, alpha=0.15, facecolor=colour,
                edgecolor=colour, linewidth=0.8,
            )
            ax.add_collection3d(poly)
            # Label
            ax.text(bx, by, bz + 2, name.replace("_", " "),
                    color=colour, fontsize=8, ha="center", fontweight="bold")

        # Mark HOME
        hx, hy, hz = HOME_POSITION
        ax.scatter([hx], [hy], [hz], marker="^", s=100,
                   color="#ffd740", edgecolors="#fff", linewidths=1.2, zorder=5)
        ax.text(hx, hy, hz + 3, "HOME", color="#ffd740", fontsize=8,
                ha="center", fontweight="bold")

        # Base origin marker
        ax.scatter([0], [0], [0], marker="s", s=120,
                   color="#b388ff", edgecolors="#fff", linewidths=1.5, zorder=5)
        ax.text(0, 0, 3, "BASE", color="#b388ff", fontsize=8,
                ha="center", fontweight="bold")

    # ── Update arm pose ──────────────────────────────────────────────
    def update_plot(self, steps: dict):
        """Redraw the arm at the given motor step positions.

        Parameters
        ----------
        steps : dict
            ``{\"m1\": int, \"m2\": int, \"m3\": int, \"m4\": int}``
        """
        joints = forward_kinematics(steps["m1"], steps["m2"], steps["m3"], steps["m4"], steps["m5"])

        xs = [j[0] for j in joints]
        ys = [j[1] for j in joints]
        zs = [j[2] for j in joints]

        self.arm_line.set_data_3d(xs, ys, zs)
        self.tip_marker.set_data_3d([xs[-1]], [ys[-1]], [zs[-1]])

        # Ghost trail — record claw tip
        self._trail_xs.append(xs[-1])
        self._trail_ys.append(ys[-1])
        self._trail_zs.append(zs[-1])
        # Keep trail length manageable
        MAX_TRAIL = 300
        if len(self._trail_xs) > MAX_TRAIL:
            self._trail_xs = self._trail_xs[-MAX_TRAIL:]
            self._trail_ys = self._trail_ys[-MAX_TRAIL:]
            self._trail_zs = self._trail_zs[-MAX_TRAIL:]
        self.trail_line.set_data_3d(self._trail_xs, self._trail_ys, self._trail_zs)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    # ── Cleanup ──────────────────────────────────────────────────────
    def close(self):
        """Leave the plot open at the end so the user can inspect it."""
        plt.ioff()
        logger.info("[VISUALIZER] Simulation complete — close the plot window to exit.")
        plt.show()


# ── Quick self-test ──────────────────────────────────────────────────
if __name__ == "__main__":
    from ik.solver import ArmIK

    arm = ArmIK()
    viz = ArmVisualizer()

    # Walk the arm across a few test positions
    test_targets = [
        HOME_POSITION,
        (20.0,  5.0,  0.0),
        (25.0, -10.0, 0.0),
        (30.0,  20.0, 12.0),
        (30.0, -20.0, 12.0),
        (15.0,  25.0, 12.0),
        HOME_POSITION,
    ]

    import time
    for t in test_targets:
        steps = arm.solve(*t)
        logger.info("Target %s → steps %s", t, steps)
        viz.update_plot(steps)
        time.sleep(1.0)

    viz.close()
