"""
visualizer.py
=============
Live 3D visualiser for the robotic arm.

Renders the arm as a realistic representation matching the physical
Dynamixel-based 4-DOF arm: cylindrical base platform, rectangular servo
housings at joints, flat bracket-style link beams, and a two-jaw gripper.

Uses *forward kinematics* to convert Dynamixel motor steps back to
Cartesian joint positions and draws the arm in a live ``matplotlib``
3D plot using ``Poly3DCollection`` for all volumetric shapes.

The workspace bins from ``config/arm.py`` are drawn as coloured markers so
you can watch the arm move between them in real time.

Performance optimisations applied (v2):
  - Base cylinder drawn once in _draw_environment() (static geometry)
  - Polygon count reduced: cylinders 14→8 sides, cables removed
  - Single batched Poly3DCollection for the entire arm (was 10+ separate)
  - set_verts() / set_facecolors() reuse on subsequent frames
  - Ghost trail reuses line object via set_data_3d() (no recreate)
  - Blocking draw() + flush_events() with plt.pause(0.02) for reliable
    frame rendering (~50 fps cap); draw_idle() was skipping frames on macOS

Dependencies: matplotlib, numpy
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import logging
import math
import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)
from mpl_toolkits.mplot3d import Axes3D            # noqa: F401  (registers 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from config.arm import BINS, HOME_POSITION, SCAN_POSE


# ─── Constants (must match solver.py) ──────────────────────────────
L1 = 25.5          # shoulder → elbow  (cm)
L2 = 23.0          # elbow   → wrist   (cm)
L3 = 20.5          # wrist   → claw tip (cm)

STEP_CENTRE   = 2048
RAD_PER_STEP  = (2.0 * math.pi) / 4096.0

SHOULDER_HEIGHT = 11.0   # base to shoulder joint (cm)

# Reduced polygon count for cylinders (was 14)
_CYLINDER_SIDES = 8


# ─── Colour palette ─────────────────────────────────────────────────
COLORS = {
    'base':         '#F5F0E0',   # Cream/white base platform
    'servo':        '#2A2A2A',   # Dark grey servo housings
    'link':         '#1A1A1A',   # Black arm segments
    'claw':         '#888888',   # Metallic grey gripper
    'claw_tip':     '#ff9100',   # Orange tip marker
    'cable_yellow': '#FFD700',   # Yellow cable
    'cable_white':  '#FFFFFF',   # White cable
    'cable_green':  '#00AA00',   # Green cable
    'ground':       '#2a2a2a',   # Ground plane
    'trail':        '#00e676',   # Ghost trail
}


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

    Angle conventions (must invert the IK solver's step encoding):
        IK: m2 = steps(theta_shoulder - pi/2)  → FK: theta_shoulder = steps_to_rad(m2) + pi/2
        IK: m3 = steps(-theta_elbow)           → FK: theta_elbow    = -steps_to_rad(m3)
        IK: m4 = steps(theta_wrist)            → FK: theta_wrist    = steps_to_rad(m4)
    """
    theta_base     =  steps_to_rad(m1)
    # m2=2048 → arm vertical (straight up); add pi/2 to recover IK's
    # "elevation above horizontal" convention where 0 = horizontal.
    theta_shoulder =  steps_to_rad(m2) + math.pi / 2
    theta_elbow    = -steps_to_rad(m3)   # negative convention (see IK)
    theta_wrist    =  steps_to_rad(m4)

    # Base is at the origin
    base = (0.0, 0.0, 0.0)

    # Shoulder sits at the base, raised by shoulder_height
    shoulder = (0.0, 0.0, SHOULDER_HEIGHT)

    # --- Work in the arm's 2-D vertical plane, then rotate by base angle ---
    # Shoulder → Elbow
    elbow_r = L1 * math.cos(theta_shoulder)
    elbow_z = L1 * math.sin(theta_shoulder) + SHOULDER_HEIGHT

    # Elbow → Wrist
    wrist_angle_abs = theta_shoulder - theta_elbow
    wrist_r = elbow_r + L2 * math.cos(wrist_angle_abs)
    wrist_z = elbow_z + L2 * math.sin(wrist_angle_abs)

    # Wrist → Claw tip
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


# ─── Geometry helpers ─────────────────────────────────────────────────

def _rotation_matrix_z(angle: float) -> np.ndarray:
    """3×3 rotation matrix about the Z axis."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def _rotation_align(direction: np.ndarray) -> np.ndarray:
    """Return a 3×3 rotation matrix that aligns the X axis to *direction*.

    The resulting matrix maps:
        X → direction (normalised)
        Y → perpendicular in the horizontal plane
        Z → completes right-hand system
    """
    d = direction / (np.linalg.norm(direction) + 1e-12)
    # Choose an up reference that is not parallel to d
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(d, up)) > 0.99:
        up = np.array([0.0, 1.0, 0.0])
    y = np.cross(d, up)
    y /= (np.linalg.norm(y) + 1e-12)
    z = np.cross(d, y)
    z /= (np.linalg.norm(z) + 1e-12)
    # Columns: d(X), y(Y), z(Z) → but we want rows for transforming
    return np.column_stack([d, y, z])


def _create_box(center: np.ndarray, size: tuple, rot: np.ndarray = None) -> list:
    """Return 6 quad faces for an axis-aligned box, optionally rotated.

    Parameters
    ----------
    center : (3,) array – centre of the box
    size   : (sx, sy, sz) – full extents along each axis
    rot    : (3,3) rotation matrix applied before translation

    Returns
    -------
    list of 6 numpy arrays, each (4,3)
    """
    sx, sy, sz = [s / 2.0 for s in size]
    # 8 corners in local frame
    corners = np.array([
        [-sx, -sy, -sz],
        [+sx, -sy, -sz],
        [+sx, +sy, -sz],
        [-sx, +sy, -sz],
        [-sx, -sy, +sz],
        [+sx, -sy, +sz],
        [+sx, +sy, +sz],
        [-sx, +sy, +sz],
    ])
    if rot is not None:
        corners = corners @ rot.T
    corners += center

    faces = [
        corners[[0, 1, 2, 3]],   # bottom
        corners[[4, 5, 6, 7]],   # top
        corners[[0, 1, 5, 4]],   # front
        corners[[2, 3, 7, 6]],   # back
        corners[[0, 3, 7, 4]],   # left
        corners[[1, 2, 6, 5]],   # right
    ]
    return faces


def _create_cylinder(center: np.ndarray, radius: float, height: float,
                     axis: str = 'z', n_sides: int = _CYLINDER_SIDES) -> list:
    """Return polygon faces for a cylinder (two caps + side quads).

    Parameters
    ----------
    center : centre of the cylinder
    radius : cylinder radius
    height : full height
    axis   : 'z' (vertical) – the cylinder extends from center-h/2 to center+h/2
    n_sides: number of facets (default 8 for performance)

    Returns
    -------
    list of numpy arrays for Poly3DCollection
    """
    angles = np.linspace(0, 2 * np.pi, n_sides, endpoint=False)
    h2 = height / 2.0
    cx, cy, cz = center

    # Circle points at bottom and top
    bottom = np.column_stack([
        cx + radius * np.cos(angles),
        cy + radius * np.sin(angles),
        np.full(n_sides, cz - h2),
    ])
    top = np.column_stack([
        cx + radius * np.cos(angles),
        cy + radius * np.sin(angles),
        np.full(n_sides, cz + h2),
    ])

    faces = []
    # Bottom cap
    faces.append(bottom)
    # Top cap
    faces.append(top)
    # Side quads
    for i in range(n_sides):
        j = (i + 1) % n_sides
        quad = np.array([bottom[i], bottom[j], top[j], top[i]])
        faces.append(quad)

    return faces


def _create_beam(start: np.ndarray, end: np.ndarray,
                 width: float = 3.0, thickness: float = 1.5) -> list:
    """Return 6 quad faces for a rectangular beam between two 3D points.

    The beam cross-section is *width* (horizontal-ish) × *thickness*
    (depth-ish), oriented so the wide face is roughly vertical.

    Parameters
    ----------
    start, end : (3,) arrays
    width, thickness : cross-section dimensions (cm)

    Returns
    -------
    list of 6 numpy arrays, each (4,3)
    """
    direction = end - start
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return []

    rot = _rotation_align(direction)
    center = (start + end) / 2.0
    return _create_box(center, (length, width, thickness), rot)


# ─── Bin box helper ──────────────────────────────────────────────────
def _box_vertices(cx, cy, cz, sx=6, sy=6, sz=6):
    """Return 6 faces (each 4 vertices) for a wireframe box centred at (cx, cy, cz)."""
    return _create_box(np.array([cx, cy, cz]), (sx, sy, sz))


# ─── Visualiser class ────────────────────────────────────────────────
class ArmVisualizer:
    """Live 3-D visualiser for the 4-DOF robotic arm.

    Renders the arm with volumetric shapes matching the physical
    Dynamixel-based robot: cylindrical base, rectangular servo housings,
    flat bracket-style link beams, and a two-jaw gripper.

    Performance: all dynamic arm geometry is batched into a single
    Poly3DCollection and reused across frames via set_verts().
    """

    # Colour palette for bins
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

        # ── Batched arm collection (reused across frames) ─────────
        self._arm_collection: Poly3DCollection | None = None
        self._arm_collections: list[Poly3DCollection] = []  # back-compat
        self._cable_lines = []  # kept empty; cables removed for perf
        self._first_frame = True

        # Claw tip marker
        self.tip_marker, = self.ax.plot(
            [], [], [], "v",
            color=COLORS['claw_tip'], markersize=12, markeredgecolor="#fff",
            markeredgewidth=1.2, zorder=11,
        )

        # Ghost trail (faint previous positions) — reused via set_data_3d
        self._trail_xs: list[float] = []
        self._trail_ys: list[float] = []
        self._trail_zs: list[float] = []
        self.trail_line, = self.ax.plot(
            [], [], [], ".", color=COLORS['trail'], alpha=0.15, markersize=3,
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.show(block=False)
        plt.pause(0.1)

    # ── Styling ──────────────────────────────────────────────────────
    def _style_axes(self):
        ax = self.ax
        ax.set_facecolor("#16213e")

        ax.set_xlim(-25, 60)
        ax.set_ylim(-50, 50)
        ax.set_zlim(-5, 70)

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
        ax.xaxis.pane.set_edgecolor((1, 1, 1, 0.1))
        ax.yaxis.pane.set_edgecolor((1, 1, 1, 0.1))
        ax.zaxis.pane.set_edgecolor((1, 1, 1, 0.1))
        ax.xaxis._axinfo['grid']['color'] = (1, 1, 1, 0.25)
        ax.yaxis._axinfo['grid']['color'] = (1, 1, 1, 0.25)
        ax.zaxis._axinfo['grid']['color'] = (1, 1, 1, 0.25)
        ax.grid(True, alpha=0.3, linewidth=0.5)

        # Camera angle – similar perspective as the reference photo
        ax.view_init(elev=25, azim=-60)

    def _draw_environment(self):
        """Draw bins, ground plane, and the static base cylinder."""
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

        # Base origin marker
        ax.scatter([0], [0], [0], marker="s", s=120,
                   color="#b388ff", edgecolors="#fff", linewidths=1.5, zorder=5)
        ax.text(0, 0, -3, "BASE", color="#b388ff", fontsize=8,
                ha="center", fontweight="bold")

        # ── Static base cylinder (never changes, drawn once) ─────
        servo_bottom_z = SHOULDER_HEIGHT - 1.5 - 3.5 / 2.0
        base_h = servo_bottom_z
        base_center = np.array([0.0, 0.0, base_h / 2.0])
        cyl_faces = _create_cylinder(base_center, radius=4.0, height=base_h,
                                     axis='z', n_sides=_CYLINDER_SIDES)
        poly = Poly3DCollection(cyl_faces, alpha=0.85,
                                facecolor=COLORS['base'],
                                edgecolor='#C0B89A', linewidth=0.4)
        ax.add_collection3d(poly)

    # ── Arm shape builders ───────────────────────────────────────────

    def _build_arm_faces(self, joints, theta_base, m5_steps):
        """Build all faces and per-face colours for the arm at the given pose.

        Returns all geometry as raw face lists + colour lists so they can
        be batched into a **single** Poly3DCollection.

        Parameters
        ----------
        joints : list of 5 tuples (x,y,z)
        theta_base : float
        m5_steps : int

        Returns
        -------
        all_faces : list of ndarray  – polygon vertices
        all_facecolors : list of str – one colour per face
        all_edgecolors : list of str – one edge colour per face
        """
        shoulder_pt = np.array(joints[1])
        elbow_pt    = np.array(joints[2])
        wrist_pt    = np.array(joints[3])
        tip_pt      = np.array(joints[4])

        all_faces = []
        all_facecolors = []
        all_edgecolors = []

        rot_base = _rotation_matrix_z(theta_base)

        # ── 1. Servo at base-shoulder junction ───────────────────
        servo_size = (3.5, 5.0, 3.5)
        servo_base_center = np.array([0.0, 0.0, SHOULDER_HEIGHT - 1.5])
        faces = _create_box(servo_base_center, servo_size, rot_base)
        all_faces.extend(faces)
        all_facecolors.extend([COLORS['servo']] * len(faces))
        all_edgecolors.extend(['#444444'] * len(faces))

        # ── 2. Link 1: Shoulder → Elbow (flat beam) ─────────────
        faces = _create_beam(shoulder_pt, elbow_pt, width=3.0, thickness=1.5)
        if faces:
            all_faces.extend(faces)
            all_facecolors.extend([COLORS['link']] * len(faces))
            all_edgecolors.extend(['#333333'] * len(faces))

        # ── 3. Servo at shoulder ─────────────────────────────────
        faces = _create_box(shoulder_pt, servo_size, rot_base)
        all_faces.extend(faces)
        all_facecolors.extend([COLORS['servo']] * len(faces))
        all_edgecolors.extend(['#444444'] * len(faces))

        # ── 4. Servo at elbow ────────────────────────────────────
        link1_dir = elbow_pt - shoulder_pt
        if np.linalg.norm(link1_dir) > 1e-6:
            elbow_rot = _rotation_align(link1_dir)
        else:
            elbow_rot = rot_base
        faces = _create_box(elbow_pt, servo_size, elbow_rot)
        all_faces.extend(faces)
        all_facecolors.extend([COLORS['servo']] * len(faces))
        all_edgecolors.extend(['#444444'] * len(faces))

        # ── 5. Link 2: Elbow → Wrist (flat beam) ────────────────
        faces = _create_beam(elbow_pt, wrist_pt, width=3.0, thickness=1.5)
        if faces:
            all_faces.extend(faces)
            all_facecolors.extend([COLORS['link']] * len(faces))
            all_edgecolors.extend(['#333333'] * len(faces))

        # ── 6. Servo at wrist ───────────────────────────────────
        link2_dir = wrist_pt - elbow_pt
        if np.linalg.norm(link2_dir) > 1e-6:
            wrist_rot = _rotation_align(link2_dir)
        else:
            wrist_rot = rot_base
        wrist_servo_size = (3.0, 4.0, 3.0)
        faces = _create_box(wrist_pt, wrist_servo_size, wrist_rot)
        all_faces.extend(faces)
        all_facecolors.extend([COLORS['servo']] * len(faces))
        all_edgecolors.extend(['#444444'] * len(faces))

        # ── 7. Link 3: Wrist → Claw tip (thinner beam) ─────────
        faces = _create_beam(wrist_pt, tip_pt, width=2.0, thickness=1.0)
        if faces:
            all_faces.extend(faces)
            all_facecolors.extend([COLORS['link']] * len(faces))
            all_edgecolors.extend(['#333333'] * len(faces))

        # ── 8. Gripper / claw (two jaws) — inlined for batching ─
        claw_faces, claw_fc, claw_ec = self._build_claw_faces(
            wrist_pt, tip_pt, m5_steps)
        all_faces.extend(claw_faces)
        all_facecolors.extend(claw_fc)
        all_edgecolors.extend(claw_ec)

        return all_faces, all_facecolors, all_edgecolors

    # Keep legacy wrapper for API compatibility
    def _build_arm_geometry(self, joints, theta_base, m5_steps):
        """Build all Poly3DCollection objects for the arm at the given pose.

        Parameters
        ----------
        joints : list of 5 tuples (x,y,z)
            [base, shoulder, elbow, wrist, tip]
        theta_base : float
            Base rotation angle (rad) for orienting servos
        m5_steps : int
            Claw motor steps for open/close angle

        Returns
        -------
        collections : list of Poly3DCollection
        cable_lines : list of mpl line objects  (always empty — cables removed)
        """
        all_faces, all_facecolors, all_edgecolors = self._build_arm_faces(
            joints, theta_base, m5_steps)

        coll = Poly3DCollection(all_faces, facecolors=all_facecolors,
                                edgecolors=all_edgecolors,
                                linewidths=0.3, alpha=0.9)
        return [coll], []

    def _build_claw_faces(self, wrist_pt, tip_pt, m5_steps):
        """Build the two-jaw gripper faces (raw lists, not collections).

        Returns
        -------
        faces : list of ndarray
        facecolors : list of str
        edgecolors : list of str
        """
        faces = []
        facecolors = []
        edgecolors = []

        claw_dir = tip_pt - wrist_pt
        claw_len = np.linalg.norm(claw_dir)
        if claw_len < 1e-6:
            return faces, facecolors, edgecolors

        claw_unit = claw_dir / claw_len
        rot = _rotation_align(claw_dir)

        claw_angle_rad = (m5_steps - STEP_CENTRE) * RAD_PER_STEP
        jaw_spread = 1.5 + 2.0 * math.sin(claw_angle_rad)
        jaw_spread = max(0.3, min(4.0, jaw_spread))

        perp = rot[:, 1]
        jaw_length = 3.0
        jaw_width  = 0.5
        jaw_thick  = 1.0

        # Jaw 1
        jaw1_center = tip_pt + perp * (jaw_spread / 2.0) - claw_unit * (jaw_length / 2.0)
        jaw1 = _create_box(jaw1_center, (jaw_length, jaw_width, jaw_thick), rot)
        faces.extend(jaw1)
        facecolors.extend([COLORS['claw']] * len(jaw1))
        edgecolors.extend(['#666666'] * len(jaw1))

        # Jaw 2
        jaw2_center = tip_pt - perp * (jaw_spread / 2.0) - claw_unit * (jaw_length / 2.0)
        jaw2 = _create_box(jaw2_center, (jaw_length, jaw_width, jaw_thick), rot)
        faces.extend(jaw2)
        facecolors.extend([COLORS['claw']] * len(jaw2))
        edgecolors.extend(['#666666'] * len(jaw2))

        return faces, facecolors, edgecolors

    def _build_claw(self, wrist_pt, tip_pt, m5_steps):
        """Build the two-jaw gripper at the end effector.

        Parameters
        ----------
        wrist_pt : (3,) array
        tip_pt   : (3,) array
        m5_steps : int – claw motor steps (2048=centre/open)

        Returns
        -------
        list of Poly3DCollection
        """
        faces, facecolors, edgecolors = self._build_claw_faces(
            wrist_pt, tip_pt, m5_steps)
        if not faces:
            return []
        coll = Poly3DCollection(faces, facecolors=facecolors,
                                edgecolors=edgecolors,
                                linewidths=0.3, alpha=0.9)
        return [coll]

    def _draw_cables(self, shoulder, elbow, wrist, tip, rot_base):
        """Draw thin cable lines along the arm segments.

        Returns list of matplotlib line objects.

        NOTE: Cables are disabled for rendering performance. This method
        is retained for API compatibility but returns an empty list.
        """
        # Cables removed for performance — they created 3 line objects
        # per frame and added significant draw overhead.
        return []

    # ── Remove dynamic arm objects ───────────────────────────────────

    def _clear_arm(self):
        """Remove all arm collections and cable lines from the axes."""
        for coll in self._arm_collections:
            try:
                coll.remove()
            except (ValueError, AttributeError):
                pass
        self._arm_collections.clear()

        if self._arm_collection is not None:
            try:
                self._arm_collection.remove()
            except (ValueError, AttributeError):
                pass
            self._arm_collection = None

        for line in self._cable_lines:
            try:
                line.remove()
            except (ValueError, AttributeError):
                pass
        self._cable_lines.clear()

    # ── Update arm pose ──────────────────────────────────────────────
    def update_plot(self, steps: dict):
        """Redraw the arm at the given motor step positions.

        Parameters
        ----------
        steps : dict
            ``{"m1": int, "m2": int, "m3": int, "m4": int, "m5": int}``
        """
        m5 = steps.get("m5", 2048)
        joints = forward_kinematics(
            steps["m1"], steps["m2"], steps["m3"], steps["m4"], m5)

        theta_base = steps_to_rad(steps["m1"])

        # Build batched face data
        all_faces, all_facecolors, all_edgecolors = self._build_arm_faces(
            joints, theta_base, m5)

        if self._first_frame or self._arm_collection is None:
            # First frame: create a single batched Poly3DCollection
            self._clear_arm()
            coll = Poly3DCollection(all_faces, facecolors=all_facecolors,
                                    edgecolors=all_edgecolors,
                                    linewidths=0.3, alpha=0.9)
            self.ax.add_collection3d(coll)
            self._arm_collection = coll
            self._arm_collections = [coll]
            self._first_frame = False
        else:
            # Subsequent frames: reuse the existing collection via set_verts
            try:
                self._arm_collection.set_verts(all_faces)
                self._arm_collection.set_facecolors(all_facecolors)
                self._arm_collection.set_edgecolors(all_edgecolors)
            except (ValueError, AttributeError):
                # Fallback: recreate if set_verts fails (e.g. face count changed)
                self._clear_arm()
                coll = Poly3DCollection(all_faces, facecolors=all_facecolors,
                                        edgecolors=all_edgecolors,
                                        linewidths=0.3, alpha=0.9)
                self.ax.add_collection3d(coll)
                self._arm_collection = coll
                self._arm_collections = [coll]

        # Update claw tip marker
        tip = joints[-1]
        self.tip_marker.set_data_3d([tip[0]], [tip[1]], [tip[2]])

        # Ghost trail — append and reuse existing line object
        self._trail_xs.append(tip[0])
        self._trail_ys.append(tip[1])
        self._trail_zs.append(tip[2])
        MAX_TRAIL = 300
        if len(self._trail_xs) > MAX_TRAIL:
            self._trail_xs = self._trail_xs[-MAX_TRAIL:]
            self._trail_ys = self._trail_ys[-MAX_TRAIL:]
            self._trail_zs = self._trail_zs[-MAX_TRAIL:]
        self.trail_line.set_data_3d(
            self._trail_xs, self._trail_ys, self._trail_zs)

        # Blocking redraw — draw() ensures the frame is actually rendered,
        # flush_events() processes GUI events, and a 20 ms pause gives the
        # backend enough time to display the frame (~50 fps cap).
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.02)

    # ── Show (alias for close / blocking display) ────────────────────
    def show(self):
        """Leave the plot open at the end so the user can inspect it."""
        plt.ioff()
        logger.info("[VISUALIZER] Simulation complete — close the plot window to exit.")
        plt.show()

    # ── Cleanup ──────────────────────────────────────────────────────
    def close(self):
        """Leave the plot open at the end so the user can inspect it."""
        self.show()


# ── Quick self-test ──────────────────────────────────────────────────
if __name__ == "__main__":
    from ik.solver import ArmIK

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    arm = ArmIK()
    viz = ArmVisualizer()

    import time

    # ── 1. Start with the scan pose (raw motor steps, no IK) ────────
    #   This is the characteristic "arm up, elbow folded back" pose
    #   that parks the wrist camera above the workspace.
    print("=== Scan Pose (raw steps) ===")
    print(f"    {SCAN_POSE}")
    viz.update_plot(SCAN_POSE)
    time.sleep(2.0)

    # ── 2. Walk the arm across IK-generated test positions ──────────
    test_targets = [
        HOME_POSITION,
        (20.0,  5.0,  0.0),
        (25.0, -10.0, 0.0),
        (30.0,  20.0, 12.0),
        (30.0, -20.0, 12.0),
        (15.0,  25.0, 12.0),
        HOME_POSITION,
    ]

    # Helper: smooth interpolation between two step dicts
    INTERP_FRAMES = 30
    INTERP_DELAY  = 1.5   # total seconds per move (matches MockSerial default)

    def _interp_steps(src: dict, dst: dict, frames: int = INTERP_FRAMES,
                      duration: float = INTERP_DELAY):
        """Interpolate from *src* to *dst* over *frames* with ease-in-out."""
        dt = duration / max(frames, 1)
        for i in range(1, frames + 1):
            t = i / frames
            t_smooth = (1 - math.cos(t * math.pi)) / 2.0
            frame = {}
            for key in ("m1", "m2", "m3", "m4", "m5"):
                frame[key] = int(round(src[key] + (dst[key] - src[key]) * t_smooth))
            viz.update_plot(frame)
            time.sleep(dt)

    prev_steps = SCAN_POSE   # starting pose

    for t in test_targets:
        steps = arm.solve(*t)
        print(f"Target {t} → steps {steps}")
        _interp_steps(prev_steps, steps)
        prev_steps = steps

    # ── 4. Return to scan pose to finish ─────────────────────────────
    print("=== Back to Scan Pose ===")
    _interp_steps(prev_steps, SCAN_POSE)

    viz.close()
