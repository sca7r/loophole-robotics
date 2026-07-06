"""Composable scene framework — the stage where robots act.

This module separates the *stage* (floor, tables, objects, lighting, cameras)
from the *players* (robots). A scene knows nothing about which robot will be
dropped into it; a robot knows nothing about which scene it will be mounted in.

The contract:

    scene = Scene()
    scene.add_table(size=(0.35, 0.45), height=0.10, pos=(0.0, 0.0))
    scene.add_table(size=(0.30, 0.30), height=0.10, pos=(0.55, 0.0))
    scene.add_object("cube", size=0.025, pos=(0.18, 0.08, 0.13), color="orange")

    # robots are added separately, via build() — see workcell.build()

You can change the scene (add more tables, move objects) without touching any
robot config. You can swap robots without touching the scene. This is exactly
the playground-vs-players model: MuJoCo is the playground, scene + robots
compose into a simulation.

All dimensions are SI: metres, kilograms, radians.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import mujoco

# ── Named colors for convenience ────────────────────────────────────────
# Users can also pass an explicit (r, g, b) or (r, g, b, a) tuple.
_NAMED_COLORS: dict[str, tuple[float, float, float, float]] = {
    "orange":  (0.95, 0.45, 0.10, 1.0),
    "red":     (0.85, 0.15, 0.15, 1.0),
    "blue":    (0.20, 0.40, 0.85, 1.0),
    "green":   (0.20, 0.70, 0.30, 1.0),
    "yellow":  (0.95, 0.85, 0.15, 1.0),
    "white":   (0.93, 0.94, 0.96, 1.0),
    "black":   (0.15, 0.15, 0.17, 1.0),
    "grey":    (0.55, 0.57, 0.60, 1.0),
    "purple":  (0.55, 0.30, 0.75, 1.0),
}

ObjectKind = Literal["cube", "sphere", "cylinder"]


def _resolve_color(color: str | tuple) -> tuple[float, float, float, float]:
    """Look up a named color or pass through an explicit tuple."""
    if isinstance(color, str):
        if color not in _NAMED_COLORS:
            raise ValueError(
                f"unknown color {color!r}; known: {sorted(_NAMED_COLORS)}"
                " — or pass an (r, g, b) or (r, g, b, a) tuple"
            )
        return _NAMED_COLORS[color]
    if len(color) == 3:
        return (color[0], color[1], color[2], 1.0)
    if len(color) == 4:
        return tuple(color)  # type: ignore[return-value]
    raise ValueError(f"color must be a name or a 3/4-tuple; got {color!r}")


# ── Scene primitives ────────────────────────────────────────────────────
@dataclass
class TableSpec:
    """A workbench in the scene.

    ``size`` is the (x, y) half-extents in metres. ``height`` is the top
    surface height above the floor. ``pos`` is the (x, y) centre of the
    table on the floor.
    """
    size: tuple[float, float]
    height: float
    pos: tuple[float, float] = (0.0, 0.0)
    name: str = ""


@dataclass
class ObjectSpec:
    """A pickable, physically-simulated object in the scene.

    The object is a free body (six-DOF floating joint) so it falls under
    gravity and can be grasped, pushed, knocked over. Mass and friction get
    physically-reasonable defaults so picks work without tuning.

    Parameters
    ----------
    kind:
        ``"cube"`` (a box), ``"sphere"``, or ``"cylinder"``.
    size:
        For a cube: edge length (metres). For a sphere: radius. For a
        cylinder: ``(radius, half_height)`` tuple.
    pos:
        World-frame (x, y, z) starting position in metres.
    color:
        Named color (e.g. ``"orange"``) or explicit ``(r, g, b)`` /
        ``(r, g, b, a)`` tuple.
    mass:
        Kilograms. Default 0.05 kg — light enough for a small gripper.
    name:
        Optional unique name; auto-generated if blank.
    """
    kind: ObjectKind
    size: float | tuple[float, float]
    pos: tuple[float, float, float]
    color: str | tuple = "orange"
    mass: float = 0.05
    name: str = ""


@dataclass
class Scene:
    """The composable stage. Robots are added separately, via build()."""

    tables: list[TableSpec] = field(default_factory=list)
    objects: list[ObjectSpec] = field(default_factory=list)
    floor: bool = True
    lighting: Literal["studio", "flat"] = "studio"
    reference_axes: bool = False
    reference_axes_origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    table_grid: bool = False     # cm grid overlay on each table top (teaching aid)

    # ── Mutators (chainable) ────────────────────────────────────────────
    def add_table(
        self,
        size: tuple[float, float] = (0.35, 0.45),
        height: float = 0.10,
        pos: tuple[float, float] = (0.0, 0.0),
        name: str = "",
    ) -> Scene:
        """Add a workbench. Half-extents in metres; height above floor."""
        self.tables.append(TableSpec(size=size, height=height, pos=pos, name=name))
        return self

    def add_object(
        self,
        kind: ObjectKind = "cube",
        size: float | tuple[float, float] = 0.025,
        pos: tuple[float, float, float] = (0.20, 0.0, 0.15),
        color: str | tuple = "orange",
        mass: float = 0.05,
        name: str = "",
    ) -> Scene:
        """Add a pickable, physically-simulated object."""
        self.objects.append(ObjectSpec(
            kind=kind, size=size, pos=pos, color=color, mass=mass, name=name,
        ))
        return self


# ── MJCF spec construction ───────────────────────────────────────────────
def add_scene_to_spec(spec: mujoco.MjSpec, scene: Scene) -> None:
    """Inject the scene's tables, objects, lighting, and floor into a spec.

    Materials needed by the scene (floor, table, frame) are assumed already
    declared on the spec by ``_apply_industrial_styling``.
    """
    world = spec.worldbody

    # Lighting
    if scene.lighting == "studio":
        _add_studio_lighting(world)
    else:
        _add_flat_lighting(world)

    # Floor
    if scene.floor:
        world.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[0, 0, 0.05],
            material="floor_mat",
        )

    # Tables
    for i, t in enumerate(scene.tables):
        name = t.name or f"table_{i}"
        _add_table(world, t, name)

    # Pickable objects
    for i, obj in enumerate(scene.objects):
        name = obj.name or f"object_{i}"
        _add_object(world, obj, name)

    # Reference axes (visual-only teaching aid).
    if scene.reference_axes:
        add_reference_axes(spec, origin=scene.reference_axes_origin)

    # Per-table grid overlay (visual-only teaching aid).
    if scene.table_grid:
        for i, t in enumerate(scene.tables):
            _add_table_grid(world, t, name=t.name or f"table_{i}")


def _add_studio_lighting(world) -> None:
    """Three-point key/fill/rim, the cleanest neutral look."""
    world.add_light(
        pos=[0.6, -0.6, 1.4], dir=[-0.4, 0.4, -1.0],
        diffuse=[0.7, 0.7, 0.7], specular=[0.3, 0.3, 0.3], castshadow=True,
    )
    world.add_light(
        pos=[-0.6, -0.4, 1.2], dir=[0.4, 0.3, -1.0],
        diffuse=[0.3, 0.3, 0.35], specular=[0.1, 0.1, 0.1], castshadow=False,
    )
    world.add_light(
        pos=[0.0, 0.8, 1.0], dir=[0.0, -0.6, -1.0],
        diffuse=[0.25, 0.25, 0.28], castshadow=False,
    )


def _add_flat_lighting(world) -> None:
    """Single bright overhead, no shadows — for diagnostic renders."""
    world.add_light(
        pos=[0.0, 0.0, 2.0], dir=[0.0, 0.0, -1.0],
        diffuse=[0.9, 0.9, 0.9], castshadow=False,
    )


def _add_table(world, t: TableSpec, name: str) -> None:
    """Steel top on a dark frame, sized by the spec."""
    top_thickness = 0.015
    leg = 0.025
    sx, sy = t.size
    h = t.height

    body = world.add_body(name=name, pos=[t.pos[0], t.pos[1], 0.0])
    body.add_geom(
        name=f"{name}_top",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0, 0, h - top_thickness],
        size=[sx, sy, top_thickness],
        material="steel_mat",
    )
    for i, (lx, ly) in enumerate([
        (sx - leg,    sy - leg),   (sx - leg,    -(sy - leg)),
        (-(sx - leg), sy - leg),   (-(sx - leg), -(sy - leg)),
    ]):
        body.add_geom(
            name=f"{name}_leg_{i}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[lx, ly, (h - top_thickness) / 2],
            size=[leg, leg, (h - top_thickness) / 2],
            material="frame_mat",
        )


def _add_object(world, obj: ObjectSpec, name: str) -> None:
    """Pickable free body — falls under gravity, can be grasped."""
    rgba = list(_resolve_color(obj.color))
    body = world.add_body(name=name, pos=list(obj.pos))
    # Free joint = 6-DOF floating, so the object is fully simulated.
    body.add_freejoint()

    if obj.kind == "cube":
        s = float(obj.size) if not isinstance(obj.size, tuple) else obj.size[0]
        half = s / 2
        body.add_geom(
            name=f"{name}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[half, half, half],
            rgba=rgba,
            mass=obj.mass,
            friction=[1.0, 0.05, 0.0001],   # grip-friendly
        )
    elif obj.kind == "sphere":
        r = float(obj.size) if not isinstance(obj.size, tuple) else obj.size[0]
        body.add_geom(
            name=f"{name}_geom",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[r, 0, 0],
            rgba=rgba,
            mass=obj.mass,
            friction=[1.0, 0.05, 0.0001],
        )
    elif obj.kind == "cylinder":
        if not isinstance(obj.size, tuple):
            raise ValueError("cylinder size must be (radius, half_height)")
        r, hh = obj.size
        body.add_geom(
            name=f"{name}_geom",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[r, hh, 0],
            rgba=rgba,
            mass=obj.mass,
            friction=[1.0, 0.05, 0.0001],
        )
    else:
        raise ValueError(f"unknown object kind {obj.kind!r}")


def add_reference_axes(
    spec: mujoco.MjSpec,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    length: float = 0.10,
    radius: float = 0.004,
) -> None:
    """Add an RGB coordinate triad at ``origin`` for the pitch and for teaching.

    Red = +X (forward from arm base), Green = +Y (left), Blue = +Z (up).
    The arrows are small visual-only geoms (no collision, no contact) so they
    do not affect physics — they are a teaching aid, not a part of the world.

    Place this at the arm's mount position to anchor the operator's frame.
    """
    world = spec.worldbody
    length_ = length
    radius_ = radius
    radius_ = radius

    # All three axes share these properties (no contact, no shadows, visual only).
    common = dict(contype=0, conaffinity=0, group=2)

    # +X axis (red) — cylinder along the body x-axis, centred at L/2.
    world.add_geom(
        name="ref_axis_x",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[radius_, length_ / 2, 0],
        pos=[origin[0] + length_ / 2, origin[1], origin[2]],
        quat=[0.7071068, 0.0, 0.7071068, 0.0],  # rotate cyl from z to x
        rgba=[1.0, 0.20, 0.20, 1.0],
        **common,
    )
    # +Y axis (green)
    world.add_geom(
        name="ref_axis_y",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[radius_, length_ / 2, 0],
        pos=[origin[0], origin[1] + length_ / 2, origin[2]],
        quat=[0.7071068, -0.7071068, 0.0, 0.0],  # rotate cyl from z to y
        rgba=[0.20, 0.85, 0.20, 1.0],
        **common,
    )
    # +Z axis (blue) — default cylinder is along z, so no rotation needed.
    world.add_geom(
        name="ref_axis_z",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[radius_, length_ / 2, 0],
        pos=[origin[0], origin[1], origin[2] + length_ / 2],
        rgba=[0.30, 0.50, 1.0, 1.0],
        **common,
    )


def _add_table_grid(world, t: TableSpec, name: str, spacing: float = 0.05) -> None:
    """Add a thin grid overlay on a table top — teaching aid for picking points.

    Lines are visual-only (no collision) and float a hair above the steel
    surface to avoid z-fighting. Spacing defaults to 5 cm, which gives the
    operator clear reference marks without visual clutter.
    """
    sx, sy = t.size
    h = t.height
    z = h + 0.0005   # 0.5 mm above the surface, no z-fight
    line_thickness = 0.0008
    line_rgba = [0.30, 0.32, 0.35, 0.55]   # dark grey, semi-transparent
    common = dict(contype=0, conaffinity=0, group=2)

    # Lines parallel to Y (running along the y direction at each x increment).
    import math
    nx = math.floor(sx / spacing)
    for k in range(-nx, nx + 1):
        x = k * spacing
        if abs(x) > sx:
            continue
        world.add_geom(
            name=f"{name}_grid_x{k}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[line_thickness, sy, 0.0002],
            pos=[t.pos[0] + x, t.pos[1], z],
            rgba=line_rgba,
            **common,
        )
    # Lines parallel to X.
    ny = math.floor(sy / spacing)
    for k in range(-ny, ny + 1):
        y = k * spacing
        if abs(y) > sy:
            continue
        world.add_geom(
            name=f"{name}_grid_y{k}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[sx, line_thickness, 0.0002],
            pos=[t.pos[0], t.pos[1] + y, z],
            rgba=line_rgba,
            **common,
        )


def default_pickplace_scene() -> Scene:
    """The canonical single-arm teaching scene: one workbench, two cubes,
    reference axes and a grid. Used by the in-process teach CLI; the server
    builds its own N-arm variant of the same recipe (see server/cli.py).
    """
    return (
        Scene(
            reference_axes=True,
            reference_axes_origin=(0.0, 0.0, 0.10),
            table_grid=True,
        )
        .add_table(size=(0.35, 0.45), height=0.10, pos=(0.0, 0.0))
        .add_object("cube", size=0.025, pos=(0.18, 0.08, 0.13), color="orange",
                    name="cube_orange")
        .add_object("cube", size=0.025, pos=(0.18, -0.08, 0.13), color="blue",
                    name="cube_blue")
    )
