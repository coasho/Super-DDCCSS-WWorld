"""Lattice (grid) fins (ADR-30, MS-7e1): a surface given `lattice = { width = m, cells = n }` is drawn as a grid fin - a
box frame as long as the surface's span and as deep as its chord along the flow, `width` across it, round a diamond
lattice of thin walls at 45 degrees, `cells` across - its open cells facing the flow, as an R-77's tail; in place of the
plate the surface's sections would give (shape/airframe.py: in the airframe and in its control's piece, so an
all-moving lattice fin turns about its spindle like any other). The walls are drawn thicker than a real one's (a few
millimetres, the mesh's cells) and the frame squared off; its aerodynamics are the surface's own (an estimate)."""
import numpy as np

FRAME = 0.006  # m: the frame's walls
WALL = 0.0045  # m: the lattice's


def _box(centre, half, axes, material, mirror):
    return {"prim": "box", "material": material, "mirror": bool(mirror), "centre": [float(c) for c in centre],
            "half": [float(h) for h in half], "axes": [[float(x) for x in a] for a in axes]}


def lattice(surface, material, mirror=None):
    """The grid fin a lattice surface is drawn as: its frame and walls, in the design frame (the defined half, mirrored
    as the surface is unless `mirror` says)."""
    spec = surface.lattice
    mirror = surface.mirror if mirror is None else mirror
    root, tip = surface.sections[0], surface.sections[-1]
    c, u = surface.frame(0.5)
    d = np.asarray(tip.le, float) - np.asarray(root.le, float)
    s = d - np.dot(d, c) * c
    length = float(np.linalg.norm(s))
    s /= length
    depth = 0.5 * (root.chord + tip.chord)
    width = float(spec["width"])
    centre = np.asarray(root.le, float) + 0.5 * depth * c + 0.5 * length * s
    outer = _box(centre, [0.5 * depth, 0.5 * length, 0.5 * width], [c, s, u], material, mirror)
    hole = _box(centre, [0.5 * depth + 0.01, 0.5 * length - FRAME, 0.5 * width - FRAME], [c, s, u], material, mirror)
    frame = {"op": "subtract", "k": 0.0, "a": outer, "b": hole}
    # the diamond lattice: two families of walls at +-45 deg across the frame's face, `cells` of them across its width
    pitch = width / max(int(spec.get("cells", 3)), 1) * np.sqrt(0.5)
    reach = 0.5 * (length + width)
    walls = []
    for along, normal in (((s + u) * np.sqrt(0.5), (s - u) * np.sqrt(0.5)), ((s - u) * np.sqrt(0.5), (s + u) * np.sqrt(0.5))):
        n = int(np.ceil(reach / pitch)) + 1
        for k in range(-n, n + 1):
            walls.append(_box(centre + k * pitch * normal, [0.5 * depth, reach, 0.5 * WALL], [c, along, normal], material, mirror))
    grid = {"op": "intersect", "k": 0.0,
            "children": [{"op": "union", "k": 0.0, "children": walls},
                         _box(centre, [0.5 * depth, 0.5 * length - 0.5 * FRAME, 0.5 * width - 0.5 * FRAME], [c, s, u], material, mirror)]}
    return {"op": "union", "k": 0.0, "children": [frame, grid]}
