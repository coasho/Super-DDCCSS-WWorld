"""Landing gear for the 3D model.

Each leg is a strut with its oleo and wheel(s), built with the wheel on the
ground and the strut up to its attachment, where it hinges: as the gear
retracts the viewer swings it up about its trunnion and, at once, twists it
about the strut (two nested fsim:gear nodes), as real legs fold. A
retractable leg folds into a bay cut into the airframe, and doors - the skin
cut out over the bay - open before it comes down and close after it has gone
up. The opening is as wide as the leg needs to pass: its two doors, hinged
along the edges either side of the leg's swing, hang clear of it all the way
(model3d measures that).

Per gear in the design ([[gear]]):
    retract = "aft"         the way the leg swings up: forward, aft, inward or outward
                            (default: a steerable nose gear aft, main gear forward)
    retract_deg = 90        how far, and
    wheel_turn = 0          degrees the wheel twists about the strut on the way up
                            ("flat": lying flat); hangar fits what is not given - the
                            angle, the twist, and a hinge canted up to 40 deg, as real
                            trunnions are - to stow the leg inside the airframe through
                            as small an opening as it can, as near a plain 90 deg swing
                            as it can
    retract_axis = [x, y, z] the trunnion's axis, when it is known (the leg at the
                            gear's own position; its mirror image turns about the
                            mirror image): a positive turn about it folds the leg
    wheels = 1              side by side on one axle
    axles = 1               a bogie: axles one behind the other on a beam under the
                            strut, axle_spacing (m) apart (a transport's four-wheel
                            truck: wheels = 2, axles = 2); its wheels in pairs, the
                            strut coming down between them
    bogie_deg = 0           degrees a bogie turns on its pivot as the leg folds, nose
                            up positive; "level": the smaller turn that leaves it lying
                            level, stowed; "over": the one that leaves it level on its
                            back (the Tu-16's somersault into a slim gondola)
    door_deg = 90           how far the doors open (default: each until it hangs straight down from its hinge)
    door_reach = 0.8        how far out (m) a door's hinge may move to clear the leg: more
                            for a wide multi-wheel truck (the B-52's)
    doors = false           an open well, without doors: the stowed wheel may stand out
                            of the skin up to its radius (the A-10's main wheels, half
                            out of their pods; the B-52's outriggers in its thin wingtips)
    doors = "leg"           one door carried on the leg: the skin over the opening, fixed to
                            the strut, closing the well flush as the leg stows and hanging
                            from it when it is down - where no pair of hinged doors clears
                            the leg (an inward-swinging bogie's ends, a sponson, a nacelle
                            or a wing root too tight for them), as real strut doors do
    fairing = true          (fixed gear) a spat over the wheel

A single leg (not mirrored) is a centre leg - a fork round its wheel, free to
fold across the centre line - also where it stands beside it (the A-10's and
the Su-25's nose wheels, clear of the gun).
"""
import contextlib

import numpy as np

from .airframe import GAP, METAL, SKIN, STRUT, TYRE, union

DOORS = 0.2  # the doors move while the gear position runs 0..0.2, the legs 0.2..1
DOOR = 0.02  # a door's thickness (m)
CLEAR = 0.04  # the open doors' clearance from the moving leg, and the stowed leg's from the closed doors (m)
GROUND_CLEAR = 0.15  # how far an open door stays above the ground under its leg's wheels (m)
SETTLE = 0.9  # a door that closes back once its leg is down does so while the gear position runs 0.9..1
DOOR_REACH = 0.8  # how far out a door's hinge may move to clear the leg (m; door_reach for a wide truck)


def _rotation(axis, deg):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    th = np.radians(deg)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def _axis_angle(R):
    """The unit axis and angle (degrees) of the rotation R."""
    th = float(np.arccos(np.clip(0.5 * (np.trace(R) - 1.0), -1.0, 1.0)))
    if th < 1e-9:
        return np.array([1.0, 0.0, 0.0]), 0.0
    if np.pi - th < 1e-6:
        w, v = np.linalg.eigh(0.5 * (R + R.T))
        return v[:, int(np.argmax(w))], float(np.degrees(th))
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(th))
    return k / np.linalg.norm(k), float(np.degrees(th))


def _level(R, over=False):
    """The turn (degrees, nose up positive) about a bogie's pivot that leaves
    it lying level once the leg's turn R has stowed it: the smaller one
    (within +-90), or the one that leaves it on its back."""
    b = float(np.degrees(np.arctan2(R[2, 0], R[2, 2])))  # level and upright
    if over:
        return b - 180.0 if b > 0.0 else b + 180.0
    return b - 180.0 if b > 90.0 else b + 180.0 if b <= -90.0 else b


def _children(node):
    return node["children"] if node.get("op") == "union" else [node]


def _swing(direction, side):
    """The axis about which a positive turn swings a hanging leg that way
    (design frame: x aft, y right, z up)."""
    if direction == "forward":
        return np.array([0.0, 1.0, 0.0])
    if direction == "aft":
        return np.array([0.0, -1.0, 0.0])
    s = -1.0 if side > 0 else 1.0   # inward: the right leg about -x
    return np.array([s if direction == "inward" else -s, 0.0, 0.0])


class Leg:
    """One leg: where it hinges, the turn that stows it, and its parts."""

    def __init__(self, gear, name, contact, side):
        spec = gear.spec
        self.name, self.side = name, side
        self.retractable = gear.retractable
        self.steerable = gear.steerable
        self.fairing = gear.fairing
        self.direction = spec.get("retract", "aft" if gear.steerable else "forward")
        if self.direction not in ("forward", "aft", "inward", "outward"):
            raise ValueError("gear %r: retract must be forward, aft, inward or outward" % gear.name)
        self.spec = spec
        self.wheels = int(spec.get("wheels", 1))
        self.door_deg = float(spec["door_deg"]) if "door_deg" in spec else None  # (None: each door hangs down open)
        self.door_reach = float(spec.get("door_reach", DOOR_REACH))
        doors = spec.get("doors", True)
        if doors not in (True, False, "leg"):
            raise ValueError("gear %r: doors is true, false or \"leg\"" % gear.name)
        self.leg_door = doors == "leg"  # the well closed by a door on the leg (a well without hinged doors)
        self.doors = doors is True
        self.r = 0.5 * gear.wheel_diameter
        self.w = gear.wheel_width
        self.axle = contact + np.array([0.0, 0.0, self.r])
        self.axles = int(spec.get("axles", 1))
        self.axle_spacing = float(spec.get("axle_spacing", 2.5 * self.r))
        if self.axles < 1 or (self.axles > 1 and (self.wheels < 2 or self.axle_spacing < 2.0 * self.r + 0.05)):
            raise ValueError("gear %r: a bogie (axles > 1) carries its wheels in pairs (wheels >= 2), "
                             "its axles more than a wheel's diameter apart" % gear.name)
        bogie = spec.get("bogie_deg", 0.0)
        if isinstance(bogie, str) and bogie not in ("level", "over"):
            raise ValueError("gear %r: bogie_deg is degrees, \"level\" or \"over\"" % gear.name)
        if bogie != 0.0 and self.axles < 2:
            raise ValueError("gear %r: bogie_deg turns a bogie (axles > 1)" % gear.name)
        self.bogie_level = bogie if bogie in ("level", "over") else None
        self.bogie_deg = 0.0 if self.bogie_level else float(bogie)
        top = gear.attach.copy() if gear.attach is not None else contact + np.array([0.0, 0.0, 2.0 * self.r + 0.8])
        if side < 0 < top[1] or side > 0 > top[1]:
            top[1] = -top[1]
        self.hinge = top
        self.strut_r = 0.035 + 0.06 * gear.wheel_diameter
        # the stowing turn: the swing about the trunnion and the twist about
        # the strut, together
        self.strut = (self.hinge - self.axle) / np.linalg.norm(self.hinge - self.axle)
        axis = _swing(self.direction, side)
        if "retract_axis" in spec:
            axis = np.asarray(spec["retract_axis"], float)
            if axis.shape != (3,) or np.linalg.norm(axis) < 1e-9:
                raise ValueError("gear %r: retract_axis = [x, y, z], not zero" % gear.name)
            if side < 0 < gear.position[1] or side > 0 > gear.position[1]:
                axis = -np.diag([1.0, -1.0, 1.0]) @ axis  # the mirror image's
        angle = float(spec.get("retract_deg", 90.0))
        turn = spec.get("wheel_turn", 0.0)
        if turn == "flat":
            # the twist that leaves the stowed axle nearest vertical
            y = np.array([0.0, 1.0, 0.0])
            swing = _rotation(axis, angle)
            turn = float(max(range(-90, 91), key=lambda a: (abs((swing @ _rotation(self.strut, a) @ y)[2]), -abs(a))))
        self.set_stow(axis, angle, float(turn) * (1.0 if side >= 0 else -1.0))  # the mirror image twists the other way
        self.fitted = False

    def set_stow(self, axis, angle, twist):
        """The swing (unit axis through the hinge, degrees) and the twist
        about the strut (degrees) that stow the leg."""
        self.swing_axis = np.asarray(axis, float) / np.linalg.norm(axis)
        self.swing_deg, self.twist_deg = float(angle), float(twist)
        self.R = _rotation(self.swing_axis, self.swing_deg) @ _rotation(self.strut, self.twist_deg)
        if self.bogie_level:
            self.bogie_deg = _level(self.R, self.bogie_level == "over")

    @property
    def bogie_turns(self):
        """Whether the bogie turns on its pivot as the leg folds."""
        return self.bogie_level is not None or self.bogie_deg != 0.0

    def bogie_turn(self, frac=1.0):
        """The bogie's own turn frac of the way up, about the axles' direction
        through its pivot (the foot)."""
        return _rotation([0.0, 1.0, 0.0], frac * self.bogie_deg)

    def _moved(self, pts, on, axis, angle, twist, frac):
        """Points pts (on: those on a turning bogie) with the leg turned frac of
        the way through the swing about axis and the twist (a candidate's)."""
        q = pts
        if self.bogie_turns:
            beta = (_level(_rotation(axis, angle) @ _rotation(self.strut, twist), self.bogie_level == "over")
                    if self.bogie_level else self.bogie_deg)
            foot = self.foot()
            q = pts.copy()
            q[on] = foot + (pts[on] - foot) @ _rotation([0.0, 1.0, 0.0], frac * beta).T
        return self.hinge + (q - self.hinge) @ (_rotation(axis, frac * angle) @ _rotation(self.strut, frac * twist)).T

    def fit(self, probe):
        """Fits the stowing turn to the airframe (a meshkit.Probe): the swing's
        angle, its hinge axis canted - leaned about z, tilted towards z - and
        the wheel's twist: one that stows the leg inside, clear of the closed
        doors, and of those the one that needs the smallest opening on its
        way, as plain as it can. A main leg stays on its own side of the
        centre line all the way. Returns how far the stowed leg stands out of
        the skin (m)."""
        pts, on = self.points()
        given = [k for k in ("retract_axis", "retract_deg", "wheel_turn") if k in self.spec]
        if len(given) == 3 or (len(given) == 2 and "retract_axis" not in given):
            return float(max(probe(self.turned(pts, bogie=on))[0].max(), 0.0))
        base = self.swing_axis if "retract_axis" in self.spec else _swing(self.direction, self.side)
        sign = 1.0 if self.side >= 0 else -1.0
        tilt_axis = np.cross(base, [0.0, 0.0, 1.0])  # turns the hinge axis towards z
        cants = [(0.0, 0.0)] if "retract_axis" in self.spec else \
            [(lean, tilt) for lean in (0.0, -15.0, 15.0, -30.0, 30.0) for tilt in (0.0, 20.0, -20.0, 40.0, -40.0)]
        angles = [float(self.spec["retract_deg"])] if "retract_deg" in self.spec else [90.0, 80.0, 100.0, 110.0, 70.0, 120.0]
        turn = self.spec.get("wheel_turn")
        cands = []
        for lean, tilt in cants:
            axis = _rotation([0.0, 0.0, 1.0], lean * sign) @ _rotation(tilt_axis, tilt) @ base
            for angle in angles:
                swing = _rotation(axis, angle)
                if turn == "flat":  # the twist that leaves the stowed axle nearest vertical
                    y = np.array([0.0, 1.0, 0.0])
                    twists = [float(max(range(-90, 91), key=lambda a: (abs((swing @ _rotation(self.strut, a) @ y)[2]), -abs(a))))]
                    signed = True
                elif turn is not None:
                    twists, signed = [float(turn) * sign], True
                else:
                    twists, signed = [0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 75.0, -75.0, 90.0, -90.0], False
                for twist in twists:
                    t = twist if signed else twist * sign
                    cands.append(((abs(lean) + abs(tilt)) / 30.0 + abs(angle - 90.0) / 30.0 + abs(twist) / 90.0,
                                  (axis, angle, t)))
        stowed = np.array([self._moved(pts, on, a, g, t, 1.0) for _, (a, g, t) in cands])
        d, _ = probe(stowed.reshape(-1, 3))
        # inside, clear of the closed doors
        out = np.maximum(d.reshape(len(cands), -1).max(axis=1) + DOOR + 0.5 * CLEAR, 0.0)
        # on its way: a quarter, half and three quarters up
        mids = np.array([[self._moved(pts, on, a, g, t, f) for f in (0.25, 0.5, 0.75)]
                         for _, (a, g, t) in cands]).reshape(len(cands), -1, 3)
        dm, _ = probe(mids.reshape(-1, 3))
        dm = dm.reshape(len(cands), -1)
        if self.side != 0:  # not across the centre line, into the other leg's bay
            y = np.concatenate([stowed[:, :, 1], mids[:, :, 1]], axis=1)
            out += np.maximum(0.05 - self.side * y, 0.0).max(axis=1)
        # the opening it needs: the leg's extent where it passes the skin or
        # hangs below it, near it (m2)
        z_skin = skin_below(probe, self.hinge[0], self.hinge[1], self.hinge[2] + 0.3)
        z_skin = self.axle[2] + self.r if z_skin is None else z_skin
        near = (dm > -0.05) & (mids[:, :, 2] > z_skin - 0.4)
        big = 1e9
        ext = [np.where(near, mids[:, :, i], -big).max(axis=1) - np.where(near, mids[:, :, i], big).min(axis=1)
               for i in (0, 1)]
        area = np.where(near.any(axis=1), np.clip(ext[0], 0.0, None) * np.clip(ext[1], 0.0, None), 0.0)
        # inside (to a centimetre) first, then the smallest opening and the plainest turn
        score = 100.0 * np.maximum(out - 0.01, 0.0) + area / 0.25 + np.array([p for p, _ in cands])
        k = int(np.argmin(score))
        self.set_stow(*cands[k][1])
        self.fitted = True
        return float(out[k])

    def mirrored(self, other):
        """Takes the mirror image of other's stowing turn (the pair's other
        leg): M R(a, t) M = R(-M a, t) for the reflection M."""
        M = np.diag([1.0, -1.0, 1.0])
        self.set_stow(-M @ other.swing_axis, other.swing_deg, -other.twist_deg)
        self.fitted = other.fitted

    def axle_points(self):
        """The axles' centres: the leg's own, or a bogie's one behind the other."""
        n = self.axles
        return [self.axle + np.array([(k - 0.5 * (n - 1)) * self.axle_spacing, 0.0, 0.0]) for k in range(n)]

    def wheel_centres(self, axle=None):
        """The wheels' centres, on every axle or on one, and how wide they span."""
        n, w = self.wheels, self.w
        gap = 0.04 if self.axles == 1 else 1.6 * self.strut_r + 0.06  # a bogie's strut comes down between
        span = n * w + (n - 1) * gap
        on = self.axle_points() if axle is None else [self.axle_points()[axle]]
        return [a + np.array([0.0, -0.5 * span + 0.5 * w + k * (w + gap), 0.0]) for a in on for k in range(n)], span

    def wheel_parts(self):
        """The names of the leg's wheel groups (part_groups): one per axle,
        each rolling about its own."""
        return ["wheel"] + ["wheel %d" % (k + 1) for k in range(1, self.axles)]

    def parts(self):
        """The leg's solids, all in one (part_groups)."""
        return union([n for g in self.part_groups().values() for n in _children(g)])

    def fixed_parts(self):
        """The solids that turn with the leg alone: all of them, or all but a
        turning bogie's."""
        if not self.bogie_turns:
            return self.parts()
        g = self.part_groups()
        return union([n for k in ("strut", "oleo") for n in _children(g[k])])

    def bogie_parts(self):
        """A turning bogie's solids: its beam and wheels."""
        g = self.part_groups()
        return union([n for k in ["bogie"] + self.wheel_parts() for n in _children(g[k])])

    def part_groups(self):
        """The leg's solids in the three pieces that move apart: the strut
        (the cylinder, down from the hinge), the oleo (its piston, and a fork
        - a nose wheel, a pair - or a stub axle - a single main wheel, from
        inboard - with a fixed gear's spat), which slides up the strut as it
        compresses and turns with the steering, and the wheel(s) on their
        axle, which roll."""
        r, w, rs = self.r, self.w, self.strut_r
        y = np.array([0.0, 1.0, 0.0])
        if self.axles > 1:
            return self._bogie_groups()
        centres, span = self.wheel_centres()
        wheel, oleo = [], []
        for c in centres:
            wheel.append({"prim": "cylinder", "material": TYRE, "a": c - 0.5 * w * y, "b": c + 0.5 * w * y, "r": r,
                          "round": min(0.45 * w, 0.3 * r)})
            wheel.append({"prim": "cylinder", "material": METAL, "a": c - (0.5 * w + 0.01) * y,
                          "b": c + (0.5 * w + 0.01) * y, "r": 0.55 * r, "round": 0.008})
        foot = self.foot()
        if self.side == 0 or self.wheels > 1:
            half = 0.5 * span + 0.7 * rs
            for s in (-1.0, 1.0):
                oleo.append({"prim": "capsule", "material": STRUT, "a": foot + s * half * y, "b": self.axle + s * half * y,
                             "r": 0.45 * rs})
            oleo.append({"prim": "capsule", "material": STRUT, "a": foot - half * y, "b": foot + half * y, "r": 0.5 * rs})
            # the axle through the fork, turning with the wheels it joins
            wheel.append({"prim": "cylinder", "material": METAL, "a": self.axle - half * y, "b": self.axle + half * y,
                          "r": 0.3 * rs, "round": 0.004})
        else:
            oleo.append({"prim": "cylinder", "material": METAL, "a": self.axle, "b": foot, "r": 0.4 * rs, "round": 0.004})
        mid = foot + 0.45 * (self.hinge - foot)
        oleo.append({"prim": "capsule", "material": METAL, "a": foot, "b": mid, "r": 0.72 * rs})  # the piston
        if self.fairing and not self.retractable:
            oleo.append(self._spat(span))
        return {"strut": union([{"prim": "capsule", "material": STRUT, "a": mid, "b": self.hinge, "r": rs}]),
                "oleo": union(oleo), "wheel": union(wheel)}

    def _bogie_groups(self):
        """A bogie's solids: the strut; the oleo - its piston down between the
        wheels to the beam, the beam along under it from the first axle to the
        last (a part of its own, the bogie, where it turns on its pivot); and
        each axle's wheels, with the axle through them."""
        r, w, rs = self.r, self.w, self.strut_r
        y = np.array([0.0, 1.0, 0.0])
        foot = self.foot()
        axles = self.axle_points()
        lift = foot - self.axle
        groups = {}
        for name, (k, a) in zip(self.wheel_parts(), enumerate(axles)):
            centres, span = self.wheel_centres(k)
            parts = []
            for c in centres:
                parts.append({"prim": "cylinder", "material": TYRE, "a": c - 0.5 * w * y, "b": c + 0.5 * w * y, "r": r,
                              "round": min(0.45 * w, 0.3 * r)})
                parts.append({"prim": "cylinder", "material": METAL, "a": c - (0.5 * w + 0.01) * y,
                              "b": c + (0.5 * w + 0.01) * y, "r": 0.55 * r, "round": 0.008})
            parts.append({"prim": "cylinder", "material": METAL, "a": a - 0.5 * span * y, "b": a + 0.5 * span * y,
                          "r": 0.3 * rs, "round": 0.004})
            groups[name] = union(parts)
        mid = foot + 0.45 * (self.hinge - foot)
        piston = {"prim": "capsule", "material": METAL, "a": foot, "b": mid, "r": 0.72 * rs}
        beam = [{"prim": "capsule", "material": STRUT, "a": axles[0] + lift, "b": axles[-1] + lift, "r": 0.55 * rs}]
        for a in axles:  # the beam's ends round the axles
            beam.append({"prim": "capsule", "material": STRUT, "a": a, "b": a + lift, "r": 0.5 * rs})
        strut = union([{"prim": "capsule", "material": STRUT, "a": mid, "b": self.hinge, "r": rs}])
        if self.bogie_turns:
            return dict({"strut": strut, "oleo": union([piston]), "bogie": union(beam)}, **groups)
        return dict({"strut": strut, "oleo": union([beam[0], piston] + beam[1:])}, **groups)

    def foot(self):
        """The bottom of the oleo: above a fork, inboard of a single wheel,
        or on a bogie's beam."""
        rs, y = self.strut_r, np.array([0.0, 1.0, 0.0])
        if self.axles > 1:
            return self.axle + np.array([0.0, 0.0, 0.6 * rs])
        if self.side == 0 or self.wheels > 1:
            return self.axle + np.array([0.0, 0.0, self.r + 0.08 + 0.6 * rs])
        _, span = self.wheel_centres()
        return self.axle - np.sign(self.side) * (0.5 * span + 1.3 * rs) * y

    def slide(self):
        """The way the oleo slides as it compresses (up the strut), and how far
        along it per metre JSBSim compresses the unit (vertically)."""
        k = self.hinge - self.foot()
        k = k / np.linalg.norm(k)
        return k, 1.0 / max(abs(float(k[2])), 0.2)

    def _spat(self, span):
        """A wheel spat: a streamlined shell over the wheel, open underneath."""
        r = self.r
        c = self.axle + np.array([0.1 * r, 0.0, 0.1 * r])
        shell = {"prim": "ellipsoid", "material": SKIN, "centre": c, "radii": [1.75 * r, 0.5 * span + 0.06, 1.05 * r]}
        return {"op": "subtract", "k": 0.0, "a": shell, "cut_material": SKIN,
                "b": {"prim": "box", "material": SKIN, "centre": self.axle - np.array([0.0, 0.0, 1.35 * r]),
                      "half": [3.0 * r, 0.5 * span + 0.2, 0.6 * r]}}

    def points(self):
        """Points on the leg as built - its tyres' rims and its strut - and
        which of them turn with a bogie."""
        th = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        ring = np.stack([np.cos(th), np.zeros_like(th), np.sin(th)], axis=1) * self.r
        centres, _ = self.wheel_centres()
        pts = [c + np.array([0.0, s * 0.5 * self.w, 0.0]) + ring for c in centres for s in (-1.0, 1.0)]
        rims = sum(len(p) for p in pts)
        pts.append(np.array([self.hinge + t * (self.axle - self.hinge) for t in np.linspace(0.0, 1.0, 12)]))
        pts = np.vstack(pts)
        return pts, (np.arange(len(pts)) < rims) & self.bogie_turns

    def surface(self, step=0.03):
        """Points all over the leg's skin (its parts' surfaces, about step
        apart) - what must stay clear of the doors - and which of them turn
        with a bogie."""
        if not self.bogie_turns:
            pts = _surface_points(self.parts(), step)
            return pts, np.zeros(len(pts), bool)
        fixed, bogie = _surface_points(self.fixed_parts(), step), _surface_points(self.bogie_parts(), step)
        return np.vstack([fixed, bogie]), np.arange(len(fixed) + len(bogie)) >= len(fixed)

    def turned(self, p, frac=1.0, bogie=None):
        """Points p with the leg turned frac of the way up (bogie: which of
        them turn with a bogie, on its pivot as well)."""
        p = np.asarray(p, float)
        if bogie is not None and self.bogie_turns:
            foot = self.foot()
            p = p.copy()
            p[bogie] = foot + (p[bogie] - foot) @ self.bogie_turn(frac).T
        R = _rotation(self.swing_axis, frac * self.swing_deg) @ _rotation(self.strut, frac * self.twist_deg)
        return self.hinge + (p - self.hinge) @ R.T

    def unturned(self, q, frac, bogie=False):
        """Points q in the frame of the leg as built, the leg frac of the way
        up - in its turning bogie's, if bogie: what turned() undoes."""
        R = _rotation(self.swing_axis, frac * self.swing_deg) @ _rotation(self.strut, frac * self.twist_deg)
        p = self.hinge + (np.asarray(q, float) - self.hinge) @ R
        if bogie:
            foot = self.foot()
            p = foot + (p - foot) @ self.bogie_turn(frac)
        return p

    @contextlib.contextmanager
    def probes(self):
        """Probes of the leg's solids as built, each with whether it is its
        turning bogie's: [(probe, bogie)]."""
        from . import meshkit
        with contextlib.ExitStack() as stack:
            out = [(stack.enter_context(meshkit.Probe({"root": self.fixed_parts()})), False)]
            if self.bogie_turns:
                out.append((stack.enter_context(meshkit.Probe({"root": self.bogie_parts()})), True))
            yield out

    def distance(self, probes, q, frac):
        """How far points q (design frame) stay from the leg frac of the way
        up (m; negative: inside it)."""
        return np.min([p(self.unturned(q, frac, bogie))[0] for p, bogie in probes], axis=0)


def _surface_points(node, step):
    """Points on the surfaces of a union of capsules and cylinders (a leg's
    parts), about step apart; other shapes are left out."""
    if node.get("op") == "union":
        return np.vstack([_surface_points(c, step) for c in node["children"]] or [np.zeros((0, 3))])
    if node.get("prim") not in ("capsule", "cylinder"):
        return np.zeros((0, 3))
    a, b, r = np.asarray(node["a"], float), np.asarray(node["b"], float), float(node["r"])
    k = b - a
    L = float(np.linalg.norm(k))
    k = k / L if L > 1e-9 else np.array([0.0, 0.0, 1.0])
    e1 = np.cross(k, [1.0, 0.0, 0.0] if abs(k[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(k, e1)
    th = np.linspace(0.0, 2.0 * np.pi, max(8, int(np.ceil(2.0 * np.pi * r / step))), endpoint=False)
    ring = np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2)
    pts = [a + t * k + r * ring for t in np.linspace(0.0, L, max(2, int(np.ceil(L / step)) + 1))]
    for end, s in ((a, -1.0), (b, 1.0)):  # the ends: a cap, or a flat face
        for f in np.linspace(0.0, 1.0, max(2, int(np.ceil(r / step)) + 1))[:-1]:
            if node["prim"] == "capsule":
                pts.append(end + s * r * np.sqrt(1.0 - f * f) * k + r * f * ring)
            else:
                pts.append(end + r * f * ring)
    return np.vstack(pts)


def legs(aircraft):
    out = []
    for g in aircraft.gear:
        for name, pos in g.positions():
            side = 0 if abs(pos[1]) < 1e-6 or not g.mirror else (1 if pos[1] > 0 else -1)
            out.append(Leg(g, name, np.asarray(pos, float), side))
    return out


def leg_scene(leg, part=None):
    """The mesher's scene of the leg, or of one of its part_groups."""
    cell = float(np.clip(leg.r / 40.0, 0.002, 0.01))
    return {"cell": cell, "error": 0.1 * cell, "safety": 2.0, "sharp_deg": 45.0, "max_triangles": 4000 if part is None else 2000,
            "root": leg.parts() if part is None else leg.part_groups()[part]}


def skin_below(probe, x, y, z_top, depth=4.0):
    """The height where a vertical line down through (x, y) from z_top last
    leaves the solid (a meshkit.Probe) - the lowest skin there - or None when
    it misses."""
    h = skin_heights(probe, [x], [y], z_top, depth)[0, 0]
    return None if np.isnan(h) else float(h)


def skin_heights(probe, xs, ys, z_top, depth=4.0, step=0.01):
    """The lowest skin's height over the grid xs x ys ([iy, ix]; nan where a
    vertical line down from z_top misses the solid)."""
    zs = np.arange(z_top, z_top - depth, -step)
    X, Y = np.meshgrid(np.asarray(xs, float), np.asarray(ys, float))
    n = X.size
    d, _ = probe(np.stack([np.repeat(X.ravel(), len(zs)), np.repeat(Y.ravel(), len(zs)), np.tile(zs, n)], axis=1))
    d = d.reshape(n, len(zs))
    leave = (d[:, :-1] < 0.0) & (d[:, 1:] >= 0.0)  # out of the solid, going down
    out = np.full(n, np.nan)
    for k in np.flatnonzero(leave.any(axis=1)):
        i = np.flatnonzero(leave[k])[-1]
        out[k] = zs[i] + (zs[i + 1] - zs[i]) * d[k, i] / (d[k, i] - d[k, i + 1])
    return out.reshape(X.shape)


def swept(leg, pts, steps=30, bogie=None):
    """Points pts of the leg (bogie: those on a turning bogie) at every step
    of its swing, down to stowed."""
    return np.vstack([leg.turned(pts, f, bogie) for f in np.linspace(0.0, 1.0, steps + 1)])


def bays(aircraft, solid, foils):
    """Per retractable leg: the bay it folds into (a cut for the airframe),
    the doors over it, and how far the stowed leg would stand out of the skin
    (m; the airframe's own, before the bay is cut)."""
    from . import meshkit
    todo = [leg for leg in legs(aircraft) if leg.retractable]
    if not todo:
        return []
    with meshkit.Probe({"foils": foils, "root": solid}) as probe:
        fitted = {}
        for leg in sorted(todo, key=lambda l: l.side < 0):
            twin = fitted.get(leg.name.replace("Left ", "Right ", 1))
            if leg.side < 0 and twin is not None:
                leg.mirrored(twin)
            else:
                leg.fit(probe)
                fitted[leg.name] = leg
        return [_bay(leg, probe, solid, foils) for leg in todo]


def _bay(leg, probe, solid, foils):
    """One leg's bay, doors and protrusion (bays)."""
    pts, on = leg.surface()
    stowed = leg.turned(pts, bogie=on)
    d, _ = probe(stowed)
    protrusion = float(max(d.max(), 0.0))
    # the leg on its way down: where it is inside the airframe (the bay's
    # space) and where it passes through the skin (the opening)
    moving = swept(leg, pts, bogie=on)
    dm, _ = probe(moving)
    inside = moving[dm < 0.0] if (dm < 0.0).any() else stowed
    through = moving[np.abs(dm) < 0.05] if (np.abs(dm) < 0.05).any() else stowed
    lo0, hi0 = through[:, :2].min(axis=0) - CLEAR, through[:, :2].max(axis=0) + CLEAR
    # the lowest skin around it, 2.5 cm apart, with room for the doors to move out either way
    z_top = float(inside[:, 2].max()) + 0.3
    # (on one lattice for every bay, even about the centre line: a bay and its mirror image get mirror-image doors)
    def lattice(a, b):
        return 0.025 * np.arange(np.floor(a / 0.025), np.ceil(b / 0.025) + 1.0)
    xs = lattice(lo0[0] - leg.door_reach, hi0[0] + leg.door_reach)
    ys = lattice(lo0[1] - leg.door_reach, hi0[1] + leg.door_reach)
    H = skin_heights(probe, xs, ys, z_top)
    X, Y = np.meshgrid(xs, ys)
    # the doors: a pair either side of the middle of the opening, each hinged on its outer edge and hanging down
    # open, each edge moved out until its open door clears the leg all the way. Hinged along x (a pair side by side)
    # or along y (one ahead of the other): the one whose open doors stay out of the airframe and clear the leg - the
    # swing's own first (a leg swinging fore and aft between doors hinged along x, a sideways one along y)
    base = _swing(leg.direction, leg.side)
    first = (1, 0) if abs(base[1]) >= abs(base[0]) else (0, 1)
    layouts = [first, first[::-1]] if leg.doors else [first]
    with leg.probes() as legp:
        tried = [_door_layout(leg, legp, probe, H, [X, Y], iu, iv, lo0.copy(), hi0.copy()) for iu, iv in layouts]
        if leg.doors and leg.door_deg is None:  # and the swing's own, opened 90 deg (a door that hangs opens little,
            # and grows to clear the leg - across a seam in the skin, perhaps)
            tried.append(_door_layout(leg, legp, probe, H, [X, Y], first[0], first[1], lo0.copy(), hi0.copy(), fixed=90.0))
        # the swing's own unless another is clearly better - and one whose doors, meshed, are not closed, whole solids
        # clear of the leg gives way to the next
        order = sorted(range(len(tried)), key=lambda k: (tried[k]["score"] - (0.02 if k == 0 else 0.0), k))
        out = None
        for k in order:
            got = _bay_with(leg, tried[k], X, Y, H, inside, through, solid, foils, protrusion)
            for d in got["doors"]:
                _gooseneck(probe, d)
                _settle(leg, legp, probe, d)
            if out is None:
                out = got
            if all(_door_ok(leg, legp, d) for d in got["doors"]):
                return got
    return out


def _door_ok(leg, legp, door, steps=24):
    """Whether a door meshes as one closed solid, and stays clear of the leg open, all the way as it swings."""
    from . import meshkit
    m = meshkit.build(door["scene"])
    if m["boundary_edges"] + m["nonmanifold_edges"] + m["misoriented_edges"] or m["components"] != 1:
        return False
    P = m["positions"][:: max(1, len(m["positions"]) // 400)]
    R = _rotation(door["axis"], door["deg"])
    opened = door["hinge"] + (P - door["hinge"]) @ R.T
    return all(leg.distance(legp, opened, f).min() >= 0.0 for f in np.linspace(0.0, 1.0, steps + 1))


def _bay_with(leg, best, X, Y, H, inside, through, solid, foils, protrusion):
    """The bay, its doors and its cut for one layout of the doors (_door_layout)."""
    doors_at, lo, hi = best["doors"], best["lo"], best["hi"]
    within = (X >= lo[0]) & (X <= hi[0]) & (Y >= lo[1]) & (Y <= hi[1]) & ~np.isnan(H)
    z_lo = float(H[within].min()) if within.any() else float(through[:, 2].min())
    top = float(inside[:, 2].max()) + 0.06
    # each door is the skin over its pane, from its lowest point up to its
    # highest - its hinge, on a side that curves up - and no skin above that
    panes = []
    for door in doors_at:
        if door is None:
            continue
        r = door["rect"]
        cell = (X >= r[0]) & (X <= r[2]) & (Y >= r[1]) & (Y <= r[3]) & ~np.isnan(H)
        z_top = max(float(door["hinge"][2]), float(H[cell].max()) if cell.any() else z_lo) + 0.05
        panes.append((door, z_top))
    if not leg.doors:
        # an open well: the opening is just where the leg passes the skin
        cell = within
        panes.append(({"rect": np.array([lo[0], lo[1], hi[0], hi[1]])},
                      (float(H[cell].max()) if cell.any() else z_lo) + 0.05))
    # the bay: the opening through the skin under the doors, and above it the
    # space the leg passes through and stows in - hollowed only where the
    # airframe is at least 3 cm thick, so it never breaks through another skin
    ilo = np.minimum(inside[:, :2].min(axis=0) - CLEAR, lo)
    ihi = np.maximum(inside[:, :2].max(axis=0) + CLEAR, hi)
    opening = union([{"prim": "box", "material": SKIN,
                      "centre": [0.5 * (d["rect"][0] + d["rect"][2]), 0.5 * (d["rect"][1] + d["rect"][3]),
                                 0.5 * (z_lo - 0.1 + zt + 0.03)],
                      "half": [0.5 * (d["rect"][2] - d["rect"][0]) + 0.5 * GAP, 0.5 * (d["rect"][3] - d["rect"][1]) + 0.5 * GAP,
                               0.5 * (zt + 0.03 - z_lo + 0.1)], "round": 0.02} for d, zt in panes])
    space = {"op": "intersect", "k": 0.0, "children": [
        {"prim": "box", "material": SKIN, "centre": [0.5 * (ilo[0] + ihi[0]), 0.5 * (ilo[1] + ihi[1]), 0.5 * (z_lo - 0.1 + top)],
         "half": [0.5 * (ihi[0] - ilo[0]) + 0.5 * GAP, 0.5 * (ihi[1] - ilo[1]) + 0.5 * GAP, 0.5 * (top - z_lo + 0.1)],
         "round": 0.03 + 0.5 * GAP},
        {"op": "offset", "r": -0.03, "child": solid}]}
    cut = {"op": "union", "k": 0.0, "children": [opening, space]}
    doors = [_door(solid, foils, d["rect"][:2], d["rect"][2:], z_lo - 0.1, zt, d["hinge"], d["axis"], d["deg"],
                   "%s door %d" % (leg.name, k + 1)) for k, (d, zt) in enumerate(panes)] if leg.doors else []
    for door, (d, _) in zip(doors, panes):
        door["buried"] = d.get("buried", 0.0)
    # a door on the leg: the skin over the well's opening, as it lies with the leg stowed (model3d carries it into
    # the leg's own frame, so it closes the well as the leg comes up)
    cover = None
    if leg.leg_door:
        d, zt = panes[0]
        cover = _door(solid, foils, d["rect"][:2], d["rect"][2:], z_lo - 0.1, zt, leg.hinge, leg.strut, 0.0, "%s door" % leg.name)
    # how far out the stowed leg may stand: an open well's wheel up to its radius
    return {"leg": leg, "cut": cut, "doors": doors, "cover": cover, "protrusion": protrusion,
            "allowed": 0.0 if leg.doors else leg.r}


def _door_layout(leg, legp, probe, H, UV, iu, iv, lo, hi, fixed=None):
    """The pair of doors across u (iu: 0 x, 1 y), hinged along v, either side of the opening's middle: each edge moved
    out until its door, hanging open, clears the leg. Its score: the most of an open door inside the airframe (a share
    of its points), and 1 for each door that has no skin or cannot clear the leg."""
    um = 0.5 * (lo[iu] + hi[iu])
    edges = [lo[iu], hi[iu]]
    limit = leg.door_reach + 0.5 * (hi[iu] - lo[iu])
    doors, fails, buried = [], 0, 0.0
    for k, sgn in ((0, -1.0), (1, 1.0)) if leg.doors else ():
        while True:
            door = _door_frame(H, UV, iu, iv, lo, hi, um, edges[k], sgn)
            if door is not None:
                door["deg"] = fixed if fixed is not None else _hang(door, leg)
            if door is None or _door_clear(leg, legp, door) >= CLEAR or abs(edges[k] - um) > limit:
                break
            edges[k] += sgn * 0.025
        if door is None:
            fails += 1
        else:
            door["buried"] = _door_buried(probe, door)
            if door["buried"] > 0.02 and leg.door_deg is None and fixed is None:
                # (hanging, part of it lies inside the airframe - a fuselage widening along the hinge, a door on a
                # steep side: the angle, 20 to 150 deg, that buries least and still clears the leg, nearest hanging)
                hang = door["deg"]
                tries = [(round(door["buried"], 2), 0.0, hang)]
                for deg in range(20, 151, 5):
                    door["deg"] = float(deg)
                    b = round(_door_buried(probe, door), 2)
                    if b < tries[0][0] and _door_clear(leg, legp, door) >= CLEAR:
                        tries.append((b, abs(deg - hang), float(deg)))
                b, _, door["deg"] = min(tries)
                door["buried"] = _door_buried(probe, door)
            fails += _door_clear(leg, legp, door) < CLEAR
            buried = max(buried, door["buried"])
        doors.append(door)
    lo[iu], hi[iu] = edges
    return {"doors": doors, "lo": lo, "hi": hi, "score": fails + buried}


def _hang(door, leg):
    """How far a door turns open (deg): the design's door_deg, or until it hangs straight down from its hinge - a
    flat door 90 deg, one curving round a fuselage's side less (turned further it would stand out sideways)."""
    if leg.door_deg is not None:
        return leg.door_deg
    a = door["axis"] / np.linalg.norm(door["axis"])
    P = door["points"][: len(door["points"]) // 2]  # (the skin's side)
    rel = P - door["hinge"]
    rel -= np.outer(rel @ a, a)
    far = np.linalg.norm(rel, axis=1)
    c = rel[far >= 0.9 * far.max()].mean(axis=0)
    t = np.array([0.0, 0.0, -1.0]) - a[2] * a
    c, t = c / np.linalg.norm(c), t / np.linalg.norm(t)
    return float(np.clip(np.degrees(np.arctan2(np.dot(a, np.cross(c, t)), np.dot(c, t))), 30.0, 120.0))


def _settle(leg, legp, probe, door):
    """How far a door stays open once its leg is down (door["rest"], deg): as far as it opened to let the leg by -
    or, a low airframe's that would reach the ground under the wheels so, turned back (closing, as a C-17's do, or
    on past hanging) to the nearest angle that stays clear of the ground and of the leg standing down. Left as it
    opened where none does."""
    from . import meshkit
    P = meshkit.build(door["scene"])["positions"]
    P = P[:: max(1, len(P) // 4000)]
    hinge = np.asarray(door["hinge"], float)
    ground = leg.axle[2] - leg.r

    def opened(deg):
        return hinge + (P - hinge) @ _rotation(door["axis"], deg).T

    door["rest"] = door["deg"]
    if opened(door["deg"])[:, 2].min() - ground >= GROUND_CLEAR:
        return
    tries = sorted(set(np.arange(0.0, 175.1, 5.0)) | {door["deg"]}, key=lambda g: abs(g - door["deg"]))
    for deg in tries[1:]:
        o = opened(deg)
        if o[:, 2].min() - ground < GROUND_CLEAR:
            continue
        if min(p(leg.unturned(o, 0.0, bogie))[0].min() for p, bogie in legp) < CLEAR:
            continue
        # (and not folded into the airframe: past hanging it turns up beside it)
        a = door["axis"] / np.linalg.norm(door["axis"])
        rel = P - hinge
        far = np.linalg.norm(rel - np.outer(rel @ a, a), axis=1) > 0.08
        if np.mean((probe(o)[0] < -(DOOR + 0.01)) & far) <= 0.02:
            door["rest"] = float(deg)
            return


def _mesh_buried(probe, door, P):
    """The share of a meshed door's points (P) inside the airframe open, beyond the strip along its hinge (there its
    own thickness dips into the skin beside it as it turns)."""
    a = door["axis"] / np.linalg.norm(door["axis"])
    rel = P - door["hinge"]
    far = np.linalg.norm(rel - np.outer(rel @ a, a), axis=1) > 0.08
    d, _ = probe(door["hinge"] + rel @ _rotation(door["axis"], door["deg"]).T)
    return float(np.mean((d < -(DOOR + 0.01)) & far))


def _gooseneck(probe, door):
    """A door wrapping round a curved side - a nacelle's, a fuselage's - that buries the strip along its hinge as it
    turns, hinged out from the skin as a real one is on gooseneck arms, as little as lets it swing clear (by its
    mesh: the skin's heights straight down miss a steep side)."""
    from . import meshkit
    P = meshkit.build(door["scene"])["positions"]
    P = P[:: max(1, len(P) // 800)]
    buried = _mesh_buried(probe, door, P)
    if buried <= 0.02:
        return
    out, base = _outward(probe, door), np.asarray(door["hinge"], float).copy()
    best = (buried, 0.0)
    for off in np.arange(0.03, 0.301, 0.03):
        door["hinge"] = base + off * out
        b = _mesh_buried(probe, door, P)
        if b < best[0] - 0.005:
            best = (b, off)
        if b <= 0.02:
            break
    # (only where it does swing clear: arms that leave it buried are no better than none)
    door["hinge"] = base + (best[1] if best[0] <= 0.03 else 0.0) * out


def _outward(probe, door):
    """Out of the airframe at a door's hinge, square to its axis: where gooseneck arms set the hinge."""
    h, a = door["hinge"], door["axis"] / np.linalg.norm(door["axis"])
    e = 0.02
    d, _ = probe(np.array([h + [e, 0, 0], h - [e, 0, 0], h + [0, e, 0], h - [0, e, 0], h + [0, 0, e], h - [0, 0, e]]))
    g = np.array([d[0] - d[1], d[2] - d[3], d[4] - d[5]])
    g -= np.dot(g, a) * a
    n = np.linalg.norm(g)
    return g / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])


def _door_buried(probe, door):
    """The share of an open door's skin inside the airframe (deeper than its own thickness and a little)."""
    P = door["points"][: len(door["points"]) // 2]
    R = _rotation(door["axis"], door["deg"])
    d, _ = probe(door["hinge"] + (P - door["hinge"]) @ R.T)
    return float(np.mean(d < -(DOOR + 0.01)))


def _door(solid, foils, lo, hi, z_lo, z_hi, hinge, axis, deg, label):
    """A door: the skin inside the box (lo..hi in x-y, z_lo..z_hi), shrunk by
    half the gap, as a shell DOOR thick."""
    c = np.array([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (z_lo + z_hi)])
    half = np.array([0.5 * (hi[0] - lo[0]) - 0.5 * GAP, 0.5 * (hi[1] - lo[1]) - 0.5 * GAP, 0.5 * (z_hi - z_lo)])
    box = {"prim": "box", "material": SKIN, "centre": c, "half": half, "round": 0.02}
    shell = {"op": "subtract", "k": 0.0, "cut_material": SKIN,
             "a": {"op": "intersect", "k": 0.0, "children": [solid, box]},
             "b": {"op": "offset", "r": -DOOR, "child": solid}}
    return {"scene": {"cell": 0.005, "error": 0.002, "safety": 3.0, "sharp_deg": 40.0, "max_triangles": 1500,
                      "foils": foils, "root": shell},
            "hinge": np.asarray(hinge, float), "axis": np.asarray(axis, float), "deg": deg, "label": label}


def _door_frame(H, UV, iu, iv, lo, hi, um, edge, sgn):
    """A door from um out to edge (sgn: the side, -1 towards lower u): its
    rect, hinge (on the skin at the edge's middle), axis and points on it -
    or None where there is no skin."""
    U, V = UV[iu], UV[iv]
    a, b = (edge, um) if sgn < 0 else (um, edge)
    inpane = (U >= a) & (U <= b) & (V >= lo[iv]) & (V <= hi[iv]) & ~np.isnan(H)
    if not inpane.any():
        return None
    rect = np.zeros(4)
    rect[iu], rect[2 + iu], rect[iv], rect[2 + iv] = a, b, lo[iv], hi[iv]
    # the hinge: along the door's own outer edge on the skin - the outermost skin across each row of it - the line
    # that fits it best: on a belly that slopes, or a nose that tapers and curves, the edge does not run level along v,
    # and a level hinge would turn the door partly in its own plane, or about one corner of it
    edge = []
    for v in np.unique(V[inpane]):
        cand = np.flatnonzero((inpane & (V == v)).ravel())
        k = cand[np.argmax(sgn * U.ravel()[cand])]
        edge.append([UV[0].ravel()[k], UV[1].ravel()[k], H.ravel()[k]])
    edge = np.asarray(edge)
    hinge = edge.mean(axis=0)
    # the turn that takes the door's free edge (towards um) down
    axis = np.zeros(3)
    axis[iv] = sgn if iu == 1 else -sgn
    if len(edge) >= 3:
        along = np.linalg.svd(edge - hinge)[2][0]
        if np.dot(along, axis) < 0.0:
            along = -along
        # (no more than 35 deg off level along v: a few cells of skin are not an edge)
        if np.dot(along, axis) > np.cos(np.radians(35.0)):
            axis = along / np.linalg.norm(along)
    P = np.stack([UV[0][inpane], UV[1][inpane], H[inpane]], axis=1)
    P = np.vstack([P, P + [0.0, 0.0, DOOR]])
    return {"rect": rect, "hinge": hinge, "axis": axis, "points": P}


def _door_clear(leg, probes, door, steps=24):
    """How far the door, open, stays from the leg as it swings (m; probes:
    the leg's, Leg.probes)."""
    R = _rotation(door["axis"], door["deg"])
    opened = door["hinge"] + (door["points"] - door["hinge"]) @ R.T
    fs = np.linspace(0.0, 1.0, steps + 1)
    return float(min(p(np.vstack([leg.unturned(opened, f, bogie) for f in fs]))[0].min() for p, bogie in probes))
