"""Authored store geometry access; aerodynamic forces use the aircraft pipeline."""
import math
import os
import tomllib
from . import pipeline
THICKNESS={"biconvex4":.04,"plate":.03}

def design_of(card_id, *, with_path=False):
    """The design whose card this is (its toml), or None."""
    root = os.path.join(pipeline.repo_root(), "aircraft")
    # (a store's design is filed under its card's id without its hyphens - looked at first; any other is searched for)
    first = card_id.replace("-", "")
    for name in [first] + sorted(n for n in os.listdir(root) if n != first and n.casefold() != ".codex"):
        path = os.path.join(root, name, name + ".toml")
        if os.path.isfile(path):
            with open(path, "rb") as f:
                d = tomllib.load(f)
            if d.get("aircraft", {}).get("card") == card_id:
                return (d, path) if with_path else d
    return (None, None) if with_path else None

def geometry(card):
    """Its shape: {x, r (m, from its nose), fins, cg, length, diameter, from}; None without one."""
    d, path = design_of(card["id"], with_path=True)
    if d and d.get("store", {}).get("shape") == "box":
        return None  # (a cargo store: no body of revolution)
    if d:
        body = next(b for b in d["body"] if b.get("kind") == "fuselage")
        st = body["stations"]
        x = [s["x"] - st[0]["x"] for s in st]
        r = [0.25 * (s["w"] + s["top"] - s["bottom"]) for s in st]
        fins = []
        for srf in d.get("surface", []):
            secs = srf["sections"]
            a, b = secs[0], secs[-1]
            dy, dz = b["le"][1] - a["le"][1], b["le"][2] - a["le"][2]
            s = math.hypot(dy, dz)
            if s <= 0.0:
                continue
            fins.append(dict(x=a["le"][0] - st[0]["x"], cr=a["chord"], ct=b["chord"], xt=b["le"][0] - a["le"][0], s=s,
                             r=math.hypot(a["le"][1], a["le"][2]), cant2=dz * dz / (s * s), n=2 if srf.get("mirror") else 1,
                             tc=THICKNESS.get(srf.get("airfoil", "plate"), 0.03), lattice="lattice" in srf))
        return dict(x=x, r=r, fins=fins, cg=d["mass"]["empty_cg"][0] - st[0]["x"], length=x[-1], diameter=2.0 * max(r),
                    design=d, design_path=path, source="its design")
    raise ValueError("Store requires an authored aircraft VLM design: "+card["id"])

def coefficients(geo, card_id=None):
    from . import airframe
    return airframe.coefficients(geo['design_path'])
