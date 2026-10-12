"""What each model gives the viewer's effects to work from, against what its design has (docs/sdk/viewer.md, Effects):

    python -m hangar effects [--record docs/hangar-effects.md]

The viewer draws nothing of its own invention: a control surface moves because the model has a joint for it
(`fsim:<channel>` nodes) and the simulation says how far; a plume, a contrail or a smoke trail starts where the model or
its manifest says an engine's exhaust leaves it (`fsim:afterburner` nodes, the manifest's `exhaust` lines) with what
the manifest says of the engine (`engine`, `rocket`) and what the simulation says it burns; wing vapour and vortices
from the manifest's `wing`, `canard`, `strake` lines. This lists, design by design, what is there - and what a design
has that its model gives the viewer nothing for (`missing`), which a test holds empty (tests/test_effects.py).
"""
import collections
import json
import os
import struct
import tomllib

from . import pipeline

#: an engine that burns fuel leaves an exhaust: the viewer needs to know where
BURNS = ("turbofan", "turbojet", "turboprop", "turboshaft", "piston")


def _nodes(glb):
    with open(glb, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 12)[0]
    kinds = collections.Counter()
    for node in json.loads(data[20:20 + n]).get("nodes", []):
        name = node.get("name", "")
        if name.startswith("fsim:"):
            kinds[name.split(":")[1].split("@")[0]] += 1
    return kinds


def _manifest(path):
    lines = collections.Counter()
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.strip() and not line.startswith("#"):
                    lines[line.split()[0]] += 1
    return lines


def audit():
    """Each design with a model: {name, category, engines {type: count}, motor, nodes, manifest, has, missing}."""
    root = os.path.join(pipeline.repo_root(), "aircraft")
    rows = []
    for name in sorted(os.listdir(root)):
        toml, glb = os.path.join(root, name, name + ".toml"), os.path.join(root, name, name + ".glb")
        if not (os.path.isfile(toml) and os.path.isfile(glb)):
            continue
        with open(toml, "rb") as f:
            d = tomllib.load(f)
        nodes, manifest = _nodes(glb), _manifest(glb + ".manifest")
        engines = d.get("engine", [])
        engines = [engines] if isinstance(engines, dict) else [e for e in engines if isinstance(e, dict)]
        types = collections.Counter()
        for e in engines:
            types[e.get("type", "piston")] += 2 if e.get("mirror") else 1
        category = d.get("aircraft", {}).get("category", "")
        channels = sorted(k for k in nodes if k not in ("afterburner", "station", "section", "cover", "nozzle", "wheel", "oleo", "steer", "bay",
                                                        "gear", "propeller", "blades", "disc", "rolleron", "rolleron-disc", "freeroll"))
        has, missing = [], []
        burning = sum(c for t, c in types.items() if t in BURNS)
        anchors = nodes.get("afterburner", 0) + manifest.get("exhaust", 0)
        rotor = category in ("helicopter", "multirotor")
        if "motor" in d:
            (has if nodes.get("afterburner") and manifest.get("rocket") else missing).append("a rocket's plume and smoke trail (its nozzle, its `rocket` line)")
        if burning:
            if anchors >= burning:
                has.append("exhaust: %d nozzle(s)%s" % (nodes.get("afterburner", 0), ", %d placed by its manifest" % manifest["exhaust"] if manifest.get("exhaust") else ""))
            elif rotor:
                has.append("no exhaust anchor (a rotorcraft's own pipeline: docs/backlog.md, B-59)")
            else:
                missing.append("an exhaust for each of its %d engine(s): %d on its model or its manifest" % (burning, anchors))
        if types.get("turbofan") and not rotor and not manifest.get("engine"):
            missing.append("its jets' kinds and ratings (the manifest's `engine` lines)")
        spinning = nodes.get("propeller", 0)
        if (types.get("turboprop", 0) + types.get("piston", 0) + types.get("electric", 0)) and not spinning:
            missing.append("a propeller that turns")
        if spinning:
            has.append("%d propeller(s) or rotor(s) turning at their engines' rpm" % spinning)
        if channels:
            has.append("moving surfaces: " + ", ".join(channels))
        if nodes.get("gear"):
            has.append("gear")
        if nodes.get("bay"):
            has.append("weapon-bay doors")
        turned = [k for k in ("rolleron", "freeroll") if nodes.get(k)]
        if turned:
            has.append("turned by the airstream: " + ", ".join(turned))
        if manifest.get("wing") or manifest.get("canard") or manifest.get("strake"):
            has.append("wing vapour and vortices")
        elif category not in ("missile", "store") and not rotor:
            missing.append("its lifting surfaces for the vapour and the vortices (the manifest's `wing` line)")
        if category == "store" and not has:
            has.append("a rigid body: nothing on it moves or burns")
        rows.append(dict(name=name, category=category, engines=dict(types), motor="motor" in d, nodes=dict(nodes), manifest=dict(manifest),
                         has=has, missing=missing))
    return rows


def text(rows):
    lines = ["# What each model gives the viewer's effects", "",
             "Written by `python -m hangar effects --record docs/hangar-effects.md` from the designs, their models' nodes and",
             "their manifests ([viewer.md](sdk/viewer.md#effects)): what the viewer has to draw each vehicle's and each payload's",
             "moving parts, plumes and trails from. The simulation supplies the state; this audit checks model anchors and",
             "manifest data, not rendered appearance or flight behavior. `tools/hangar/tests/test_effects.py` checks the",
             "current rules' `missing` lists.", "",
             "**Current audit exclusions.** Rotorcraft without exhaust anchors are reported in `what it gives`, not counted",
             "as `missing`; rotorcraft also bypass the lifting-surface requirement. Rotorcraft exhaust, rotor wake and tip",
             "vortices remain [B-59](backlog.md). Propeller heat haze and tip vortices, and a released store's continuing",
             "wreck fire, are also outside these inventory checks. A zero `missing` count means zero failures under these",
             "rules, not complete physical-effects coverage or a fresh visual acceptance run.", "",
             "| design | category | engines | what it gives | missing |", "| --- | --- | --- | --- | --- |"]
    for r in rows:
        engines = ", ".join("%s x%d" % kv for kv in sorted(r["engines"].items())) + (" rocket motor" if r["motor"] else "")
        lines.append("| `%s` | %s | %s | %s | %s |" % (r["name"], r["category"], engines or "-", "; ".join(r["has"]) or "-", "; ".join(r["missing"]) or "-"))
    lines += ["", "%d designs; %d with something missing." % (len(rows), sum(1 for r in rows if r["missing"])), ""]
    return "\n".join(lines)


def main(argv):
    rows = audit()
    for r in rows:
        if r["missing"]:
            print("%-16s missing: %s" % (r["name"], "; ".join(r["missing"])))
    print("%d designs, %d with something missing" % (len(rows), sum(1 for r in rows if r["missing"])))
    if "--record" in argv:
        target = argv[argv.index("--record") + 1]
        path = target if os.path.isabs(target) else os.path.join(pipeline.repo_root(), target)
        pipeline.write_text(path, text(rows))
        print("recorded: %s" % pipeline.shown(path))
    return 1 if any(r["missing"] for r in rows) else 0
