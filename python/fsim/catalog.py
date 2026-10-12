"""What the platform carries, read without a world: the aircraft it flies, each one's payload fit and loadouts, the
payload and store cards, and each drawn model with its manifest (docs/sdk/python.md, "The catalog").

    import fsim.catalog
    fsim.catalog.aircraft()                    # ['a10c', 'b52h', ..., 'f16c', ...]: every type with a fit
    fsim.catalog.fit("f16c")["payloads"]       # its payloads: name, card, count
    fsim.catalog.loadouts("f16c")              # [Loadout(name='air superiority', note=..., stores=((1, 'aim-120c'), ...)), ...]
    fsim.catalog.store("aim-120c")             # a store's card: designation, class, parameters with their sources
    fsim.catalog.payload("an-apg-68v9")        # a payload's card
    fsim.catalog.model("f16c")                 # the path of its drawn model (glTF binary), or None
    fsim.catalog.manifest("f16c")              # the model's manifest: its axes, stations, engines, components

It reads the package's own data (fsim/share/flightsim: what a World loads its vehicles from), so what it says is
what a World of this package does. Nothing is written; each call gives a fresh object the caller may keep.
"""
import collections
import json
import os

__all__ = ["Loadout", "Manifest", "aircraft", "fit", "loadouts", "manifest", "model", "payload", "payloads", "share_dir", "store",
           "stores"]

_SHARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "share", "flightsim")

Loadout = collections.namedtuple("Loadout", "name note stores")
Loadout.__doc__ = """A load an aircraft's fit names for its stations (``Vehicle.load_loadout(name)``): ``stores`` is
(station, store id) for each station it fills, the store a card's id (``fsim.catalog.store``)."""

Manifest = collections.namedtuple("Manifest", "forward up scale engines rockets stations components dispensers surfaces")
Manifest.__doc__ = """A drawn model's manifest (docs/hangar.md): its ``forward`` and ``up`` axes and ``scale``; and, in body
axes from the model's origin (x forward, y right, z down, metres): ``stations`` {number: (x, y, z, bay)}, a store's place
on each (bay 0: outside); ``engines`` and ``rockets``, each its line's figures; ``components``, each (kind, shape, section,
centre, half extents, name) - a crew member's, a tank's, an engine's; ``dispensers``, each (section, place, direction,
name); ``surfaces``, each (kind, figures)."""


def share_dir():
    """The package's data directory (fsim/share/flightsim)."""
    return _SHARE


def _name(name):
    name = str(name)
    if not name or name != os.path.basename(name) or name.startswith("."):
        raise ValueError("not a name: %r" % (name,))
    return name


def _json(*parts):
    path = os.path.join(_SHARE, *parts)
    if not os.path.isfile(path):
        raise KeyError(os.path.splitext(parts[-1])[0] if len(parts) == 2 else parts[-2])
    with open(path, encoding="utf8") as f:
        return json.load(f)


def _ids(folder):
    path = os.path.join(_SHARE, folder)
    return sorted(os.path.splitext(n)[0] for n in os.listdir(path) if n.endswith(".json")) if os.path.isdir(path) else []


def aircraft():
    """The types that carry a payload fit, by name (a World's "jsbsim:<name>"), sorted."""
    path = os.path.join(_SHARE, "aircraft")
    if not os.path.isdir(path):
        return []
    return sorted(n for n in os.listdir(path) if os.path.isfile(os.path.join(path, n, "payloads.json")))


def fit(name):
    """An aircraft's fit (its payloads.json): its ``payloads`` (name, card, count), its ``loadouts`` and their
    sources. KeyError for a type without one."""
    return _json("aircraft", _name(name), "payloads.json")


def loadouts(name):
    """The loads an aircraft's fit names (fsim.catalog.Loadout), in its order."""
    return [Loadout(lo.get("name", ""), lo.get("note", ""), tuple((int(s["station"]), s["store"]) for s in lo.get("stations", [])))
            for lo in fit(name).get("loadouts", [])]


def stores():
    """Every store card's id, sorted."""
    return _ids("stores")


def store(store_id):
    """A store's card (assets/stores/<id>.json): its ``designation``, ``class`` ("aam", "bomb", "tank", "pod", "cargo"),
    ``model`` and its ``parameters``, each with its value, unit, basis and source. KeyError for an unknown id."""
    return _json("stores", _name(store_id) + ".json")


def payloads():
    """Every payload card's id, sorted."""
    return _ids("payloads")


def payload(card_id):
    """A payload's card (assets/payloads/<id>.json): its ``designation``, ``family`` and ``parameters``. KeyError for
    an unknown id."""
    return _json("payloads", _name(card_id) + ".json")


def model(name):
    """The path of a type's drawn model (a glTF binary: an aircraft's, a store's), or None where it has none."""
    name = _name(name)
    path = os.path.join(_SHARE, "aircraft", name, name + ".glb")
    return path if os.path.isfile(path) else None


def manifest(name):
    """A drawn model's manifest (fsim.catalog.Manifest), or None where the model has none."""
    name = _name(name)
    path = os.path.join(_SHARE, "aircraft", name, name + ".glb.manifest")
    if not os.path.isfile(path):
        return None
    forward, up, scale = "+x", "+z", 1.0
    engines, rockets, stations, components, dispensers, surfaces = [], [], {}, [], [], []
    with open(path, encoding="utf8") as f:
        for line in f:
            parts = line.split("#", 1)[0].split()
            if not parts:
                continue
            key, rest = parts[0], parts[1:]
            try:
                if key == "forward":
                    forward = rest[0]
                elif key == "up":
                    up = rest[0]
                elif key == "scale":
                    scale = float(rest[0])
                elif key == "engine":
                    engines.append(tuple(float(x) for x in rest[1:]))
                elif key == "rocket":
                    rockets.append((rest[1],) + tuple(float(x) for x in rest[2:]))
                elif key == "station":
                    stations[int(rest[0])] = (float(rest[1]), float(rest[2]), float(rest[3]), int(rest[4]))
                elif key == "component":
                    components.append((rest[0], rest[1], rest[2], tuple(float(x) for x in rest[3:6]), tuple(float(x) for x in rest[6:9]),
                                       " ".join(rest[9:])))
                elif key == "dispenser":
                    dispensers.append((rest[0], tuple(float(x) for x in rest[1:4]), tuple(float(x) for x in rest[4:7]), " ".join(rest[7:])))
                else:
                    surfaces.append((key, tuple(float(x) for x in rest)))
            except (IndexError, ValueError):
                continue  # (a line of a kind, or a form, this reader does not know: left out)
    return Manifest(forward, up, scale, engines, rockets, stations, components, dispensers, surfaces)
