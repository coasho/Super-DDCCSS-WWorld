"""The scenario registry: what the testbench can run, grouped into suites (FA, and later OMS, P2P, C2...).

A scenario is a generator function decorated with @scenario. The runner calls it with a Test (testbench.runner.Test) and
advances it each time the wait it yields is over:

    @scenario("fa9.launch", group="FA-9 Ground operations", title="Takeoff from a runway",
              summary="A wing parked on 28R takes off along it and climbs out.", refs="ADR-29 4.49",
              options=[aircraft("c172", WINGS), wind()])
    def launch(t):
        v = t.spawn_on_runway(t.opt("aircraft"))
        a = t.watch(v.submit_behavior("launch", airfield=t.SFO, runway=t.RWY_28R), "launch")
        yield t.until_done(a, 300)
        t.check("the launch completed", a.state == fsim.ActivityState.COMPLETED)

A suite is a module in src/applications/testbench/fsim_testbench/suites/: every module there is imported at start, and its scenarios register
themselves. Adding a system's tests is adding a module (see suites/_template.py).
"""
import collections
import dataclasses
import importlib
import pkgutil


@dataclasses.dataclass
class Option:
    """A choice the user makes before a run: shown as a form field, passed to the scenario as t.opt(name)."""

    name: str
    label: str
    default: object
    kind: str = "number"  # "number", "choice", "bool", "text"
    choices: tuple = ()
    min: float = None
    max: float = None
    step: float = None
    unit: str = ""
    help: str = ""

    def parse(self, value):
        if value is None:
            return self.default
        if self.kind == "number":
            x = float(value)
            if self.min is not None:
                x = max(self.min, x)
            if self.max is not None:
                x = min(self.max, x)
            return x
        if self.kind == "bool":
            return value in (True, "true", "1", 1, "on")
        if self.kind == "choice":
            return value if value in self.choices else self.default
        return str(value)

    def describe(self):
        d = dataclasses.asdict(self)
        d["choices"] = list(self.choices)
        return d


@dataclasses.dataclass
class Scenario:
    id: str
    suite: str
    group: str
    title: str
    summary: str
    run: object
    refs: str = ""
    options: tuple = ()
    origin: str = "sfo"  # where it happens (see runner.ORIGINS)
    terrain: bool = True  # physics on the elevation tiles the viewer draws
    interactive: bool = False  # flown by hand, never ending: left out of headless runs

    def describe(self):
        return {"id": self.id, "suite": self.suite, "group": self.group, "title": self.title, "summary": self.summary,
                "refs": self.refs, "options": [o.describe() for o in self.options]}


REGISTRY = collections.OrderedDict()
SUITES = collections.OrderedDict()  # suite -> its description


def suite(name, description):
    """Declare a suite (once, in its module): the name scenarios give as `suite=`, and what it tests."""
    SUITES.setdefault(name, description)


def scenario(id, *, group, title, summary, refs="", options=(), suite="FA", origin="sfo", terrain=True, interactive=False):
    def register(fn):
        if id in REGISTRY:
            raise ValueError("scenario %r registered twice" % id)
        REGISTRY[id] = Scenario(id, suite, group, title, summary, fn, refs, tuple(options), origin, terrain, interactive)
        return fn
    return register


def discover():
    """Import every suite module: each registers its scenarios."""
    from . import suites
    for m in pkgutil.iter_modules(suites.__path__):
        if not m.name.startswith("_"):
            importlib.import_module(suites.__name__ + "." + m.name)
    return REGISTRY


def catalog():
    return {"suites": [{"name": n, "description": d} for n, d in SUITES.items()],
            "scenarios": [s.describe() for s in REGISTRY.values()]}


# --- options the suites share ------------------------------------------------------------------------------------

def aircraft(default, choices, label="Aircraft", name="aircraft"):
    return Option(name, label, default, "choice", tuple(choices))


def wind(default=0.0, label="Wind across the runway", name="wind"):
    return Option(name, label, default, "number", min=0.0, max=20.0, step=1.0, unit="m/s",
                  help="From the runway's left: from the south-south-west at SFO 28R")


def number(name, label, default, lo=None, hi=None, step=None, unit="", help=""):
    return Option(name, label, default, "number", min=lo, max=hi, step=step, unit=unit, help=help)


def choice(name, label, default, choices, help=""):
    return Option(name, label, default, "choice", tuple(choices), help=help)


def flag(name, label, default=False, help=""):
    return Option(name, label, default, "bool", help=help)
