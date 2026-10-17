"""Flight stability, aircraft by aircraft, flown through the Vehicle Interface at full load (or as spawned).

One scenario an aircraft (stability.<aircraft>): a takeoff by Flight Autonomy's launch, then - a fresh aircraft in the
air at 3,000 m and its cruise speed - a level hold, heading, altitude and speed steps, an orbit each way, a route, a
curve, a crosswind with gusts, and its slow flight: calibrated airspeeds stepped down to the least the VI lets it be
commanded, and turns there. Every command is a policy's, through fsim.guidance.hsa, pattern, route and curve (and
launch); nothing below them is commanded. A rotorcraft flies its own set from the hover.

Full load is what the aircraft's own files let it carry (docs/hangar.md, Mass accounting): its fixed load, every tank
full, and every station its fit's loadouts load - where two give a station a store, the heavier - held to its design's
declared maximum take-off weight where it has one: a loadout that would take it past is refused by the store management
as it is loaded (at the fuel it spawns with) and left off, and fuel is left out to meet the weight.

Each run's numbers and checks are kept: src/applications/testbench/results/stability/<aircraft>_<load>.json, and SUMMARY.md beside them,
written again from every result there.

    fsim testbench --headless stability.                 all 35, at full load
    fsim testbench --headless stability. --option load=spawn
    fsim testbench --headless stability.f16c
"""
import datetime
import glob
import json
import math
import os
import tomllib

import numpy as np

import fsim

from ..registry import choice, scenario, suite
from ._fa import ROTORS, WINGS

suite("Stability", "Flight stability of every aircraft at full load, flown through the Vehicle Interface")

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", ".."))
#: Where the results are kept; FSIM_STABILITY_RESULTS names another directory (a set kept apart: before a change, say)
RESULTS = os.environ.get("FSIM_STABILITY_RESULTS") or os.path.join(ROOT, "src", "applications", "testbench", "results", "stability")
G = 9.80665
R_EARTH = 6371008.8
KG_PER_LB = 0.45359237
POLL_S = 0.1  # the runner looks at a scenario's waits, and calls its trackers, this often

# What a check asks. Each is an engineering judgement of "flown stably", the same for every aircraft; where a limit
# scales with the aircraft it says how.
HOLD_ALTITUDE_M = 10.0        # a level hold: its height within this, peak to peak, over its second half-minute
HOLD_PITCH_DEG = 1.0          # ... its pitch
HOLD_ROLL_DEG = 2.0           # ... its bank
TURN_OVERSHOOT_DEG = 5.0      # a 90 deg heading change: past its heading by no more
TURN_ALTITUDE_M = 60.0        # ... and no lower than this below its height
STEP_OVERSHOOT = 0.15         # an altitude step: past it by no more than this share of it
SPEED_OVERSHOOT = 0.30        # a speed step
PATH_SHARE = 0.03             # an orbit or a route: off its path by no more than this share of its turn radius ...
PATH_M = 30.0                 # ... and this
SIDESLIP_DEG = 8.0            # a sustained turn: its sideslip no more
GUST_ALTITUDE_M = 30.0        # in the gusts: its height within this
SLOW_ALTITUDE_M = 60.0        # slow flight and its turns: no lower than this below its height
BANK_OVER_DEG = 8.0           # a turn: its bank past its limit by no more
BANK_OVER_S = 5.0             # a slow turn: past the bank its speed leaves (and 5 deg) for no longer - its roll's overshoot
AFTER_LAUNCH_M = 20.0         # after its launch: its height within this, peak to peak, from two minutes after it
COLS = ("t", "alt", "hdg", "tas", "cas", "roll", "pitch", "alpha", "beta", "vs", "thr", "lat", "lon", "gs", "xtk", "agl")
C = {n: i for i, n in enumerate(COLS)}


def wrap(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def offset(lat, lon, north_m, east_m):
    return lat + north_m / R_EARTH, lon + east_m / (R_EARTH * math.cos(lat))


def dist(lat1, lon1, lat2, lon2):
    return math.hypot((lat2 - lat1) * R_EARTH, (lon2 - lon1) * R_EARTH * math.cos(0.5 * (lat1 + lat2)))


def fin(x, digits=2):
    if x is None:
        return None
    x = float(x)
    return round(x, digits) if math.isfinite(x) else None


def spread(a, col, scale=1.0):
    """A column's peak to peak over a window's second half."""
    x = a[len(a) // 2:, C[col]] * scale
    return float(x.max() - x.min()) if len(x) else 0.0


def step_of(a, col, x0, x1, band, scale=1.0):
    """A step from x0 to x1: its overshoot, when it is within `band` for good (None: never), where it ends."""
    t = a[:, 0] - a[0, 0]
    x = a[:, C[col]] * scale
    x0, x1, band = x0 * scale, x1 * scale, band * scale
    sign = 1.0 if x1 >= x0 else -1.0
    err = x - x1
    outside = np.nonzero(np.abs(err) > band)[0]
    settle = 0.0 if not len(outside) else float(t[outside[-1] + 1]) if outside[-1] + 1 < len(t) else None
    return {"step": fin(x1 - x0), "overshoot": fin(max(0.0, float((sign * err).max()))), "settle_s": fin(settle, 1), "final_error": fin(err[-1])}


def turn_bank_deg(cas_ms, least_cas_ms, margin=1.3):
    """The bank a calibrated airspeed leaves a wing: its least airspeed's load factor over the VI's turn margin of 1.3,
    15 deg at the least (control::turnBankRadFor). `margin` 1: the bank it would stall at there."""
    if not least_cas_ms > 0.0:
        return 90.0
    n = (cas_ms / least_cas_ms) ** 2 / margin
    return max(math.degrees(math.acos(1.0 / n)) if n > 1.0 else 0.0, 15.0 if margin > 1.0 else 0.0)


class Flight:
    """One aircraft's run: the aircraft it flies now, what it records of it, its results."""

    def __init__(self, t, kind, load):
        self.t, self.kind, self.load = t, kind, load
        self.rotor = kind in ROTORS
        with open(os.path.join(ROOT, "aircraft", kind, kind + ".toml"), "rb") as f:
            self.design = tomllib.load(f)
        fit = os.path.join(ROOT, "aircraft", kind, "payloads.json")
        self.fit = {}
        if os.path.isfile(fit):
            with open(fit, encoding="utf-8") as f:
                self.fit = json.load(f)
        self.v = None
        self.alt0 = self.v0 = self.hdg0 = 0.0  # what it is asked to hold: its height, its speed, its heading
        self.samples = []
        self.lost = None       # why the aircraft was lost, since it was last spawned
        self.guard = True      # the losses looked for (not during a takeoff: it starts on the ground)
        self.live = None       # the activity whose progress is recorded
        self.hsa = None
        self.flags = 0
        self.k_held = None
        self.out = {"aircraft": kind, "load": load, "date": datetime.date.today().isoformat(), "phases": {}, "checks": []}
        t.every(self.record)

    # --- the checks, the results -------------------------------------------------------------------------------------
    def check(self, phase, label, ok, detail=""):
        ok = bool(ok)
        self.t.check("%s: %s" % (phase, label), ok, detail)
        self.out["checks"].append({"phase": phase, "check": label, "passed": ok, "detail": detail})
        return ok

    def write(self):
        failed = [c for c in self.out["checks"] if not c["passed"]]
        self.out["passed"], self.out["failed"] = len(self.out["checks"]) - len(failed), len(failed)
        self.out["simulated_s"] = fin(self.t.world.time, 0)
        os.makedirs(RESULTS, exist_ok=True)
        path = os.path.join(RESULTS, "%s_%s.json" % (self.kind, self.load))
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(self.out, f, indent=1)
            f.write("\n")
        write_summary()
        self.t.log("results kept: %s" % path)

    # --- the aircraft and its load -----------------------------------------------------------------------------------
    def loaded(self, v):
        """`v` given the load (the world stepped by the caller afterwards): what was loaded."""
        info = {"stores": [], "notes": []}
        if self.load != "full":
            info["notes"].append("as it spawns: its fixed load, its tanks as its design fills them, no stores")
            return info
        chosen = {}
        for lo in self.fit.get("loadouts", []):
            cards = {}
            try:
                v.load_loadout(lo["name"])
            except fsim.MissionRejected as ex:
                info["notes"].append("loadout %r refused: %s" % (lo["name"], ex.reason))
                continue
            for s in v.stores:
                cards[s.station] = cards.get(s.station, 0.0) + s.mass_kg
            for e in lo.get("stations", []):
                st = e["station"]
                if st not in chosen or cards.get(st, 0.0) > chosen[st][2]:
                    chosen[st] = (e["store"], int(e.get("count", 1)), cards.get(st, 0.0), lo["name"])
        for st, (store, count, kg, name) in sorted(chosen.items()):
            try:
                v.load_store(st, store, count)
                info["stores"].append({"station": st, "store": store, "count": count, "kg": fin(kg, 1), "loadout": name})
            except fsim.MissionRejected as ex:
                info["notes"].append("station %d %s refused: %s" % (st, store, ex.reason))
        mass = self.design.get("mass", {})
        tanks = mass.get("tank", [])
        stores = sum(s["kg"] for s in info["stores"])
        if tanks:
            capacity = sum(t["capacity"] for t in tanks)
            share = 1.0
            fixed = sum(p["mass"] for p in mass.get("payload", []))
            most = mass.get("max_takeoff", mass.get("mtow"))  # (its design's maximum take-off weight, where it declares one)
            if most and mass["empty"] + fixed + capacity + stores > most:
                share = max(0.0, (most - mass["empty"] - fixed - stores) / capacity)
                info["notes"].append("fuel %.0f %% of capacity: held to its declared maximum take-off weight, %.0f kg" % (100.0 * share, most))
            for i, t in enumerate(tanks):
                v.set_property("propulsion/tank[%d]/contents-lbs" % i, share * t["capacity"] / KG_PER_LB)
            info["fuel_kg"] = fin(share * capacity, 0)
        elif not self.fit.get("loadouts"):
            info["notes"].append("nothing to add: its weight is one figure, its tank full or its battery charged within it")
        info["stores_kg"] = fin(stores, 1)
        return info

    def spawn(self, on_runway=False):
        t = self.t
        self.drop()
        if on_runway:
            self.v = t.spawn_on_runway(self.kind)
        else:
            p = t.performance(self.kind)
            if self.rotor:
                small = self.design.get("mass", {}).get("mass_kg", 1e9) < 50.0
                self.alt0, self.v0 = (60.0 if small else 300.0), 0.0
            else:
                ceiling = p.ceiling_m if math.isfinite(p.ceiling_m) else 6000.0
                self.alt0, self.v0 = min(3000.0, 0.5 * ceiling), p.cruise_tas_ms
            # (a wing west, out over the sea: an hour's flight east of the airfield stands the Sierra Nevada)
            self.v = t.spawn(self.kind, 0.0, 9000.0, alt_m=self.alt0, heading_deg=90.0 if self.rotor else 270.0, speed_ms=self.v0)
        self.hdg0 = self.v.state.euler_rad[2]
        self.unwrapped = self.hdg0
        self.lost, self.hsa, self.live = None, None, None
        self.guard = not on_runway
        return self.v

    def drop(self):
        if self.v is None:
            return
        try:
            self.t.runner.names.discard(self.v.name)
            self.t.vehicles.remove(self.v)
            self.v.remove()
        except Exception:  # noqa: BLE001 - gone already
            pass
        self.v = None

    # --- recording ---------------------------------------------------------------------------------------------------
    def record(self):
        v = self.v
        if v is None or self.lost:
            return
        s = v.state
        self.unwrapped += wrap(s.euler_rad[2] - self.unwrapped)
        xtk = math.nan
        for act in (self.hsa, self.live):
            if act is None:
                continue
            info = act.info
            if info is not None and info.state == fsim.ActivityState.ACTIVE:
                self.flags |= int(info.constraints)
        if self.live is not None:
            pr = self.live.progress
            if pr is not None and pr.cross_track_m is not None:
                xtk = pr.cross_track_m
        vn, ve, vd = s.velocity_ned_ms
        self.samples.append((self.t.world.time, s.altitude_msl_m, self.unwrapped, s.airspeed_true_ms, s.airspeed_calibrated_ms, s.euler_rad[0],
                             s.euler_rad[1], s.alpha_rad, s.beta_rad, -vd, s.throttle_position[0], s.latitude_rad, s.longitude_rad,
                             math.hypot(vn, ve), xtk, s.altitude_agl_m))
        if s.crashed:
            self.lost = "struck the ground"
        elif s.diverged:
            self.lost = "its flight model diverged"
        elif self.guard and s.altitude_agl_m < 25.0 and not (self.rotor and self.alt0 < 100.0):
            self.lost = "descended to %.0f m above the ground" % s.altitude_agl_m

    def window(self, i0):
        return np.asarray(self.samples[i0:], dtype=float).reshape(-1, len(COLS))

    def extremes(self, i0):
        a = self.window(i0)
        if not len(a):
            return {}
        d = math.degrees
        return {"alpha_max_deg": fin(d(a[:, C["alpha"]].max()), 1), "sideslip_max_deg": fin(d(np.abs(a[:, C["beta"]]).max()), 1),
                "bank_max_deg": fin(d(np.abs(a[:, C["roll"]]).max()), 1), "cas_min_ms": fin(a[:, C["cas"]].min(), 1),
                "altitude_min_m": fin(a[:, C["alt"]].min(), 0), "throttle_mean": fin(a[:, C["thr"]].mean()),
                "flags": [f.name for f in fsim.ActivityFlag if self.flags & f]}

    # --- waits -------------------------------------------------------------------------------------------------------
    def fly(self, seconds):
        yield self.t.until(lambda: self.lost is not None, seconds)

    def until(self, inband, hold_s, timeout_s, label="", abort=None):
        """Until inband(state) has held hold_s: True; False at the timeout, once abort(state) holds, or once the
        aircraft is lost."""
        held, gave_up = [0.0], [False]

        def ready():
            if self.lost:
                return True
            s = self.v.state
            if abort is not None and abort(s):
                gave_up[0] = True
                return True
            held[0] = held[0] + POLL_S if inband(s) else 0.0
            return held[0] >= hold_s

        ok = yield self.t.until(ready, timeout_s, label)
        return bool(ok) and not self.lost and not gave_up[0]

    def until_done(self, activity, timeout_s):
        yield self.t.until(lambda: self.lost is not None or not activity.live, timeout_s)

    # --- phases: each a generator; its result into out["phases"] -------------------------------------------------------
    def phase(self, name, body):
        """`body()` flown as a phase - on a fresh aircraft, settled, if the last was lost; its checks failed if this one is."""
        if self.v is not None and not self.lost:
            try:  # (a battery that would not last the run - a Crazyflie's is seven minutes: each phase on a fresh one)
                nav = self.v.navigation_report()
                if nav.energy == fsim.Energy.BATTERY and math.isfinite(nav.endurance_s) and nav.endurance_s < 900.0:
                    self.drop()
                    self.out["fresh_each_phase"] = True
            except fsim.Error:
                pass
        if self.v is None or self.lost:
            self.spawn()
            if not self.out.get("fresh_each_phase"):
                self.out.setdefault("respawns", []).append(name)
            yield from self.fly(0.5)
            self.loaded(self.v)
            yield from self.settle({})
        res = {}
        i0 = len(self.samples)
        self.flags = 0
        try:
            yield from body(res)
        except fsim.Rejected as ex:
            res["rejected"] = str(ex)
            self.check(name, "its command accepted", False, str(ex))
        res.update(self.extremes(i0))
        if self.lost:
            res["lost"] = self.lost
        self.check(name, "the aircraft kept", not self.lost, self.lost or "")
        self.out["phases"][name] = res

    def speed_band(self):
        return max(1.5, 0.012 * self.v0)

    def turn_radius(self, speed):
        p = self.t.performance(self.kind)
        if self.rotor:
            acc = p.max_acceleration_ms2 if math.isfinite(p.max_acceleration_ms2) else 3.0
            bw = p.velocity_bandwidth_rad_s if math.isfinite(p.velocity_bandwidth_rad_s) else 0.5
            return max(speed * speed / (0.6 * acc), 1.3 * speed / (bw / 3.0), 5.0)
        bank = p.max_bank_rad if math.isfinite(p.max_bank_rad) else math.radians(30.0)
        return speed * speed / (G * math.tan(0.8 * bank))

    def bank_limit_deg(self):
        p = self.t.performance(self.kind)
        return math.degrees(p.max_bank_rad) if math.isfinite(p.max_bank_rad) else 45.0

    def settle(self, res):
        v = self.v
        if self.rotor:
            self.hsa = v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=0.0, speed_reference="ground_speed")
            ok = yield from self.until(lambda s: abs(s.altitude_msl_m - self.alt0) < 2.0 and math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1]) < 0.5
                                       and abs(wrap(s.euler_rad[2] - self.hdg0)) < math.radians(2.0), 10.0, 120.0)
        else:
            self.hsa = v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=self.v0, speed_reference="true_airspeed")
            band = self.speed_band()
            ok = yield from self.until(lambda s: abs(s.altitude_msl_m - self.alt0) < 10.0 and abs(s.airspeed_true_ms - self.v0) < band
                                       and abs(wrap(s.euler_rad[2] - self.hdg0)) < math.radians(1.0) and abs(s.velocity_ned_ms[2]) < 1.0, 20.0, 300.0)
        res["settled"] = ok
        return ok

    def p_takeoff(self):
        """Its launch from the runway, loaded where it stands; followed for five minutes after it completes."""
        t, name, res = self.t, "takeoff", {}
        v = self.spawn(on_runway=True)
        res["mass_spawned_kg"] = fin(v.mass_kg, 3)
        res["loading"] = self.loaded(v)
        yield t.wait(3.0)  # (on its wheels under the load)
        res["mass_kg"] = fin(v.mass_kg, 3)
        self.out["mass_spawned_kg"], self.out["mass_kg"], self.out["loading"] = res["mass_spawned_kg"], res["mass_kg"], res["loading"]
        try:
            a = self.live = t.watch(v.submit_behavior("launch", airfield=t.SFO, runway=t.RWY), "launch")
        except fsim.Rejected as ex:
            self.check(name, "its launch accepted", False, str(ex))
            self.out["phases"][name] = res
            return
        i0 = len(self.samples)
        yield from self.until_done(a, 600.0)
        w = self.window(i0)
        done = a.state == fsim.ActivityState.COMPLETED
        air = np.nonzero(w[:, C["agl"]] > 3.0)[0] if len(w) else []
        if len(air):
            along, _ = t.runway_offsets(_Place(w[air[0], C["lat"]], w[air[0], C["lon"]]))
            res["roll_m"], res["liftoff_cas_ms"] = fin(along, 0), fin(w[air[0], C["cas"]], 1)
        res["alpha_max_deg"] = fin(math.degrees(w[:, C["alpha"]].max()), 1) if len(w) else None
        self.check(name, "its launch completes", done and not self.lost, "%s %s%s" % (a.state.name, getattr(a.info.reason, "name", a.info.reason) if a.info else "",
                                                                                  "; " + self.lost if self.lost else ""))
        if done and not self.lost:
            self.guard = True
            i1 = len(self.samples)
            yield from self.fly(300.0)
            w = self.window(i1)
            tail = w[int(120.0 / POLL_S):]
            if len(tail) and not self.lost:
                swing = float(tail[:, C["alt"]].max() - tail[:, C["alt"]].min())  # (above sea level: the ground under it is not level)
                res["after"] = {"height_swing_m": fin(swing, 1), "pitch_swing_deg": fin(math.degrees(float(tail[:, C["pitch"]].max() - tail[:, C["pitch"]].min())), 1),
                                "throttle": [fin(tail[:, C["thr"]].min()), fin(tail[:, C["thr"]].max())], "cas_ms": fin(tail[-1, C["cas"]], 1)}
                self.check(name, "steady after its launch", swing <= AFTER_LAUNCH_M, "its height swings %.0f m, from two minutes after it" % swing)
            self.check(name, "kept after its launch", not self.lost, self.lost or "")
        self.out["phases"][name] = res
        self.live = None
        self.drop()

    def p_start(self, res):
        """In the air, loaded, on an hsa that holds what it was started at; the load's own transient flown out."""
        res["settled"] = yield from self.settle({})
        self.check("start", "settles on its height, heading and speed", res["settled"])

    def p_hold(self, res):
        i0 = len(self.samples)
        yield from self.fly(60.0)
        a = self.window(i0)
        if self.lost or not len(a):
            return
        res.update({"altitude_swing_m": fin(spread(a, "alt")), "pitch_swing_deg": fin(spread(a, "pitch", math.degrees(1.0))),
                    "roll_swing_deg": fin(spread(a, "roll", math.degrees(1.0))), "altitude_error_m": fin(float(a[:, C["alt"]].mean()) - self.alt0),
                    "alpha_deg": fin(math.degrees(float(a[:, C["alpha"]].mean())), 1)})
        if self.rotor:
            res["drift_m"] = fin(max(dist(a[0, C["lat"]], a[0, C["lon"]], la, lo) for la, lo in a[::10, [C["lat"], C["lon"]]]), 1)
        self.check("hold", "level and steady", res["altitude_swing_m"] <= HOLD_ALTITUDE_M and res["pitch_swing_deg"] <= HOLD_PITCH_DEG
                   and res["roll_swing_deg"] <= HOLD_ROLL_DEG, "height %.1f m, pitch %.2f deg, bank %.2f deg, peak to peak" % (
                       res["altitude_swing_m"], res["pitch_swing_deg"], res["roll_swing_deg"]))

    def p_heading(self, name, res, turn_deg):
        i0 = len(self.samples)
        h0 = self.unwrapped
        target = h0 + math.radians(turn_deg)
        self.hsa.update(heading_rad=wrap(target))
        band = math.radians(1.5)
        ok = yield from self.until(lambda s: abs(self.unwrapped - target) < band and abs(s.euler_rad[0]) < math.radians(4.0), 15.0, 300.0)
        a = self.window(i0)
        self.hdg0 = wrap(target)
        if self.lost or not len(a):
            return
        st = step_of(a, "hdg", h0, target, band, math.degrees(1.0))
        res.update({"settled": ok, "heading": st, "altitude_lost_m": fin(self.alt0 - float(a[:, C["alt"]].min()), 1),
                    "bank_deg": fin(math.degrees(float(np.abs(a[:, C["roll"]]).max())), 1)})
        if not self.rotor:
            res["speed_lost_ms"] = fin(self.v0 - float(a[:, C["tas"]].min()), 1)
        self.check(name, "turned onto its heading", ok and st["overshoot"] <= TURN_OVERSHOOT_DEG, "settled in %s s, %.1f deg past it" % (st["settle_s"], st["overshoot"]))
        self.check(name, "its height kept", res["altitude_lost_m"] <= TURN_ALTITUDE_M, "%.0f m lost" % res["altitude_lost_m"])
        if not self.rotor:
            self.check(name, "within its bank limit", res["bank_deg"] <= self.bank_limit_deg() + BANK_OVER_DEG,
                       "%.0f deg, its limit %.0f" % (res["bank_deg"], self.bank_limit_deg()))

    def p_altitude(self, name, res, delta):
        i0 = len(self.samples)
        start, target = self.alt0, self.alt0 + delta
        self.hsa.update(altitude_m=target)
        band = 2.0 if self.rotor else 10.0
        ok = yield from self.until(lambda s: abs(s.altitude_msl_m - target) < band and abs(s.velocity_ned_ms[2]) < (0.5 if self.rotor else 1.0), 15.0, 480.0)
        a = self.window(i0)
        self.alt0 = target
        if self.lost or not len(a):
            return
        st = step_of(a, "alt", start, target, band)
        res.update({"settled": ok, "altitude": st})
        if not self.rotor:
            res["speed_lost_ms"] = fin(self.v0 - float(a[:, C["tas"]].min()), 1)
        self.check(name, "reaches and holds its height", ok and st["overshoot"] <= STEP_OVERSHOOT * abs(delta), "settled in %s s, %.1f m past it" % (st["settle_s"], st["overshoot"]))

    def p_speed(self, name, res, target, reference="true_airspeed", col="tas"):
        i0 = len(self.samples)
        start = float(self.samples[-1][C[col]])
        self.hsa.update(speed=target, speed_reference=reference)
        band = 0.5 if self.rotor else max(1.5, 0.012 * max(target, self.v0))

        def there(s):
            now = s.airspeed_true_ms if col == "tas" else math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1])
            return abs(now - target) < band and abs(s.altitude_msl_m - self.alt0) < (3.0 if self.rotor else 15.0)

        ok = yield from self.until(there, 15.0, 300.0)
        a = self.window(i0)
        self.v0 = target
        if self.lost or not len(a):
            return
        st = step_of(a, col, start, target, band)
        res.update({"settled": ok, "speed": st, "altitude_error_m": fin(float(np.abs(a[:, C["alt"]] - self.alt0).max()), 1)})
        self.check(name, "reaches and holds its speed", ok and st["overshoot"] <= max(SPEED_OVERSHOOT * abs(target - start), 2.0 * band),
                   "settled in %s s, %.1f m/s past it" % (st["settle_s"], st["overshoot"]))

    def path_limit(self, radius):
        return PATH_SHARE * radius + PATH_M

    def rejoin(self):
        s = self.v.state
        self.hdg0 = s.euler_rad[2]
        if self.rotor:
            self.hsa = self.v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=self.v0, speed_reference="ground_speed")
            yield from self.fly(20.0)
        else:
            self.hsa = self.v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=self.v0, speed_reference="true_airspeed")
            band = self.speed_band()
            yield from self.until(lambda q: abs(q.altitude_msl_m - self.alt0) < 10.0 and abs(q.airspeed_true_ms - self.v0) < band
                                  and abs(q.euler_rad[0]) < math.radians(2.0), 10.0, 120.0)

    def p_orbit(self, name, res, clockwise):
        v, s = self.v, self.v.state
        speed = max(float(self.samples[-1][C["gs"]]), 1.0)
        r = 1.2 * self.turn_radius(speed)
        h = s.euler_rad[2]
        side = 0.5 * math.pi if clockwise else -0.5 * math.pi
        centre = offset(s.latitude_rad, s.longitude_rad, r * math.cos(h + side), r * math.sin(h + side))
        a = self.live = self.t.watch(v.submit_pattern(pattern="orbit", latitude_rad=centre[0], longitude_rad=centre[1], radius_m=r, altitude_m=self.alt0,
                                                      orbits=1, clockwise=clockwise), name)
        i0 = len(self.samples)
        yield from self.until_done(a, 1.8 * 2.0 * math.pi * r / speed + 90.0)
        w = self.window(i0)
        done = a.state == fsim.ActivityState.COMPLETED
        self.live = None
        res["radius_m"] = fin(r, 0)
        if len(w) > int(10.0 / POLL_S) and not self.lost:
            body = w[int(8.0 / POLL_S):]
            off = np.array([dist(centre[0], centre[1], la, lo) for la, lo in body[::5, [C["lat"], C["lon"]]]]) - r
            bank = np.abs(body[:, C["roll"]])
            turning = body[bank > 0.5 * bank.max()]
            res.update({"completed": done, "off_path_m": fin(float(np.abs(off).max()), 1), "altitude_error_m": fin(float(np.abs(body[:, C["alt"]] - self.alt0).max()), 1),
                        "sideslip_deg": fin(math.degrees(float(np.abs(turning[:, C["beta"]]).max())), 1) if len(turning) else None})
            self.check(name, "flown round", done, "%s" % a.state.name)
            self.check(name, "on its circle", res["off_path_m"] <= self.path_limit(r), "%.0f m off, of %.0f m radius" % (res["off_path_m"], r))
            if not self.rotor:
                self.check(name, "coordinated", (res["sideslip_deg"] or 0.0) <= SIDESLIP_DEG, "%.1f deg of sideslip" % (res["sideslip_deg"] or 0.0))
        if not self.lost:
            yield from self.rejoin()

    def p_route(self, name, res):
        v, s = self.v, self.v.state
        speed = max(float(self.samples[-1][C["gs"]]), 1.0)
        r = self.turn_radius(speed)
        leg = max(4.0 * r, 20.0 * speed)
        h = s.euler_rad[2]
        fwd, rgt = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
        pts = []
        for i, (f, g) in enumerate(((1.0, 0.0), (1.0, 1.0), (2.0, 1.0))):
            lat, lon = offset(s.latitude_rad, s.longitude_rad, leg * (f * fwd[0] + g * rgt[0]), leg * (f * fwd[1] + g * rgt[1]))
            pts.append(fsim.Waypoint(lat, lon, altitude_m=self.alt0, id=101 + i))
        a = self.live = self.t.watch(v.submit_route(pts), name)
        i0 = len(self.samples)
        yield from self.until_done(a, 1.6 * 3.0 * leg / speed + 120.0)
        w = self.window(i0)
        done = a.state == fsim.ActivityState.COMPLETED
        self.live = None
        res["leg_m"] = fin(leg, 0)
        if len(w) > int(5.0 / POLL_S) and not self.lost:
            x = w[int(3.0 / POLL_S):, C["xtk"]]
            x = x[np.isfinite(x)]
            res.update({"completed": done, "off_path_m": fin(float(np.abs(x).max()), 1) if len(x) else None,
                        "altitude_error_m": fin(float(np.abs(w[:, C["alt"]] - self.alt0).max()), 1)})
            self.check(name, "flown to its last point", done, "%s" % a.state.name)
            self.check(name, "on its legs and turns", (res["off_path_m"] or 0.0) <= self.path_limit(r), "%.0f m off at the most" % (res["off_path_m"] or 0.0))
        if not self.lost:
            yield from self.rejoin()

    def p_curve(self, name, res):
        """An S: one quintic Bezier, 6 turn radii long and 1.5 across - its tightest a quarter of what the aircraft turns."""
        v, s = self.v, self.v.state
        speed = max(float(self.samples[-1][C["gs"]]), 1.0)
        r = self.turn_radius(speed)
        length, across = 6.0 * r, 1.5 * r
        h = s.euler_rad[2]
        f = [0.0, 0.2 * length, 0.4 * length, 0.6 * length, 0.8 * length, length]
        g = [0.0, 0.0, 0.0, across, across, across]
        north = [a * math.cos(h) - b * math.sin(h) for a, b in zip(f, g)]
        east = [a * math.sin(h) + b * math.cos(h) for a, b in zip(f, g)]
        a = self.live = self.t.watch(v.submit_curve([fsim.BezierSegment(north, east, [0.0] * 6)]), name)
        i0 = len(self.samples)
        yield from self.until_done(a, 2.0 * length / speed + 90.0)
        w = self.window(i0)
        done = a.state == fsim.ActivityState.COMPLETED
        self.live = None
        if len(w) > int(5.0 / POLL_S) and not self.lost:
            x = w[int(3.0 / POLL_S):, C["xtk"]]
            x = x[np.isfinite(x)]
            res.update({"completed": done, "off_path_m": fin(float(np.abs(x).max()), 1) if len(x) else None})
            self.check(name, "flown to its end", done, "%s" % a.state.name)
            self.check(name, "on its curve", (res["off_path_m"] or 0.0) <= PATH_M, "%.0f m off at the most" % (res["off_path_m"] or 0.0))
        if not self.lost:
            yield from self.rejoin()

    def p_gusts(self, name, res):
        p = self.t.performance(self.kind)
        ref = max(self.v0, p.cruise_tas_ms if math.isfinite(p.cruise_tas_ms) else 10.0)
        wind, peak = (min(6.0, 0.5 * ref + 1.0), min(4.0, 0.35 * ref + 0.5)) if self.rotor else (min(8.0, 0.12 * ref), min(6.0, 0.10 * ref))
        res["crosswind_ms"], res["gust_ms"] = fin(wind, 1), fin(peak, 1)
        self.t.world.set_wind(math.degrees(self.hdg0) + 90.0, wind)  # (the session's wind: every run's, for these two minutes)
        self.v.add_effect("wind_gusts", mean_interval_s=8.0, peak_ms=peak, duration_s=4.0)
        i0 = len(self.samples)
        try:
            yield from self.fly(90.0)
        finally:
            try:
                self.v.clear_effects()
                self.t.world.set_wind(0.0, 0.0)
            except Exception:  # noqa: BLE001 - the aircraft gone
                pass
        a = self.window(i0)
        if not self.lost and len(a):
            res.update({"altitude_error_m": fin(float(np.abs(a[:, C["alt"]] - self.alt0).max()), 1),
                        "roll_swing_deg": fin(math.degrees(float(a[:, C["roll"]].max() - a[:, C["roll"]].min())), 1),
                        "sideslip_deg": fin(math.degrees(float(np.abs(a[:, C["beta"]]).max())), 1)})
            if self.rotor:
                res["drift_m"] = fin(max(dist(a[0, C["lat"]], a[0, C["lon"]], la, lo) for la, lo in a[::10, [C["lat"], C["lon"]]]), 1)
            self.check(name, "its height kept in the gusts", res["altitude_error_m"] <= GUST_ALTITUDE_M, "%.1f m off at the most" % res["altitude_error_m"])
            yield from self.fly(30.0)

    def low_energy(self):
        """Whether the hsa says, now, that it flies faster than it was asked to keep its height (the VI's speed floor)."""
        info = self.hsa.info if self.hsa is not None else None
        return info is not None and bool(int(info.constraints) & int(fsim.ActivityFlag.LOW_ENERGY))

    def level_at(self, target):
        """A state's test: level at the height, at the calibrated airspeed - or level on the speed floor above it, saying so
        (within the height a slow flight may lose: it comes back up past its height as its thrust comes in, and is slow
        to settle from there)."""
        return lambda s: abs(s.velocity_ned_ms[2]) < 1.0 and (
            (abs(s.altitude_msl_m - self.alt0) < 15.0 and abs(s.airspeed_calibrated_ms - target) < 2.0)
            or (abs(s.altitude_msl_m - self.alt0) < SLOW_ALTITUDE_M and s.airspeed_calibrated_ms > target and self.low_energy()))

    def p_slow(self, name, res):
        """Down to the least the VI lets it be commanded: calibrated airspeeds stepped down to its floor (its performance's
        least_cas_ms as it is now, loaded - 1.1 times the least airspeed its weight leaves it: a command below is clamped to
        it). Each is flown level - at the speed, or where the
        aircraft cannot hold its height there, faster on the VI's speed floor with LOW_ENERGY on its activity - or not."""
        v = self.v
        floor = v.performance.least_cas_ms
        res["floor_cas_ms"], res["floor_as_spawned_cas_ms"] = fin(floor, 1), fin(self.t.performance(self.kind).least_cas_ms, 1)
        steps = res["steps"] = []
        self.k_held = self.k_level = None
        start = float(self.samples[-1][C["cas"]])
        seen, every = 0, True
        for k in (1.5, 1.4, 1.3, 1.2, 1.1, 1.05, 1.0):
            target = k * floor
            if target >= start - 2.0:
                continue
            row = {"k": k, "cas_ms": fin(target, 1)}
            steps.append(row)
            self.hsa = v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=target, speed_reference="calibrated_airspeed")
            i0 = len(self.samples)
            before, self.flags = self.flags, 0
            ok = yield from self.until(self.level_at(target), 15.0, 240.0, abort=lambda s: s.altitude_msl_m < self.alt0 - 300.0)
            w = self.window(i0)
            row["level"] = bool(ok)
            row["flags"] = [f.name for f in fsim.ActivityFlag if self.flags & f]
            seen |= self.flags
            self.flags |= before
            if len(w):
                end = float(w[-10:, C["cas"]].mean())
                row.update({"cas_end_ms": fin(end, 1), "altitude_lost_m": fin(self.alt0 - float(w[:, C["alt"]].min()), 0),
                            "alpha_max_deg": fin(math.degrees(float(w[:, C["alpha"]].max())), 1),
                            "throttle_end": fin(float(w[-10:, C["thr"]].mean())), "sink_end_ms": fin(-float(w[-10:, C["vs"]].mean()), 1)})
                row["held"] = bool(ok) and abs(end - target) < 2.0
            if not row["level"]:
                break
            self.k_level = k
            every = every and bool(row.get("held"))
            if every:
                self.k_held = k
        res["slowest_held_k"], res["slowest_level_k"] = self.k_held, self.k_level
        lost = max((r.get("altitude_lost_m") or 0.0) for r in steps) if steps else 0.0
        self.check(name, "level at the least speed the VI accepts", self.k_level == 1.0 and lost <= SLOW_ALTITUDE_M,
                   "level down to %s times its floor of %.1f m/s (the speed itself held down to %s times); %.0f m lost at the most" % (
                       self.k_level, floor, self.k_held, lost))
        faster = [r for r in steps if r.get("level") and not r.get("held")]
        self.check(name, "says where it flies faster than asked", all("LOW_ENERGY" in r["flags"] for r in faster),
                   "; ".join("asked %.1f, flew %.1f m/s: %s" % (r["cas_ms"], r["cas_end_ms"], ", ".join(r["flags"]) or "no flag") for r in faster)
                   or "each speed asked was flown")
        if lost > SLOW_ALTITUDE_M:
            self.check(name, "says that its height was lost", bool(seen & int(fsim.ActivityFlag.DEVIATING)), "%.0f m lost; %s" % (
                lost, ", ".join(f.name for f in fsim.ActivityFlag if seen & f) or "no flag"))
        if not self.lost:  # away from it again: its cruise
            self.hsa = v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=self.v0, speed_reference="true_airspeed")
            band = self.speed_band()
            res["recovered"] = yield from self.until(lambda s: abs(s.altitude_msl_m - self.alt0) < 15.0 and abs(s.airspeed_true_ms - self.v0) < band
                                                     and abs(s.velocity_ned_ms[2]) < 1.0, 15.0, 400.0)

    def p_slow_turn(self, name, res, k):
        """A 60 deg turn at k times the VI's floor (k None: at the floor itself), within the bank its speed leaves it: the
        VI's turn margin (docs/vehicle-interface.md, 7.3)."""
        v = self.v
        floor, least = v.performance.least_cas_ms, v.performance.min_cas_ms
        target = (k or 1.0) * floor
        res["cas_ms"] = fin(target, 1)
        self.hsa = v.submit_hsa(heading_rad=self.hdg0, altitude_m=self.alt0, speed=target, speed_reference="calibrated_airspeed")
        res["at_speed"] = yield from self.until(self.level_at(target), 15.0, 400.0)
        if self.lost:
            return
        if not res["at_speed"]:
            self.check(name, "level at the speed before its turn", False, "not level at %.1f m/s within 400 s" % target)
        i0 = len(self.samples)
        h0 = self.unwrapped
        res["cas_flown_ms"] = fin(float(self.samples[-1][C["cas"]]), 1)
        heading = h0 + math.radians(60.0)
        # (its height: between where it starts the turn - on its speed floor it may still be coming back to its height - and
        # the height it holds)
        z0 = float(self.samples[-1][C["alt"]])
        low, high = min(z0, self.alt0), max(z0, self.alt0)
        res["altitude_start_m"] = fin(z0, 0)
        self.hsa.update(heading_rad=wrap(heading))
        ok = yield from self.until(lambda s: abs(self.unwrapped - heading) < math.radians(1.5) and abs(s.euler_rad[0]) < math.radians(4.0)
                                   and low - 15.0 < s.altitude_msl_m < high + 15.0, 15.0, 300.0)
        w = self.window(i0)
        self.hdg0 = wrap(heading)
        if self.lost or not len(w):
            return
        # the bank its speed leaves it, sample by sample (the VI's turn margin; its limit, if less): past it by 5 deg for
        # no longer than its roll overshoots, and never to the bank it would stall at
        bank = np.degrees(np.abs(w[:, C["roll"]]))
        left = np.array([min(self.bank_limit_deg(), turn_bank_deg(c, least)) for c in w[:, C["cas"]]])
        stall = np.array([turn_bank_deg(c, least, 1.0) for c in w[:, C["cas"]]])
        over_s = float(np.count_nonzero(bank > left + 5.0)) * POLL_S
        res.update({"settled": ok, "altitude_lost_m": fin(low - float(w[:, C["alt"]].min()), 0), "bank_deg": fin(float(bank.max()), 1),
                    "bank_margin_deg": fin(float(left.max()), 1), "bank_over_s": fin(over_s, 1),
                    "stall_bank_deg": fin(float(stall.min()), 1)})
        self.check(name, "turned, its height kept", ok and res["altitude_lost_m"] <= SLOW_ALTITUDE_M, "%.0f m lost, %.0f deg of bank" % (
            res["altitude_lost_m"], res["bank_deg"]))
        self.check(name, "within the bank its speed leaves", over_s <= BANK_OVER_S and (float(left.max()) < 16.0 or bool(np.all(bank < np.maximum(stall, left + 5.0)))),
                   "%.0f deg at the most; its speed leaves %.0f (its limit %.0f), past that for %.1f s" % (
                       res["bank_deg"], res["bank_margin_deg"], self.bank_limit_deg(), over_s))
        upset = bool(self.flags & int(fsim.ActivityFlag.UPSET))
        self.check(name, "no upset", not upset, "UPSET on its activity" if upset else "")

    # --- the run -----------------------------------------------------------------------------------------------------
    def run(self):
        ph = self.phase
        yield from self.p_takeoff()
        self.spawn()
        yield from self.fly(0.5)
        loading = self.loaded(self.v)
        yield from self.fly(0.5)
        if "loading" not in self.out:  # (no takeoff flown: weighed here)
            self.out["loading"], self.out["mass_kg"] = loading, fin(self.v.mass_kg, 3)
        self.out["flown_at"] = {"altitude_m": self.alt0, "tas_ms": fin(self.v0, 1), "mass_kg": fin(self.v.mass_kg, 3)}
        perf = self.v.performance
        self.out["performance"] = {"min_cas_ms": fin(perf.min_cas_ms, 1), "least_cas_ms": fin(perf.least_cas_ms, 1), "max_bank_deg": fin(self.bank_limit_deg(), 0), "cruise_tas_ms": fin(perf.cruise_tas_ms, 1)}
        yield from ph("start", self.p_start)
        yield from ph("hold", self.p_hold)
        yield from ph("heading +90", lambda r: self.p_heading("heading +90", r, 90.0))
        yield from ph("heading -90", lambda r: self.p_heading("heading -90", r, -90.0))
        if self.rotor:
            small = self.alt0 < 100.0
            p = self.t.performance(self.kind)
            top = p.max_ground_speed_ms if math.isfinite(p.max_ground_speed_ms) else 2.0 * p.cruise_tas_ms
            yield from ph("climb", lambda r: self.p_altitude("climb", r, 15.0 if small else 60.0))
            yield from ph("descent", lambda r: self.p_altitude("descent", r, -15.0 if small else -60.0))
            yield from ph("to its cruise", lambda r: self.p_speed("to its cruise", r, p.cruise_tas_ms, "ground_speed", "gs"))
            yield from ph("orbit right", lambda r: self.p_orbit("orbit right", r, 1))
            yield from ph("orbit left", lambda r: self.p_orbit("orbit left", r, 0))
            yield from ph("route", lambda r: self.p_route("route", r))
            yield from ph("curve", lambda r: self.p_curve("curve", r))
            yield from ph("to 90 % of its top speed", lambda r: self.p_speed("to 90 % of its top speed", r, 0.9 * top, "ground_speed", "gs"))
            yield from ph("to the hover", lambda r: self.p_speed("to the hover", r, 0.0, "ground_speed", "gs"))
            yield from ph("gusts", lambda r: self.p_gusts("gusts", r))
        else:
            dz = 300.0 if self.v0 > 40.0 else 100.0
            cruise = self.v0
            yield from ph("climb", lambda r: self.p_altitude("climb", r, dz))
            yield from ph("descent", lambda r: self.p_altitude("descent", r, -dz))
            yield from ph("speed -15 %", lambda r: self.p_speed("speed -15 %", r, 0.85 * cruise))
            yield from ph("speed back", lambda r: self.p_speed("speed back", r, cruise))
            yield from ph("orbit right", lambda r: self.p_orbit("orbit right", r, 1))
            yield from ph("orbit left", lambda r: self.p_orbit("orbit left", r, 0))
            yield from ph("route", lambda r: self.p_route("route", r))
            yield from ph("curve", lambda r: self.p_curve("curve", r))
            yield from ph("gusts", lambda r: self.p_gusts("gusts", r))
            yield from ph("slow flight", lambda r: self.p_slow("slow flight", r))
            yield from ph("turn at 1.5 x its floor", lambda r: self.p_slow_turn("turn at 1.5 x its floor", r, 1.5))
            yield from ph("turn at 1.3 x its floor", lambda r: self.p_slow_turn("turn at 1.3 x its floor", r, 1.3))
            yield from ph("turn at its floor", lambda r: self.p_slow_turn("turn at its floor", r, None))
        self.drop()  # (nothing flies it on from here)
        self.write()


class _Place:
    """A place as a state has it, for Test.runway_offsets."""

    def __init__(self, latitude_rad, longitude_rad):
        self.latitude_rad, self.longitude_rad = latitude_rad, longitude_rad


def write_summary():
    """SUMMARY.md: every kept result, a row an aircraft and load."""
    rows = []
    for path in sorted(glob.glob(os.path.join(RESULTS, "*.json"))):
        with open(path, encoding="utf-8") as f:
            r = json.load(f)
        failed = ["%s: %s%s" % (c["phase"], c["check"], " (%s)" % c["detail"] if c["detail"] else "") for c in r.get("checks", []) if not c["passed"]]
        rows.append("| %s | %s | %s | %s | %s | %s of %s | %s |" % (
            r["aircraft"], r["load"], r.get("date", ""), "%.0f" % r["mass_kg"] if r.get("mass_kg") and r["mass_kg"] >= 100 else r.get("mass_kg"),
            "%.0f" % r["mass_spawned_kg"] if r.get("mass_spawned_kg") and r["mass_spawned_kg"] >= 100 else r.get("mass_spawned_kg"),
            r.get("passed"), (r.get("passed") or 0) + (r.get("failed") or 0), "<br>".join(failed) or "-"))
    text = ("# Flight stability through the Vehicle Interface\n\n"
            "What `fsim testbench --headless stability.` last found, a row an aircraft and load (the suite:\n"
            "`src/applications/testbench/fsim_testbench/suites/stability.py`; each row's numbers: `<aircraft>_<load>.json` beside this file).\n\n"
            "| aircraft | load | flown | kg | kg as spawned | checks passed | failed |\n|---|---|---|---|---|---|---|\n" + "\n".join(rows) + "\n")
    with open(os.path.join(RESULTS, "SUMMARY.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _register(kind, group):
    @scenario("stability." + kind, suite="Stability", group=group, title="%s: stability at full load" % kind, refs="ADR-28; ADR-29",
              summary="A takeoff, then in the air: a level hold, heading, altitude and speed steps, an orbit each way, a route, a curve, "
                      "gusts and slow flight - every command through the Vehicle Interface, the aircraft at full load (or as it spawns). "
                      "Its numbers and checks are kept in src/applications/testbench/results/stability.",
              options=[choice("load", "Load", "full", ("full", "spawn"), help="full: every tank full and every station its loadouts load; spawn: as its design spawns it")])
    def run(t):
        yield from Flight(t, kind, t.opt("load")).run()
    return run


for _kind in WINGS:
    _register(_kind, "Stability: fixed wing")
for _kind in ROTORS:
    _register(_kind, "Stability: rotorcraft")
