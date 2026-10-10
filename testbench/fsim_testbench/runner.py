"""The runner: one simulation thread that owns the session's fsim.World and plays the scenarios run in it - one, or
several side by side - in real time (or faster), keeping a snapshot of what is happening for the window. The World is
published, so flightsim-viewer shows every aircraft in it; World.focus_viewer turns the viewer to one.

Scenarios see a Test (`t`): the world, the place they happen, helpers to spawn and command vehicles, and the waits they
yield. Each run in a session has a lane of its own - 28R or the parallel 28L, its starts in the air 4.5 km from the
others' - so runs added beside each other do not fly into each other. Nothing here knows a scenario: a suite is only a
module of them (suites/)."""
import collections
import gc
import math
import queue
import sys
import threading
import time
import traceback

import fsim

R = 6371008.8  # the platform's mean radius (core/Geodesy.h)
WORLD_NAME = "testbench"
SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 0.0)  # 0: as fast as it goes
LANE_SPACING_M = 4500.0  # between the lanes' starts in the air, north and south of the first
# The session world's clock: the hour at SFO (local solar time) the viewer lights it for - the testbench's runs are
# lit as at that hour whatever the wall clock says ("now": the wall clock's own)
CLOCKS = {"midday": 13.0, "morning": 9.0, "evening": 18.5, "night": 23.0, "now": None}
SFO_SOLAR_OFFSET_H = 122.375 / 15.0  # local solar time + this = UTC
# The session world's sky: the cloud the viewer draws (the world's weather: cover, base m, precipitation) - every run's,
# like the clock
SKIES = {"clear": (0.0, 2000.0, 0.0), "few": (0.2, 1500.0, 0.0), "scattered": (0.4, 1500.0, 0.0), "broken": (0.7, 1200.0, 0.0),
         "overcast": (0.95, 900.0, 0.0), "rain": (1.0, 600.0, 0.8)}
# Every aircraft's radars are on as it is created, as its crew would have them (Test.radars_on): the payload families
# that are radars. An air search radar also searches; a weather or ground-mapping radar is only powered (its modes come
# with MS-11; A-GRA has none for real-beam ground mapping)
RADAR_FAMILIES = ("air_radar", "weather_radar", "ground_map_radar")
SEARCH_BARS = 4  # a fighter's radar: its whole field of regard in azimuth, 4 bars about the horizon
ROTODOME_BARS = 2  # a rotodome: 360 deg, 2 bars stacked electronically
# The radar of the aircraft the viewer follows searches at the signal level (MS-6: each dwell through its samples and
# processing, the ground's clutter among them), so the viewer draws its range-Doppler map beside its B-scope or PPI;
# every other radar at the detection level. The signal level costs ~5 % of real time per radar at the testbench's
# 600 m over the bay (the detection level next to nothing): one radar at a time keeps a session at speed. Headless runs, with
# no viewer, search at the detection level alone (Runner.signal_level).


class Origin:
    """Where a scenario happens: a runway's threshold, its direction and its far end."""

    def __init__(self, name, threshold, far_end, airfield_id, runway_id, reverse_id):
        self.name = name
        self.lat0, self.lon0 = map(math.radians, threshold)
        lat1, lon1 = map(math.radians, far_end)
        north = (lat1 - self.lat0) * R
        east = (lon1 - self.lon0) * R * math.cos(self.lat0)
        self.length_m = math.hypot(north, east)
        self.direction_rad = math.atan2(east, north) % (2 * math.pi)  # true, threshold to far end
        self.airfield_id, self.runway_id, self.reverse_id = airfield_id, runway_id, reverse_id

    def parallel(self, name, left_m, length_m, runway_id, reverse_id):
        """The parallel runway `left_m` to its left, its threshold abeam this one's, `length_m` long."""
        d = self.direction_rad - 0.5 * math.pi
        lat = self.lat0 + left_m * math.cos(d) / R
        lon = self.lon0 + left_m * math.sin(d) / (R * math.cos(self.lat0))
        far = (lat + length_m * math.cos(self.direction_rad) / R,
               lon + length_m * math.sin(self.direction_rad) / (R * math.cos(self.lat0)))
        return Origin(name, (math.degrees(lat), math.degrees(lon)), tuple(map(math.degrees, far)), self.airfield_id, runway_id, reverse_id)


# San Francisco International's runway 28R (10L the other way): thresholds from the FAA's airport data, 3,618 m apart;
# 28L (10R) parallel to it 229 m (750 ft) south, 3,231 m long. In the airfield the vehicles carry, 28R is runway 28 and
# 10L runway 10; 28L is runway 29 and 10R runway 11.
SFO_28R = Origin("SFO 28R", (37.613534, -122.357141), (37.628714, -122.393519), 1, 28, 10)
SFO_28L = SFO_28R.parallel("SFO 28L", 229.0, 3231.0, 29, 11)
LANES = {"sfo": (SFO_28R, SFO_28L)}


def lane_offset_m(lane):
    """The north offset of a lane's starts in the air: 0, +4.5 km, -4.5 km, +9 km, ..."""
    return 0.0 if lane == 0 else LANE_SPACING_M * ((lane + 1) // 2) * (1.0 if lane % 2 else -1.0)


class Wait:
    """What a scenario yields: the runner resumes it with True when `ready` holds, False when `timeout_s` passes."""

    def __init__(self, ready, timeout_s, label):
        self.ready, self.timeout_s, self.label = ready, timeout_s, label
        self.start = None


class Expectation:
    def __init__(self, test, label):
        self.test, self.label = test, label
        self.state, self.detail = "pending", ""

    def set(self, ok, detail=""):
        self.state = "pass" if ok else "fail"
        self.detail = detail
        self.test.log(("PASS " if ok else "FAIL ") + self.label + (": " + detail if detail else ""))
        return ok


class Test:
    """What a scenario is given. Places are relative to its origin - its lane's runway: north and east of the runway's
    threshold (its lane's starts in the air shifted north or south), or along the runway (towards its far end) and
    across it (+ right)."""

    def __init__(self, runner, run):
        self.runner = runner
        self.run = run
        self.scenario = run.scenario
        self.options = run.options
        origins = LANES[run.scenario.origin]
        self.origin = origins[run.lane % len(origins)]
        self.lane_north_m = lane_offset_m(run.lane)
        self.world = runner.world
        self.vehicles = []
        self.activities = []  # (label, Activity)
        self.expectations = []
        self.metrics = []  # (label, fn)
        self.actions = []  # (label, fn)
        self.trackers = []  # fn() every 0.1 s of simulated time
        self.searches = []  # (vehicle, payload name): air search radars powered, their search not commanded yet
        self.searching = {}  # (vehicle name, payload name) -> [vehicle, activity id, signal level]: the searches commanded
        self.motion = {}  # vehicle name -> [path m, most ground speed m/s, most |bank| deg, start lat, start lon]
        self.SFO, self.RWY, self.RWY_REVERSE = self.origin.airfield_id, self.origin.runway_id, self.origin.reverse_id
        self.runway_name = self.origin.name
        self.scale = 1.0  # the geometry's, for the aircraft (fit)
        self._field = None

    # --- options, log, results ------------------------------------------------------------------------------------
    def opt(self, name):
        return self.options[name]

    def log(self, text):
        self.runner.log("#%d %7.1f  %s" % (self.run.number, self.world.time, text))

    def check(self, label, ok, detail=""):
        e = self.expect(label)
        return e.set(bool(ok), detail)

    def expect(self, label):
        e = Expectation(self, label)
        self.expectations.append(e)
        return e

    def metric(self, label, fn):
        """A live readout: fn() whenever the window is updated (a few times a second of wall time - not every step:
        what must see every moment, a touchdown or the worst of something, belongs in every())."""
        self.metrics.append((label, fn))

    def every(self, fn):
        """fn() every 0.1 s of simulated time, however fast it runs: for records a check reads later."""
        self.trackers.append(fn)

    def action(self, label, fn):
        """A button the user may press while it runs: fn() on the simulation thread."""
        self.actions.append((label, fn))

    def watch(self, activity, label):
        """Show an activity's state and progress; returns it."""
        self.activities.append((label, activity))
        return activity

    # --- waits ----------------------------------------------------------------------------------------------------
    def wait(self, seconds):
        return Wait(lambda: False, seconds, "%.0f s" % seconds)

    def until(self, ready, timeout_s, label=""):
        return Wait(ready, timeout_s, label)

    def until_done(self, activity, timeout_s, label=None):
        return Wait(lambda: not activity.live, timeout_s, label or "until it ends")

    # --- places ---------------------------------------------------------------------------------------------------
    def _offset(self, north_m, east_m):
        o = self.origin
        return (math.degrees(o.lat0 + north_m / R), math.degrees(o.lon0 + east_m / (R * math.cos(o.lat0))))

    def place(self, north_m=0.0, east_m=0.0):
        """(latitude_deg, longitude_deg) north and east of the origin, in this run's lane of the air."""
        return self._offset(north_m + self.lane_north_m, east_m)

    def place_rad(self, north_m=0.0, east_m=0.0):
        lat, lon = self.place(north_m, east_m)
        return math.radians(lat), math.radians(lon)

    def along(self, along_m, across_m=0.0):
        """(latitude_deg, longitude_deg) `along_m` down the runway from its threshold and `across_m` right of it."""
        d = self.origin.direction_rad
        north = along_m * math.cos(d) - across_m * math.sin(d)
        east = along_m * math.sin(d) + across_m * math.cos(d)
        return self._offset(north, east)

    def along_rad(self, along_m, across_m=0.0):
        lat, lon = self.along(along_m, across_m)
        return math.radians(lat), math.radians(lon)

    def runway_offsets(self, s):
        """A state's (along, across) the runway from its threshold, m."""
        o = self.origin
        north = (s.latitude_rad - o.lat0) * R
        east = (s.longitude_rad - o.lon0) * R * math.cos(o.lat0)
        d = o.direction_rad
        return north * math.cos(d) + east * math.sin(d), -north * math.sin(d) + east * math.cos(d)

    def heading_deg(self, offset_deg=0.0):
        """The runway's direction (+ offset), degrees true."""
        return (math.degrees(self.origin.direction_rad) + offset_deg) % 360.0

    def ground_m(self, lat_deg, lon_deg):
        h = self.world.terrain(math.radians(lat_deg), math.radians(lon_deg))
        return 0.0 if h is None else h

    @property
    def field_m(self):
        """The runway threshold's height above the ellipsoid."""
        if self._field is None:
            self._field = self.ground_m(math.degrees(self.origin.lat0), math.degrees(self.origin.lon0))
        return self._field

    def set_wind_across(self, speed_ms, from_left=True):
        """Wind straight across the runway, from its left (or right) - the session's wind, every run's."""
        if speed_ms > 0.0:
            frm = self.heading_deg(-90.0 if from_left else 90.0)
            self.world.set_wind(frm, speed_ms)
            self.log("wind %.0f m/s from %03.0f (for every run in the session)" % (speed_ms, frm))

    # --- vehicles -------------------------------------------------------------------------------------------------
    def performance(self, kind):
        """The aircraft's performance summary (min_cas_ms, cruise_tas_ms, hovers...), from a vehicle in a world of its own."""
        return self.runner.performance(kind)

    def cruise_ms(self, kind):
        """A speed it flies comfortably: a light aircraft's 1.4 times its least (60 m/s where that is under its cruise);
        a jet's up to 2.5 times its least, no faster than its cruise - at 1.4 times an F-15C flies 15 deg nose up (its
        angle of attack, which looks like a model pointing off its path); 0 for a rotorcraft (it starts in its hover)."""
        p = self.performance(kind)
        if p.hovers:
            return 0.0
        speed = max(1.4 * p.min_cas_ms, min(60.0, p.cruise_tas_ms))
        if speed > 60.0:
            speed = max(speed, min(p.cruise_tas_ms, 2.5 * p.min_cas_ms))
        return speed

    def turn_radius_m(self, kind, bank_deg=25.0):
        """The radius the aircraft turns at its comfortable speed and a planned bank (a route's 25 deg)."""
        v = self.cruise_ms(kind) or 15.0
        return v * v / (9.80665 * math.tan(math.radians(bank_deg)))

    def fit(self, kind):
        """The scale a scenario's geometry - drawn for a C172, which turns in about 660 m - takes for this aircraft: its
        turn radius over the C172's, 1 at least. A jet's legs, circles and corridors grow with it, so what fits a C172
        is not refused for a C-17A or flown through for an F-16C."""
        self.scale = max(1.0, self.turn_radius_m(kind) / 660.0)
        if self.scale > 1.0:
            self.log("geometry x%.1f for the %s (it turns in %.0f m)" % (self.scale, kind, self.turn_radius_m(kind)))
        return self.scale

    def spawn(self, kind, north_m=0.0, east_m=0.0, *, alt_m=None, agl_m=None, heading_deg=None, speed_ms=None, name=None,
              on_ground=False, airfield=True, at=None):
        """A vehicle of `kind` (an aircraft directory name, e.g. "c172"): at `at` (lat_deg, lon_deg) or north/east of the
        origin (in this run's lane); at `alt_m` above the ellipsoid or `agl_m` above the ground (default 600 m up);
        heading the runway's direction unless given; at its comfortable speed unless given. Its name is made unique in
        the session. The airfield is loaded into it."""
        lat, lon = at if at is not None else self.place(north_m, east_m)
        if heading_deg is None:
            heading_deg = self.heading_deg()
        if speed_ms is None:
            speed_ms = 0.0 if on_ground else self.cruise_ms(kind)
        if alt_m is None and not on_ground:
            alt_m = self.ground_m(lat, lon) + (600.0 if agl_m is None else agl_m)
        name = self.runner.claim_name(name or kind)
        v = self.world.create_vehicle(name, "jsbsim:" + kind, latitude_deg=lat, longitude_deg=lon, altitude_msl_m=alt_m,
                                      heading_deg=heading_deg, airspeed_ms=speed_ms, on_ground=on_ground or None)
        if airfield:
            self.load_airfield(v)
        self.vehicles.append(v)
        self.log("%s: %s at %.5f, %.5f%s" % (name, kind, lat, lon, " on the ground" if on_ground else ", %.0f m, %.0f m/s" % (alt_m, speed_ms)))
        if len(self.vehicles) == 1:
            self.runner.focus(v)  # (the viewer turns to a new run's first aircraft)
        self.radars_on(v)
        return v

    def radars_on(self, v):
        """Every radar the vehicle carries (its fit's) powered to operate; an air search radar then searches - a
        fighter's its whole field of regard in range-while-search, a rotodome its 360 deg in surveillance - out to its
        card's instrumented range. The viewer's radar windows (key p) show what each one sees. A scenario that wants a
        radar otherwise commands it after spawn: a newer command supersedes this one."""
        for p in v.payloads:
            if p.family not in RADAR_FAMILIES:
                continue
            try:
                if p.state != fsim.PayloadState.OPERATE:
                    v.set_payload_state(p.name, "operate")
            except fsim.MissionRejected as ex:
                self.log("%s: %s %s not powered (%s)" % (v.name, p.name, p.designation, ex.reason))
                continue
            if p.family == "air_radar":
                self.searches.append((v, p.name))
            else:
                self.log("%s: %s %s operating" % (v.name, p.name, p.designation))
        self.start_searches()

    def start_searches(self):
        """Command the searches of the radars powered by radars_on that now operate (through a warm-up, where a card
        gives one)."""
        waiting = []
        for v, name in self.searches:
            p = v.payload(name)
            if p is None or p.state in (fsim.PayloadState.OFF, fsim.PayloadState.FAILED):
                continue  # (turned off or failed meanwhile: left as it is)
            if p.state != fsim.PayloadState.OPERATE:
                waiting.append((v, name))
                continue
            self._search(v, name, self.runner.signal_level and v.name == self.runner.focused)
        self.searches = waiting

    def _search(self, v, name, signal_level):
        """The search radars_on gives a radar (a newer search of its own supersedes it); True if commanded."""
        p, s = v.payload(name), v.radar_status(name)
        try:
            if s.rotodome:
                a = v.amti_search(name, fsim.AmtiMode.SURVEILLANCE, bars=ROTODOME_BARS, range_scale_m=s.instrumented_range_m,
                                  signal_level=signal_level)
            else:
                a = v.amti_search(name, fsim.AmtiMode.RANGE_WHILE_SEARCH, azimuth_half_width_rad=s.azimuth_limit_rad,
                                  bars=SEARCH_BARS, range_scale_m=s.instrumented_range_m, signal_level=signal_level)
        except fsim.MissionRejected as ex:
            self.log("%s: %s %s search refused (%s)" % (v.name, name, p.designation, ex.reason))
            return False
        self.searching[(v.name, name)] = [v, a, signal_level]
        self.log("%s: %s %s searching, %s to %.0f km, %s level" % (
            v.name, name, p.designation, "360 deg" if s.rotodome else "+-%.0f deg" % math.degrees(s.azimuth_limit_rad),
            s.instrumented_range_m / 1000.0, "signal" if signal_level else "detection"))
        return True

    def follow_signal_level(self):
        """The followed aircraft's radar to the signal level, every other one back to the detection level - each
        search radars_on commanded that is still the radar's (one a scenario commanded itself is left as it is)."""
        for key, (v, a, signal) in list(self.searching.items()):
            want = self.runner.signal_level and v.name == self.runner.focused
            if want == signal:
                continue
            try:
                s = v.radar_status(key[1])
            except Exception:  # noqa: BLE001 - removed
                del self.searching[key]
                continue
            if s is None or not s.searching or s.activity != a:
                del self.searching[key]  # (stopped, or superseded by the scenario's own)
                continue
            self._search(v, key[1], want)

    def spawn_on_runway(self, kind, along_m=0.0, across_m=0.0, reverse=False, name=None):
        """Parked on the runway's centre line, `along_m` from its threshold, facing down it (or up it, `reverse`)."""
        lat, lon = self.along(along_m, across_m)
        v = self.spawn(kind, at=(lat, lon), on_ground=True, heading_deg=self.heading_deg(180.0 if reverse else 0.0), name=name)
        self.step_seconds(1.0)  # (settled on its wheels)
        return v

    def step_seconds(self, seconds):
        self.world.step(max(1, int(round(seconds / self.world.step_seconds))))

    def load_airfield(self, v):
        """SFO's 28R and 28L, both ways, into the vehicle's airfields (each run flies its lane's)."""
        runways = []
        for o in LANES[self.scenario.origin]:
            field = self.ground_m(math.degrees(o.lat0), math.degrees(o.lon0))
            lat1 = o.lat0 + o.length_m * math.cos(o.direction_rad) / R
            lon1 = o.lon0 + o.length_m * math.sin(o.direction_rad) / (R * math.cos(o.lat0))
            a = fsim.RunwayPoint(o.lat0, o.lon0, field)
            b = fsim.RunwayPoint(lat1, lon1, field)
            runways.append(fsim.Runway(o.runway_id, o.direction_rad, o.length_m, fsim.RunwayCoordinates(a, a, b), fsim.RunwayCoordinates(a, a, b)))
            runways.append(fsim.Runway(o.reverse_id, (o.direction_rad + math.pi) % (2 * math.pi), o.length_m, fsim.RunwayCoordinates(b, b, a),
                                       fsim.RunwayCoordinates(b, b, a)))
        v.load_airfield(fsim.Airfield(self.origin.airfield_id, "", fsim.HOLD, tuple(runways)))


def _name(x):
    return getattr(x, "name", str(x))


def _finite(x, digits=1):
    return None if x is None or not math.isfinite(x) else round(x, digits)


class Run:
    """One scenario run in the session."""

    def __init__(self, number, scenario, options, lane):
        self.number, self.scenario, self.options, self.lane = number, scenario, options, lane
        self.test = None
        self.gen = None
        self.wait = None
        self.status = "running"  # running, finished, failed, rejected (a command refused), error
        self.note = ""  # why a run that could not start did not


class Runner:
    def __init__(self, registry, offline=True):
        self.registry = registry
        self.offline = offline  # the terrain's tiles from the local map set alone, never fetched
        self.commands = queue.Queue()
        self.lock = threading.Lock()
        self.logs = collections.deque(maxlen=600)
        self.log_seq = 0
        self.world = None
        self.runs = []
        self.next_number = 1
        self.names = set()
        self.focused = None  # the vehicle name the viewer was last turned to
        self.signal_level = True  # its radar at the signal level (headless: False - no viewer, no range-Doppler map)
        self.paused = False
        self.speed = 1.0
        self.clock = "midday"
        self.sky = "scattered"
        self.snapshot = {"status": "idle", "runs": []}
        self._perf = {}
        self.thread = threading.Thread(target=self._loop, name="testbench-sim", daemon=True)
        self.alive = True

    def start(self):
        self.thread.start()

    def shutdown(self):
        self.alive = False
        self.commands.put(("quit",))
        self.thread.join(timeout=5.0)

    def log(self, text):
        with self.lock:
            self.log_seq += 1
            self.logs.append((self.log_seq, text))

    # --- what runs share ----------------------------------------------------------------------------------------------
    def claim_name(self, base):
        """A vehicle name not yet in the session: `base`, else base-2, base-3, ..."""
        name, k = base, 1
        while name in self.names:
            k += 1
            name = "%s-%d" % (base, k)
        self.names.add(name)
        return name

    def performance(self, kind):
        if kind not in self._perf:
            look = fsim.World("testbench-look", publish=False, workers=1, pin_workers=False, seed=1, terrain=False)
            v = look.create_vehicle(kind, "jsbsim:" + kind, latitude_deg=41.0, longitude_deg=0.0, altitude_msl_m=1000.0,
                                    heading_deg=0.0, airspeed_ms=60.0)
            self._perf[kind] = v.performance
            del v
            look.close()
        return self._perf[kind]

    def focus(self, vehicle):
        """Turn the viewer to a vehicle (a Vehicle or its name)."""
        if self.world is None:
            return
        if isinstance(vehicle, str):
            vehicle = next((v for r in self.runs if r.test for v in r.test.vehicles if v.name == vehicle), None)
            if vehicle is None:
                return
        try:
            self.world.focus_viewer(vehicle)
            self.focused = vehicle.name
        except Exception as ex:  # noqa: BLE001 - a vehicle gone
            self.log("could not turn the viewer: %s" % ex)
            return
        for r in self.runs:
            if r.test is not None:
                r.test.follow_signal_level()

    # --- commands from the server's threads -------------------------------------------------------------------------
    def submit(self, *command):
        self.commands.put(command)

    def _run_by_number(self, number):
        return next((r for r in self.runs if r.number == number), None)

    def _handle(self, command):
        kind = command[0]
        if kind == "run":
            self._start(command[1], command[2], add=len(command) > 3 and command[3])
        elif kind == "stop":
            self._close_session()
        elif kind == "stop_run":
            self._stop_run(self._run_by_number(command[1]))
        elif kind == "pause":
            self.paused = bool(command[1])
        elif kind == "speed":
            self.speed = float(command[1])
        elif kind == "focus":
            self.focus(command[1])
        elif kind == "clock":
            if command[1] in CLOCKS:
                self.clock = command[1]
                self._set_clock()
        elif kind == "sky":
            if command[1] in SKIES:
                self.sky = command[1]
                self._set_sky()
        elif kind == "action":
            run = self._run_by_number(command[1])
            if run and run.test and 0 <= command[2] < len(run.test.actions):
                label, fn = run.test.actions[command[2]]
                run.test.log("action: " + label)
                try:
                    fn()
                except Exception as ex:  # noqa: BLE001 - shown to the user
                    run.test.log("action failed: %s: %s" % (type(ex).__name__, ex))
        elif kind == "quit":
            self._close_session()

    # --- the session --------------------------------------------------------------------------------------------------
    def _close_session(self):
        # Every Vehicle and Activity holds the native world: the scenarios' frames, their Tests and their lambdas let
        # go of them before the world is closed, or it lives on - still published, and the next session's world, of
        # the same name, could not be (the viewer would keep showing the old one)
        runs, self.runs = self.runs, []
        for r in runs:
            if r.gen is not None:
                r.gen.close()
            r.gen = r.test = r.wait = None
        del runs
        world, self.world = self.world, None
        if world is not None:
            world.close()
        del world
        gc.collect()
        self.names.clear()
        self.focused = None

    def _stop_run(self, run):
        """One run out of the session: its scenario stopped, its vehicles removed."""
        if run is None:
            return
        if run.gen is not None:
            run.gen.close()
        if run.test is not None:
            for v in run.test.vehicles:
                try:
                    self.names.discard(v.name)
                    v.remove()
                except Exception:  # noqa: BLE001 - already gone
                    pass
            run.test.log("stopped; its aircraft removed")
        run.gen = run.test = run.wait = None
        self.runs.remove(run)
        gc.collect()
        if not self.runs:
            self._close_session()

    def _start(self, scenario_id, raw_options, add=False):
        sc = self.registry[scenario_id]
        options = {o.name: o.parse(raw_options.get(o.name)) for o in sc.options}
        if not add or self.world is None:
            self._close_session()
            with self.lock:
                self.logs.clear()
            self.world = fsim.World(WORLD_NAME, publish=True, workers=2, pin_workers=False, seed=7, terrain=sc.terrain,
                                    terrain_offline=self.offline)
            if not self.world.published:
                # (another world of this name still alive: the viewer would go on showing it, the old aircraft and all)
                self._close_session()
                raise RuntimeError("the new world could not be published for the viewer: a world named '%s' is still "
                                   "alive (the previous session's was not released)" % WORLD_NAME)
            self._set_clock()
            self._set_sky()
            self.paused = False
        lanes = {r.lane for r in self.runs}
        lane = next(k for k in range(len(self.runs) + 1) if k not in lanes)
        run = Run(self.next_number, sc, options, lane)
        self.next_number += 1
        self.runs.append(run)
        run.test = Test(self, run)
        run.test.log("%s - %s (%s), on %s%s" % (sc.id, sc.title, ", ".join("%s=%s" % kv for kv in options.items()) or "no options",
                                                run.test.runway_name, "" if lane == 0 else ", its air starts %+.1f km north" % (lane_offset_m(lane) / 1000.0)))
        try:
            run.gen = sc.run(run.test)
            self._advance(run, None)
        except Exception:  # noqa: BLE001
            self._error(run)

    def _command_failed(self, command, ex):
        """A command from the window that raised: logged with its traceback; a run that could not start is shown as a
        run in error, with the reason, rather than leaving the window on the last session."""
        self.log("ERROR: %s failed - %s: %s" % (command[0], type(ex).__name__, ex))
        for line in traceback.format_exc().rstrip().splitlines():
            self.log(line)
        if command[0] == "run" and command[1] in self.registry:
            run = Run(self.next_number, self.registry[command[1]], {}, 0)
            self.next_number += 1
            run.status = "error"
            run.note = "could not start: %s: %s" % (type(ex).__name__, ex)
            self.runs.append(run)

    def _set_clock(self):
        """The session world's epoch: today's date at the chosen hour at SFO, less the time already simulated."""
        if self.world is None:
            return
        hour = CLOCKS.get(self.clock)
        if hour is None:
            epoch = time.time()
        else:
            midnight = math.floor(time.time() / 86400.0) * 86400.0  # (today, UTC)
            epoch = midnight + (hour + SFO_SOLAR_OFFSET_H) * 3600.0
        self.world.set_environment(epoch_utc_seconds=epoch - self.world.time)

    def _set_sky(self):
        """The session world's cloud (the viewer draws it; the flight models do not feel it)."""
        if self.world is None:
            return
        cover, base, rain = SKIES[self.sky]
        self.world.set_environment(cloud_cover=cover, cloud_base_m=base, precipitation=rain)

    def _error(self, run):
        """A scenario that raised: a command the vehicle refused is the run's result - rejected, with the vehicle's own
        reason (an aircraft the scenario does not suit) - and anything else an error, its traceback logged. Either way
        the run ends and the session, and a headless batch, go on."""
        ex = sys.exc_info()[1]
        if isinstance(ex, fsim.Rejected) and run.test is not None:
            what = ex.reason + ("" if ex.index < 0 else " (item %d%s)" % (ex.index, "" if ex.constraint == "none" else ", " + ex.constraint))
            if ex.description:
                what += ": " + ex.description
            kinds = ", ".join(v.type.split(":")[-1] for v in run.test.vehicles) or "vehicle"
            run.test.expect("its commands accepted by the " + kinds).set(False, "refused " + what)
            run.status = "rejected"
            run.test.log("REJECTED: the vehicle refused a command - " + what)
        else:
            run.status = "error"
            for line in traceback.format_exc().rstrip().splitlines():
                self.log("#%d %s" % (run.number, line))
        if run.gen is not None:
            try:
                run.gen.close()
            except Exception:  # noqa: BLE001
                pass
        run.gen, run.wait = None, None

    def _advance(self, run, result):
        """Resume a run's scenario until it yields a wait that is not over yet, or ends."""
        while run.gen is not None:
            try:
                w = run.gen.send(result) if run.wait is not None or result is not None else next(run.gen)
            except StopIteration:
                run.gen, run.wait = None, None
                self._finish(run)
                return
            if not isinstance(w, Wait):
                raise TypeError("a scenario yields waits (t.wait, t.until, t.until_done), not %r" % (w,))
            w.start = self.world.time
            run.wait = w
            if w.ready():
                result = True
                continue
            return

    def _finish(self, run):
        t = run.test
        failed = [e for e in t.expectations if e.state == "fail"]
        pending = [e for e in t.expectations if e.state == "pending"]
        for e in pending:
            e.set(False, "never decided")
        run.status = "failed" if failed or pending else "finished"
        t.log("%s: %d of %d checks passed" % ("FAILED" if run.status == "failed" else "PASSED",
                                              sum(e.state == "pass" for e in t.expectations), len(t.expectations)))

    def _poll(self):
        for run in list(self.runs):
            w = run.wait
            if w is None or run.gen is None:
                continue
            try:
                self._track_motion(run.test)
                if run.test.searches:
                    run.test.start_searches()
                for fn in run.test.trackers:
                    fn()
                if w.ready():
                    self._advance(run, True)
                elif self.world.time - w.start >= w.timeout_s:
                    if w.label and w.label[0].isalpha():
                        run.test.log("timed out after %.0f s: %s" % (w.timeout_s, w.label))
                    self._advance(run, False)
            except Exception:  # noqa: BLE001
                self._error(run)

    @staticmethod
    def _track_motion(t):
        """How far each vehicle has gone and how hard it was flown: a run whose aircraft never moves shows it."""
        for v in t.vehicles:
            try:
                s = v.state
            except Exception:  # noqa: BLE001 - removed
                continue
            m = t.motion.get(v.name)
            if m is None:
                m = t.motion[v.name] = [0.0, 0.0, 0.0, s.latitude_rad, s.longitude_rad, s.latitude_rad, s.longitude_rad]
            step = math.hypot((s.latitude_rad - m[5]) * R, (s.longitude_rad - m[6]) * R * math.cos(s.latitude_rad))
            m[0] += step
            m[1] = max(m[1], math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1]))
            m[2] = max(m[2], abs(math.degrees(s.euler_rad[0])))
            m[5], m[6] = s.latitude_rad, s.longitude_rad

    @property
    def status(self):
        if not self.runs:
            return "idle"
        states = [r.status for r in self.runs]
        if "running" in states:
            return "running"
        return "error" if "error" in states else "failed" if "failed" in states or "rejected" in states else "finished"

    # --- the loop -----------------------------------------------------------------------------------------------------
    def _loop(self):
        last = time.monotonic()
        last_snap = 0.0
        debt = 0.0
        while self.alive:
            while True:
                try:
                    command = self.commands.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._handle(command)
                except Exception as ex:  # noqa: BLE001 - reported; the loop (and the window) go on
                    self._command_failed(command, ex)
            now = time.monotonic()
            wall = min(now - last, 0.25)
            last = now
            if self.world is not None and not self.paused:
                try:
                    dt = self.world.step_seconds
                    poll = max(1, int(round(0.1 / dt)))  # the scenarios' waits are looked at every 0.1 s
                    if self.speed > 0.0:
                        debt += wall * self.speed
                        steps = int(debt / dt)
                        debt -= steps * dt
                        while steps > 0 and self.world is not None:
                            n = min(steps, poll)
                            self.world.step(n)
                            steps -= n
                            self._poll()
                    else:
                        until = time.monotonic() + 0.03
                        while time.monotonic() < until and self.world is not None:
                            self.world.step(poll)
                            self._poll()
                except Exception:  # noqa: BLE001
                    self.log("the world failed:")
                    for line in traceback.format_exc().rstrip().splitlines():
                        self.log(line)
                    self._close_session()
            else:
                debt = 0.0
            if now - last_snap > 0.15:
                last_snap = now
                self._snapshot()
            time.sleep(0.004 if self.speed == 0.0 else 0.012)
        self._close_session()

    def _snapshot(self):
        snap = {"status": self.status, "paused": self.paused, "speed": self.speed, "clock": self.clock, "sky": self.sky,
                "time": round(self.world.time, 1) if self.world else None, "focused": self.focused,
                "runs": [], "vehicles": [], "activities": []}
        for run in self.runs:
            t = run.test
            r = {"run": run.number, "scenario": run.scenario.id, "title": run.scenario.title, "status": run.status,
                 "runway": t.runway_name if t else "", "lane": run.lane, "checks": [], "metrics": [], "actions": [], "wait": None,
                 "note": run.note}
            if t is not None and self.world is not None:
                try:
                    for v in t.vehicles:
                        x = self._vehicle(t, v, v.state)
                        x["run"] = run.number
                        snap["vehicles"].append(x)
                    for label, a in t.activities:
                        x = self._activity(label, a)
                        x["run"] = run.number
                        snap["activities"].append(x)
                    for label, fn in t.metrics:
                        try:
                            value = fn()
                        except Exception as ex:  # noqa: BLE001
                            value = "(%s)" % type(ex).__name__
                        r["metrics"].append({"label": label, "value": value if isinstance(value, str) else _finite(value, 2)})
                    r["actions"] = [label for label, _ in t.actions] if run.status == "running" or t.actions else []
                    if run.wait is not None and run.gen is not None and not run.scenario.interactive:
                        r["wait"] = {"label": run.wait.label, "elapsed": round(self.world.time - run.wait.start, 1),
                                     "timeout": run.wait.timeout_s}
                except Exception as ex:  # noqa: BLE001 - a vehicle removed mid-snapshot
                    r["note"] = "%s: %s" % (type(ex).__name__, ex)
                r["checks"] = [{"label": e.label, "state": e.state, "detail": e.detail} for e in t.expectations]
                r["motion"] = [{"name": k, "path_km": round(m[0] / 1000.0, 2), "gs_max": round(m[1], 1), "bank_max": round(m[2])}
                               for k, m in t.motion.items()]
            snap["runs"].append(r)
        with self.lock:
            snap["log"] = list(self.logs)
            self.snapshot = snap

    @staticmethod
    def _vehicle(t, v, s):
        along, across = t.runway_offsets(s)
        level = None
        try:
            level = _name(v.commanded.top_level)
        except Exception:  # noqa: BLE001
            pass
        return {"name": v.name, "type": v.type.split(":")[-1], "lat": round(math.degrees(s.latitude_rad), 6),
                "lon": round(math.degrees(s.longitude_rad), 6), "alt": _finite(s.altitude_msl_m), "agl": _finite(s.altitude_agl_m),
                "cas": _finite(s.airspeed_calibrated_ms), "tas": _finite(s.airspeed_true_ms),
                "gs": _finite(math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1])), "vs": _finite(-s.velocity_ned_ms[2]),
                "hdg": _finite(math.degrees(s.euler_rad[2]) % 360.0), "pitch": _finite(math.degrees(s.euler_rad[1])),
                "roll": _finite(math.degrees(s.euler_rad[0])), "g": _finite(s.load_factor, 2), "gear": _finite(s.gear_position, 2),
                "flaps": _finite(math.degrees(s.flaps_rad)), "throttle": _finite(s.throttle_position[0], 2),
                "fuel": _finite(s.fuel_kg, 0), "ground": bool(s.on_ground), "crashed": bool(s.crashed), "diverged": bool(s.diverged),
                "along": _finite(along, 0), "across": _finite(across, 1), "level": level, "runway": t.runway_name,
                "radar": Runner._radar(v)}

    @staticmethod
    def _radar(v):
        """Its radars, in a few words each: designation, state, and an air search radar's mode and tracks."""
        out = []
        for p in v.payloads:
            if p.family not in RADAR_FAMILIES:
                continue
            text = "%s %s" % (p.designation, _name(p.state).lower())
            if p.family == "air_radar" and p.state == fsim.PayloadState.OPERATE:
                s = v.radar_status(p.name)
                if s is not None and s.searching:
                    text = "%s %s%s, %d track%s" % (p.designation, _name(s.mode).lower().replace("_", " "), " (signal)" if s.signal_level else "",
                                                   s.tracks, "" if s.tracks == 1 else "s")
            out.append(text)
        return "; ".join(out)

    @staticmethod
    def _activity(label, a):
        info = a.info
        if info is None:
            return {"label": label, "id": "%#x" % a.id, "state": "forgotten"}
        out = {"label": label, "id": "%#x" % a.id, "state": _name(info.state), "reason": _name(info.reason),
               "source": _name(info.source)}
        p = a.progress
        if p is not None:
            out.update({"segment": None if p.segments in (0, None) else "%d/%d" % (p.segment, p.segments),
                        "percent": _finite(p.percent), "to_go_m": _finite(p.distance_to_go_m, 0),
                        "to_go_s": _finite(p.time_to_go_s, 0), "xtrack": _finite(p.cross_track_m)})
        return out
