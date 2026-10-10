# testbench

A window for testing the platform's features by eye: choose a scenario, set its options, run it, and watch the
aircraft in flightsim-viewer while the window shows its checks, its vehicles' state and its activities as they fly.

## How to open it

**Double-click `Testbench.cmd`** in the repository root (`D:\Projects\Rust\DecisionDRLTest\Testbench.cmd`), or the
**"flightsim testbench"** shortcut on the desktop, which runs it.

1. Two windows open: the testbench, and the viewer beside it.
2. In the testbench, pick a scenario on the left (the groups FA-1 to FA-10 open when clicked), set its options, and
   press **Run (new session)**.
3. For more aircraft at once, pick another scenario (or the same one, or **Sandbox** for an aircraft you fly with
   buttons) and press **Add to session**: it runs beside what is already flying, in the same world - on 28L where the
   first is on 28R, its starts in the air 4.5 km north or south of the others'. Add as many as you like; each run's
   **Stop run** removes it and its aircraft; **End session** removes them all.
4. Every aircraft is in the viewer. Choose which one it follows with **Viewer follows** at the top of the testbench,
   or **View** on a row of the Vehicles table - at any time; `tab` in the viewer still works too. A new run's first
   aircraft is followed as it appears.
5. The Session panel shows each run's checks passing or failing, its readouts and its buttons (reject the takeoff, go
   around, ...); the Vehicles and Activities tables show every aircraft's state and every activity, by run. The speed
   box runs it faster; **Time of day** and **Sky** (clear to overcast and rain; scattered cumulus to begin with) set
   how the viewer lights the session and the cloud it draws - the flight models feel neither.
   Every aircraft's radars are on from the moment it appears (ADR-30's payloads, from its fit): a fighter's radar
   searches its whole field of regard in range-while-search, an AEW rotodome sweeps 360 deg, both out to the card's
   instrumented range; a weather or ground-mapping radar is powered (its modes come later). The Vehicles table's
   **Radar** column shows each one's mode and tracks; in the viewer `p` shows their windows (B-scope / PPI). The
   radar of the aircraft the viewer follows searches at the signal level, so its range-Doppler map is drawn too;
   the others at the detection level (the signal level costs ~5 % of real time per radar, the detection level next
   to nothing). Choose the aircraft with **View** or **Viewer follows**: the viewer's own `tab` does not tell the
   testbench, so the map stays with the aircraft the testbench last turned it to. Headless runs search at the
   detection level alone.
6. Close the testbench window to stop: the viewer closes with it, and the minimised console goes by itself.

From a terminal it is `fsim testbench` with the same options:

```
fsim testbench                         the window and the viewer
fsim testbench --online                also download map tiles the local map set lacks (by default nothing is fetched)
fsim testbench --run fa10.recovery     start a scenario at once
fsim testbench --list                  the scenarios
fsim testbench --headless fa9.,fa10.   run scenarios as fast as they go, no window; exit 1 if any fails
```

`Testbench.cmd` takes them too (`Testbench.cmd --run tactics.bvr`). It needs the project built once (`fsim build`); it says
so if it is not. The window is a Microsoft Edge app window (no address bar or tabs) on a page the testbench serves on
127.0.0.1; nothing is installed and nothing leaves the machine. Without Edge the default browser opens the page.

## What it covers

Every completed Flight Autonomy stage (ADR-29, [docs/flight-autonomy.md](../docs/flight-autonomy.md)), at San
Francisco International: its runway 28R for takeoffs and landings, the bay east of it for everything in the air.

| Group | Scenarios |
| --- | --- |
| FA-1 Discovery and support | the support table, a refusal with its reason |
| FA-2 Commands and activities | HSA with live UPDATEs, ranks and start windows, stored tasks and suggestions |
| FA-3 Performance and energy | navigation report and playtime, long-range cruise against maximum endurance, a route beyond the fuel, the performance profile |
| FA-4 Terrain, altimeter, frames, magnetic | a route into San Bruno Mountain, flying an isobar while the QNH changes, a magnetic heading, a route in a moving frame |
| FA-5 Patterns and curves | orbit (laps, exit point), racetrack and figure eight, hold (each entry), a rotorcraft's hover, an orbit round a moving ship, Bezier and NURBS curves |
| FA-6 Routes | fly-by and fly-over, turn points, loiter points, segment performance, required time of arrival, planned states, RNP, paths and branches, RF legs |
| FA-7 Route plans | upload, activation, supersession; validation in a wind |
| FA-8 Must fly, marshall, intercept | a point from a window of bearings, another aircraft, a zone, a corridor, a volume, a three-aircraft stack, each intercept method |
| FA-9 Ground operations | takeoff (any wing, any crosswind), a rotorcraft's lift, the rejected takeoff (CANCEL, button, engine failure), the taxi, a route from the ground |
| FA-10 Recovery and landing | the recovery to 28R with CleanUp, DirtyUp and go-around buttons, the crosswind wave-off, speedbrakes, placards, a route that lands, aerobatics given up and recovered, the U-2S, a rotorcraft's recovery |
| Platform | flown into a mountain (the terrain strike), started inside one, a sandbox to fly by hand |
| Tactics: BVR | two fifth-generation fighters (F-22A, F-35A, J-20A, Su-57) beyond visual range over the Pacific, flown by temporary decision rules standing in for OMS: radar against the other's stealth, IRST, warning receivers; notional missiles (boost, loft, datalink, seeker, notch, fuse) shot inside their reach as the target flies; crank, notch, drag, abort, egress; nobody shot down - a hit scores the kill, "knock it off", both separate, rearm and refuel, and the next set begins, until stopped |

## Adding a suite (OMS, P2P, C2, ...)

A suite is a module in `testbench/fsim_testbench/suites/`; every module there is loaded at start and its scenarios appear in the
window. Copy `suites/_template.py` to `suites/<system>.py` and write scenarios: each is a generator given a `Test`
(`fsim_testbench/runner.py`) that spawns vehicles, commands them, declares readouts, buttons and checks, and yields waits.
Nothing else changes - the window, the runner and the headless mode take any scenario.

## How it works (testbench/fsim_testbench/)

- `runner.py` - one simulation thread owns the session's `fsim.World` (published as `testbench`, so the viewer
  attaches to it), steps it in real time or faster, advances each run's scenario when its wait is over, gives each run
  a lane (its runway, its stretch of sky), turns the viewer with `World.focus_viewer`, and keeps a snapshot for the
  window.
- `server.py` - the page and a small JSON API (`/api/catalog`, `/api/status`, `/api/run` (`add`: beside the session),
  `/api/stop`, `/api/stop_run`, `/api/focus`, `/api/pause`, `/api/speed`, `/api/clock`, `/api/sky`, `/api/action`, `/api/viewer`), standard
  library only.
- `ui/` - the page: plain HTML, CSS and JavaScript, no dependencies.
- `registry.py` - scenarios, suites and their options.
