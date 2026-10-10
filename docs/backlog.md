# Backlog

Findings measured and not yet fixed, across the platform: what the stability suite, the mass accounting and the
viewer work of 2026-10-10 left open (commit a05c5ef). It continues the numbering of
[flight-autonomy.md](flight-autonomy.md)'s active backlog (B-1 to B-18, all closed there) and keeps its rule: each
item says where it was measured and what closes it, and none is dropped because it lies outside the step that found
it.

Priority: **1** an aircraft is lost or a number is wrong in normal use; **2** a limit a user will meet; **3** a gap in
fidelity or tooling that nothing depends on yet.

## Flight: what the stability suite still finds

All at full load, through the Vehicle Interface (`testbench/results/stability/<aircraft>_full.json`).

| Item | Finding (where measured) | Area | P | Closed when |
| --- | --- | --- | --- | --- |
| B-19 | **The Gripen departs in a sustained orbit at full load.** 13,092 kg (tanks, missiles, pod), a 45 deg orbit at its cruise speed: its sideslip grows to 68 deg and it leaves its circle by 412 m (right) and 932 m (left). At 175 m/s it flew the same orbit (the first survey); as it spawns it does not depart. Its stores' Munk moment costs directional stability its yaw law does not make up. | flight control, hangar (gripen) | 1 | both orbit phases pass at full load - or its design declares the load it is cleared to and the store management refuses more |
| B-20 | **The B-52H's height swings after a launch at 221 t**: 86 m peak to peak, undamped, from two minutes after its takeoff completes. | flight control (launch, position loop) | 2 | "steady after its launch" passes (20 m) |
| B-21 | **The Mirage 2000's height swings after a launch at full load**: 29 m. | flight control | 3 | the same check passes |
| B-22 | **A loaded fighter's roll overshoots the bank it is held to.** F-16C: 70 deg where 60 is commanded (turn at 1.5 x its floor) and 61 where 51 is (1.3 x) - the bank it would stall at there; F-22A: 69 for 60. For 2 to 3 s. Its attitude loop is designed at the inertia it spawns with; tanks and missiles on its wings slow its roll's answer. | autopilot design (hangar autopilot stage), ControlStack | 2 | the roll loop is scheduled with the stores' inertia (or designed at full load too): overshoot within 5 deg at both loads |
| B-23 | **The U-2S does not settle on a heading at its least speed, full of fuel**: after a 60 deg turn at 15 deg of bank its heading swings +-2 deg and its sideslip +-2 deg, dying away over a minute; not within 1.5 deg for 15 s in 300 s. A Dutch roll its yaw damper leaves lightly damped there. | hangar (u2s), yaw law | 3 | "turned, its height kept" passes at its floor |
| B-24 | **After a speed-floor recovery it comes back past its altitude and is slow to settle.** The loaded Mirage 2000 rose 74 m above its altitude as its afterburner came in, and took over two minutes to come back within 15 m (DEVIATING shown meanwhile). The vertical-speed loop's integral winds up while protection holds its climb. | ControlStack (velocity loop anti-windup) | 2 | the integral is held while the energy rule limits the climb: back within 15 m in 30 s, no DEVIATING on the way up |
| B-25 | **The speed floor works through the throttle alone.** With no thrust to give (a C172, its tanks dry, commanded its least speed) the aircraft is not made faster: it mushes at 24.8 m/s, under its 25.4 m/s stall, sinking, flagged LOW_ENERGY and DEVIATING (`python/tests/test_load_limits.py`). | Protection (energy rule) | 2 | below its floor with its throttle at its stop it trades height for speed (a descent to the floor's airspeed), and holds the floor |
| B-26 | **Beyond their heaviest row the performance tables are extended in a straight line**, up to 60 % of that weight (`kBeyondHeaviest`). A fully loaded F-16C is 38 % beyond, 1.6 times the span of its three rows; the tables' least level speeds fall with weight on several fighters, so the extension runs the wrong way for that column. A profile at full load is an estimate. | hangar performance, PerformanceTables | 2 | hangar flies a row at a loaded weight (its heaviest loadout), or the extension is by weight ratio for each column's physics |
| B-27 | **An angle-of-attack limit past the wing's departure.** The RQ-4B drops a wing at 14 to 16 deg, its limit 19.5; the E-7A reached 26 deg at its stall speed. Commanded their stall speed (the Vehicle Interface's least, before the 1.1 margin) both left controlled flight: the RQ-4B to the ground. The margin keeps commands away; the limits are still wrong. | hangar (profile: alpha_max) | 2 | each design's alpha_max is where its model departs, less a margin, checked by a slow-flight test at its stall speed with protection on |

## hangar: flown again, and what would not fly

| Item | Finding (where measured) | Area | P | Closed when |
| --- | --- | --- | --- | --- |
| B-28 | **The F-35A's and the Su-57's performance stage ends `command refused: crashed`** (`performance.py`, its table flights), the F-35A's on its design as committed too. Their profiles are still those flown with their bay missiles as fixed load: 608 and 972 kg heavier than they spawn (2.7 and 3.3 %). | hangar performance | 2 | the stage flies both through; their tables and least speeds are at their spawn weight |
| B-29 | **The tables' ceiling 5.2 % under the flight test's** on the EA-18G and the F-22A once flown lighter (55,220 ft against 58,226; 58,320 against 61,486): the check's band is 5 %. With their stores as fixed load they agreed (+0.1 %). | hangar performance | 3 | the two measures agree within the band, or the reason they do not is found and the check says it |
| B-30 | **A design built again does not give the committed files.** hangar run on the F-35A as committed, nothing changed, rewrote its flight model and autopilot (its pitch gain 9.35 to 10.38): the generated files in the repository are not what the tool makes today. | hangar (reproducibility), CI | 2 | a CI job rebuilds every design and compares; the differences are understood or the files regenerated |
| B-31 | **The EA-18G's main gear bays are drawn open from below with its gear up** (the viewer, its model before and after its pods left it). | hangar model3d (gear doors) | 3 | its doors close over the wells |

## Stores and mass

| Item | Finding (where measured) | Area | P | Closed when |
| --- | --- | --- | --- | --- |
| B-32 | **The B-52H, the H-6K and the UH-60 carry nothing**: their store-station cards list no stations, so a bomber's "full load" is its fuel. | Mission Systems cards, hangar carriage | 1 | each has its stations and at least one loadout of stores with cards and models (the B-52H's bay and pylons, the H-6K's pylons, the UH-60's stub wings) |
| B-33 | **A gun's ammunition never leaves**: 60 to 820 kg of fixed load that does not fall as the gun fires. | Mission Systems (guns), mass | 3 | firing draws the ammunition's point mass down by the rounds fired |
| B-34 | **Cargo falls as a box.** No parachute, no extraction system, no requirement on the ramp or the aircraft's attitude; a card's mass is one figure (no load that varies). The two cards' sizes and limits are from memory, marked assumed. | stores (cargo), store cards | 2 | a drogue and main canopy as drag that opens; the cards' figures read from their sources; a card's load settable within its limit |
| B-35 | **Nothing holds a load to a maximum weight.** The C-17A with full fuel and eighteen pallets is 276.8 t in the stability suite, above the 265 t published for it; its design declares no `mtow`, and the store management checks a station's limit, not the aircraft's. | hangar designs (`mtow`), Mission Systems | 2 | every design declares its maximum take-off weight; loading beyond it is refused or flagged |
| B-36 | **Air-to-air missiles are bodies, not weapons.** A strike with one is refused (`store_not_flyable`); jettisoned it falls inert. Made on its own (`create_vehicle("jsbsim:aim120c")`) its motor is lit from the first step. Their powered flight, guidance and fuzes (MS-7c, MS-7d) were stopped earlier and are an owner's decision to take up. | Mission Systems | - | decided by the owner |
| B-37 | **Stores have no radar cross section** (only the Mk 82 has a table): a Growler's pods, a fighter's tanks and missiles add nothing to what a radar sees; the EA-18G's own table is now its clean airframe's. | tools/rcs, Mission Systems | 2 | each store's table computed and added at its station, or the loaded aircraft's computed |
| B-38 | **What counts as fixed equipment** was decided without the owner: the U-2S's nose and Q-bay sensors and the RQ-4B's suite are `equipment` (not stores); gun ammunition and the C172's luggage are very small items. | owner's decision | - | confirmed, or the sensors made stores |
| B-39 | **The ALQ-99 pod's model is the generic pod's** (a body of revolution from its length and diameter): the ram-air turbine at its nose that the airframe's own pods had is gone. | hangar stores | 3 | built by the vehicles' pipeline from a design of its own (in hand: see below) |

## Interfaces, viewer, tests

| Item | Finding (where measured) | Area | P | Closed when |
| --- | --- | --- | --- | --- |
| B-40 | **`sdk_boundary` fails**: `docs/sdk/boundary.md` is out of date (`tools/boundary/boundary.py render`), on the tree before this work too. | docs, CI | 2 | rendered and committed; the test passes |
| B-41 | **The load's limits are not in the C ABI.** `Performance::leastCasMs()`, `loadSpeedRatio` and `loadKg` exist in C++; Python works `least_cas_ms` out as 1.1 times `min_cas_ms` itself. | C ABI, Python | 2 | `fsim_performance` carries them (struct_size-gated), Python reads them |
| B-42 | **A replay shows no flight status**: a recording keeps each vehicle's state and stores, not its Vehicle Interface's values. | ipc Recording, viewer | 3 | the sample's `derived` values recorded and replayed |
| B-43 | **The monitor panel scrolls in a small window** at a high DPI scale (1600 x 900 at 150 %): its last lines are below its edge. As before this work; the flight status was put above the vehicle list because of it. | viewer (MonitorGui layout) | 3 | the panel fits, or folds its sections |
| B-44 | **DEVIATING is raised by a recovery's own overshoot** (B-24): true, and not what an operator needs told. | CapabilityHost | 3 | closed with B-24 |
| B-45 | **The stability suite keeps only the full-load results.** The as-spawned run (`--option load=spawn`) is not kept beside them, so a change that hurts the clean aircraft shows only in ctest. | testbench | 3 | both loads' results kept and summarised |
