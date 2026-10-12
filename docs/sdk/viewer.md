# Transparent visualisation

`flightsim-viewer.exe` is a separate, prebuilt program. It finds your world
by itself, shows it, and follows every change. Your training application
contains no visualisation code and does not know whether a viewer is running.

## What you do

Nothing. Create a `World` (with `publish = true`, the default) and step it. To turn the viewer to a vehicle from
your program, `World::focusViewer(id)` (Python `world.focus_viewer(vehicle)`, C `fsim_world_focus_viewer`): it
follows that vehicle as a `tab` to it would, and the user's choice stands after it.
Start the viewer whenever you like, before or after the trainer, on the same
machine:

```bash
build/ucrt64-release/bin/flightsim-viewer.exe              # attach to the newest published world
build/ucrt64-release/bin/flightsim-viewer.exe --world my-experiment
build/ucrt64-release/bin/flightsim-viewer.exe --list       # print live worlds
```

The viewer is a GUI application: no console window appears when it is
started from Explorer (its log then goes to `logs\viewer.log` beside the program);
started from a terminal it prints there (`--list`, `--help`, `--stats`). `--stats <s>` prints each second's frame
rate and frame-time breakdown, the GPU's time per pass, and the second's worst frame broken down, with how many frames
ran over 1.5 times the last second's mean (late: a missed refresh). Measure fullscreen (`--fullscreen`), the size it is
used at.

Until a world exists the window says "waiting for a training application";
when your program starts, the vehicles appear with their names, types and
active control levels; when it exits, the viewer says so and waits for the
next one. Restarting the trainer re-attaches automatically. A world closed
and made again under the same name in the same process (the testbench's
Stop, then Start) is attached afresh the moment it is: everything shown of
the old one - its aircraft, their trails, the focus - is dropped.

## What the viewer shows

- Every live vehicle, as a 3D model over full-Earth satellite imagery (to level 19, ~0.3 m/px) and terrain (30 m data to level 15, smoothly refined below it so the imagery keeps sharpening at low altitude), with a label, a trail and a chase/orbit/overview camera (`tab` cycles vehicles; left drag rotates the view around its focus (middle drag too); space + left drag pans - detached (camera mode "free", or before any vehicle exists) it moves the globe so the ground follows the cursor, including across the poles, and locked to a vehicle the camera only rotates about it; the wheel zooms from 6 m to the whole Earth and changes nothing but the distance (`--zoom-to-cursor` for osgEarth's zoomToMouse); space alone, released without a drag, pauses; the right button is not a camera control; the camera never enters the terrain).
- Vehicle creation, reset (new "generation") and removal, as they happen.
- Several training applications at once: the monitor lists every live world and switches between them (`--world <name>` picks one at start).
- Per-type models: a vehicle of type `jsbsim:f16` is drawn with `models/f16.glb` (or `.gltf`) if such a file exists in an asset directory (`<exe>/../share/flightsim/models`, the source tree's `assets/models`, or a `--assets` path), else with the platform's default aircraft; `VehicleSpec::model` names a file explicitly. Files are loaded once, on first use, and shared by every vehicle of that type; a `<file>.manifest` (`forward`, `up`, `scale`) fixes axes and size. Aircraft made with [hangar](../hangar.md) carry their own model (`aircraft/<name>/<name>.glb`), found the same way - and so does every store (a missile, a bomb, a tank, a pod, cargo: each a design of its own, [hangar.md](../hangar.md#stores)).
- Moving control surfaces, for models that mark them (see below): ailerons, elevator, rudder and flaps follow each vehicle's deflections, from the trainer or the replay.
- What the engines and the air do (see [Effects](#effects)): afterburner flames, glowing nozzles and hot exhaust; wing tip vortices; condensation on the airframe - over the wings and canards, in the leading-edge extensions' vortices - and the transonic vapour cone; contrails high up.
- The world's clock (sun position and sky), wind, atmosphere and weather - its cloud drawn as volumetric clouds (see [Clouds](#clouds)).
- The trainer's throughput (vehicle-steps/s), simulation time and the age of the last update.
- Per vehicle: state summary and the active control level.
- What each vehicle's Vehicle Interface says of it (see [Flight status](#flight-status)): the selected vehicle's weight, its airspeed against the least a command may ask and its stall at that weight, its speed floor and the bank its airspeed leaves it; and for every vehicle, in the list and on its label, that it is low on energy, deviating or upset.
- The trainer's cameras (`fsim_vision`, [vision.md](vision.md)): a "cameras" window with every image the training application rendered last, live (`v` toggles it).
- The followed vehicle's Mission Systems ([mission-systems.md](mission-systems.md#in-the-viewer)): its payloads and their states, and each radar's display (a B-scope, or a PPI for a rotodome) with its scan, detections and tracks - the one it follows marked, its acquisition and illumination named - as the SDK's instances have them (`p` toggles them; each window collapses).

## How it works (design 9.7)

The `World` maps one named shared-memory segment (`Local\fsim.world.<name>`)
holding a header, a vehicle table and three snapshot slots, and registers the
name in a tiny registry segment for discovery. After each `step()` it copies
the vehicle states into the next slot **if at least `publishIntervalSeconds`
of wall time have passed** (16 ms by default), behind a sequence counter. That
is the whole cost: a bounded `memcpy` at most ~60 times per second, no locks,
no waiting, no allocation, no notice of readers. A trainer running 5,000x
real time publishes at 60 Hz, not at 600 kHz; a trainer that pauses is simply
shown as "not updating".

The viewer reads slots under a seqlock (retrying if a copy is overwritten
while it reads), interpolates between the two newest snapshots in wall time
so motion is smooth at any simulation pace, and re-reads the vehicle table
whenever its generation counter changes.

Measured on an 8-core desktop with 64 vehicles: 760k vehicle-steps/s alone,
within a few percent with a viewer attached (the difference is the viewer's
own rendering competing for CPU, not the publishing; the viewer is paced by
vsync and uses ~3% of one core while mirroring 64 vehicles).

## Options that matter to a trainer

| `WorldOptions` | Effect |
| --- | --- |
| `name` | what the viewer lists and attaches to; use a distinct name per concurrent experiment |
| `publish = false` | no segment at all (e.g. headless sweeps on a cluster) |
| `publishIntervalSeconds` | the copy rate limit; raise it to reduce the copy further, lower it for smoother motion at real-time pace |
| `capacity` | vehicle slots in the segment (256 by default); vehicles beyond it simulate but are not shown |
| `VehicleSpec::model` | optional glTF path shown instead of the type's model (`models/<type>.glb`, a design's `aircraft/<type>/<type>.glb`, or the design registered to stand in for a stock aircraft in `aircraft/models.txt`) or the default aircraft |
| `recordPath` | also write the run to a `.fsrec` file; `flightsim-viewer.exe --replay <file>` plays it back (space pauses, `.` steps one frame, `[` `]` change the time factor, the timeline slider seeks, `home` restarts, loops at the end). The recording carries the world's environment as it changes (its `E` records, format 4) - the replay's clouds, the air its contrails, smoke and vapour form in and the sun by its clock are the world's, as live; a recording made before has none and is drawn in the viewer's defaults (a typical day's humidity, no clouds but its own, no wind) |

## Moving control surfaces

A glTF node named `fsim:<channel>` turns with the vehicle's deflection of that
channel. The channel is one of:

- `aileron` - the left aileron's position; give the right one a gain of `-1`,
  or turn its axis the other way;
- `elevator`;
- `rudder`;
- `flaps` (`flap` works too).

The node turns about its own x axis, by the deflection in radians. JSBSim's
sign conventions apply: elevator and flaps trailing edge down, aileron left
trailing edge down, rudder trailing edge left. An optional gain scales the
deflection: `fsim:aileron:-1`, `fsim:elevator:0.5`. A part that several
channels move adds them: `fsim:elevator+aileron:-1` (an elevon). A trailing
`@<lo>,<hi>` holds the turn within the part's own stops, in degrees:
`fsim:elevator@-20,50` for a canard that travels further than the elevons
sharing its channel.

Other moving parts follow the vehicle's state too - the same numbers your
code reads from `state()`, so what you see is what the simulation did:

- `fsim:gear:<deg>[:<g0>:<g1>]` turns by `deg` as the gear position (1 down,
  0 up) goes from `g0` to `g1` (default 1 to 0): a leg, or a door
  (`fsim:gear:90:0:0.2` opens in the first fifth of the way down). Nest two
  for a leg that swings and twists at once.
- `fsim:speedbrake:<rad>` turns by `rad` times `speedbrakePosition` (the
  drag devices' opening, 0 shut .. 1 open; `fcs/speedbrake-pos-norm`): an
  airbrake's plate or a spoiler's panel, hinged at its forward edge. As a term
  of a control surface's name (`fsim:rudder+speedbrake:-0.44`) it is that
  surface's share of a speedbrake made of control surfaces - the F-22A's
  rudders apart and ailerons up.
- `fsim:lef[:<gain>][@<lo>,<hi>]` is a leading-edge flap, turned by
  `leadingEdgeFlapRad` (where the aircraft's flight controls put them,
  `fcs/lef-pos-deg`; 0 for an aircraft without) times the gain, held within
  its stops.
- `fsim:propeller:<engine>` turns about its x axis at that engine's
  `engineRpm`; it stops with the engine and holds still while the clock does.
  A rotor's speed comes from the simulation: a helicopter's rotors report
  their own, a multirotor's each motor's rotor as its flight control system
  spins it from the commands (`propulsion/engine[i]/rotor-rpm`) - whatever
  drives them, a policy's per-motor thrusts included.
- `fsim:disc:<engine>:<rpm>` and `fsim:blades:<engine>:<rpm>` are a fast
  rotor's blur. Sampled at the frame rate, blades turning past a few hundred
  rpm seem to stand still or creep backwards, where the eye sees a disc: so a
  disc node (inside the rotor's `fsim:propeller` node, a translucent annulus
  in its y-z plane) is shown while the engine turns at `rpm` or more, and the
  blades node beside it below that; each other time shrunk to nothing, with 5 %
  of hysteresis. hangar puts the threshold where a blade moves a third of the
  way to the next between frames at 60 Hz.
- `fsim:nozzle:<engine>:<deg>` turns a nozzle petal open by `deg` times the
  engine's `nozzlePosition` (JSBSim's turbine: shut at military power, open at
  idle and with the afterburner lit).
- `fsim:afterburner[:<engine>]` marks where the engine's jet leaves the
  nozzle: its origin at the exit, its x axis along the jet. The viewer draws
  the exhaust there (see [Effects](#effects)), as big as the node's own
  geometry is across its x axis, and turned with the node, so a nozzle that
  vectors takes its flame with it. With the effects off, the node's own mesh
  is the flame, stretched along x with the engine's `afterburner` (0 out ..
  1 full) and hidden when it is out.
- `fsim:oleo:<wheel>[:<gain>]` slides along its x axis by the unit's
  `wheelCompressionM` (times the gain): a strut's piston, so a parked
  aircraft stands on its wheels.
- `fsim:steer:<wheel>` turns about its x axis by `wheelSteerRad`.
- `fsim:wheel:<wheel>:<radius>` rolls about its x axis at `wheelSpeedMs`
  over the radius, and spins down in the air.

`<wheel>` is the unit's place among the aircraft's wheeled gear (JSBSim's
BOGEY contacts, in the aircraft file's order - the nose wheel, then the left
and right mains, for most).

To make a model's surface move:

1. Put the node's origin on the hinge line.
2. Turn the node so its x axis runs along the hinge, pointing the way that
   makes a positive rotation the positive deflection.
3. Put the surface's mesh under the node.

Every vehicle turns its own copy of these nodes; the geometry under them is
shared. Models written by hangar are built this way. The vision cameras and
their segmentation images show the same deflections.

### Breakup sections

A designed aircraft's model is split into the sections a structural kill breaks it into - `fsim:section:nose`,
`centre`, `left-wing`, `right-wing`, `tail` (hangar's, [hangar.md](../hangar.md#the-3d-model)) - each node at its
break, what is mounted on it (its control surfaces, gear, pylons, nozzles) below it. Whole, they are at rest; the
damage chain's `Separated` events hide the failed sections, while debris entities draw those sections alone at
their own simulated poses. This is implemented in `VehicleVisuals::hideSection` and the viewer's event handling.
The native engagement delivery did not rebuild this optional viewer; source support is not fresh viewer acceptance.

### Parts the airstream turns

A missile's `fsim:rolleron:<radius>` node (its rolleron, a notched wheel) spins about its x axis with its rim at half
the airspeed (`airspeedTrueMs`); above 12 turns a second - where, sampled at the frame rate, its notches would seem to
stand still or crawl - it is hidden and its `fsim:rolleron-disc:<radius>` node (the blur its spin makes) shown, as a
rotor's disc stands in for its blades. A `fsim:freeroll:<turns per metre>` node (a free-rolling tail) turns about its
x axis that many times per metre flown.

## Stores on their stations

An aircraft's stores (MS-7: `loadLoadout`, a scenario's loadout) are drawn on its stations while it carries
them - each with its type's model, the store card's id with its hyphens dropped (`aim-120c` -> `jsbsim:aim120c`, the
design in `aircraft/aim120c`) - and gone from the station as it is released (a weapon in flight is a vehicle of its
own then) or jettisoned. The stations' places are the model's manifest's (`station <n> <x> <y> <z> <bay 0|1>`: a
store's CG, body axes from the model's origin), written by hangar from the aircraft's store-station card. A station's
pylon and launcher - the model's node `fsim:station:<n>` - show once a store has been on it, and stay when it leaves;
a clean aircraft shows none. A store in a weapon bay hangs in its bay behind the bay's doors, the model's nodes
`fsim:bay:<bay>:<deg>` (hangar's, cut from the skin over the opening), which turn their travel as the store management
opens them (`StoreInfo::bayDoors`, half a second before a release from the bay) and shut after it. A live world sends
each store's card and its bay's doors in its mission record; a recording carries the stores as they change (its `M` records, version 3), so a
replay draws them too.

Everything an aircraft carries beside its people, its built-in equipment and its small items is a store ([hangar.md](../hangar.md#mass-accounting)), drawn the same way: the F-22A's, F-35A's, J-20A's and Su-57's bay missiles in their bays, the EA-18G's three ALQ-99 jamming pods on its stations 3, 6 and 9 (its airframe's model no longer has them: as it spawns it is drawn clean, and with its "jamming pods" loadout each pod hangs under its pylon and leaves with a jettison), a missile jettisoned as the body it is. A transport's cargo - a loaded pallet, an airdrop bundle - rides on its hold's pallet positions (its manifest's stations, inside its skin: drawn there, out of sight); released over the ramp it is a vehicle of its own, drawn as its box, its pallet and its straps (`aircraft/463lpallet/463lpallet.glb`, `aircraft/cdsa22/cdsa22.glb`), and listed among the vehicles. hangar cuts no bay and hangs no rack for a hold's stations: the hold is the fuselage as it is drawn.

### Where a picture is hung

A vehicle's state is its centre of gravity's: where it is, how it moves. Its model, its stations and its parts are
measured from its model's origin - its empty CG, the point hangar builds its picture about - and the two are not the
same point: the CG moves in the airframe as fuel burns and stores leave (0.1 to 0.35 m in a loaded fighter, 0.7 m up in
a C-17A or a C-130J with their fuel, 1.1 m ahead in a KC-46A). The simulation releases a store from its station as
measured from the origin, and keeps the airframe still when a store's leaving moves the CG.

The viewer draws the same thing. Each published sample says where the vehicle's CG is from its model's origin (its
`derived` values from `kDerivedCgFromOriginM`, body axes: `ipc/WorldLayout.h`, layout 8), and the viewer hangs the
model that far back from the state's position (`ipc::originEcef`; `drawnState` in `app/viewer_main.cpp`) - the model,
the stores on its stations, its flames and its vapour with it. So a store is drawn on its station exactly where it
becomes a vehicle of its own (within 4 mm as an F-16C's five stores leave: `tests/test_mass_change.cpp`), and the
airframe does not hop as they go. The panels' figures (height, position) stay the state's own. A recording carries the
same values (format version 6); one made before has none and is drawn as it was, each model's origin at its CG.

A store's own model has its origin at its CG as it hangs on a station: a tank's, a pod's and a cargo store's middle,
the Mk 82's CG, a missile's CG with its motor full. A missile's file says so (`fsim/model/origin-x-in`), since its
empty CG is its structure's, burnt out: lit, its CG moves 0.2 to 0.5 m ahead as its motor burns, and its picture stays
on its airframe. hangar's model stage holds every store's model to this - its nose and its tail from its origin, its
length, its width or its fins' reach measured on the mesh's own vertices ([hangar.md](../hangar.md#stores)).

## Flight status

A live world publishes, with each snapshot, what each vehicle's Vehicle Interface has of it ([vehicle-interface.md](../vehicle-interface.md), 7.3):

| Shown | Where | What |
| --- | --- | --- |
| `16416 kg` | the line above the vehicle list, for the selected vehicle (in the monitor panel while the list is hidden) | what it weighs now: its stores and fuel in it (the tooltip: its fuel) |
| `cas 95` | the same line | its calibrated airspeed, m/s: amber below the least a command may ask, red below its stall, in the air |
| `least 80` `stall 73` | the same line | the least calibrated airspeed a command may ask, and where it stalls, at that weight (the tooltip: how much its load has raised them) |
| `floor 95` | the same line, amber, while it holds | its speed floor: it sank at the airspeed commanded and is flown faster |
| `bank <= 40` | the same line, while its airspeed leaves less than 60° | the steepest bank a turn is flown with at that airspeed; amber while it is banked 5° past it |
| `LOW ENERGY` | the list's state column and the vehicle's label, amber | an activity of its is on the speed floor (`kActivityLowEnergy`) |
| `DEVIATING`, `UPSET` | the list's state column and the vehicle's label, red | it is leaving the altitude it commands (`kActivityDeviating`); a wing far past its angle-of-attack limit (`kActivityUpset`). The gravest of the three is the one shown |

- A selected vehicle's label keeps a white ring when it is coloured so, so that it is still told from the others. On the ground nothing is marked.
- The values ride in each vehicle sample's eight `derived` values (`ipc::Derived`, `WorldLayout.h`), which the layout had set aside and never written: the segment is as large as it was, its version unchanged, and a viewer and a library of either age read each other's (an older library's zeros show nothing). They are filled only when a snapshot is copied - 60 times a second at most - not each step.
- A replay carries none: its list and labels are as before.
- Cost, measured (an RTX 5070 Ti, 3840x2160 fullscreen, the 35 designed aircraft loaded - their stores drawn - and in view with the panels and labels shown; three runs each of the viewer before these changes and after, 25 s from 15 s in): 145 to 147 frames a second before, 146 to 147 after, the application's own share of a frame 0.38 to 0.40 ms before and 0.33 to 0.37 ms after - the frame is the GPU's (5.9 ms of clouds, vapour and the scene). The simulation steps those 35 aircraft, published, at 2,730 to 2,760 world steps a second before and 2,740 to 2,765 after.

## Effects

Current armed missile behavior is defined by the [native engagement contract](engagement.md). The Hangar
showcase and `flightsim-showcase` recordings are scripted visual fixtures: their missile motion, fin deflections
and scheduled bursts are not native flight, seeker or fuze acceptance. The dated visual calibration and timing
results below retain their recorded scope. The [engagement validation record](engagement-validation.md) did not
rebuild or test the native Vulkan viewer.

Drawn from each vehicle's state as the simulation reports it - nothing is
added to `VehicleState` for them - after the effects DCS World shows:

| Effect | When | From |
| --- | --- | --- |
| Afterburner: a translucent jet - its potential core, the shear layer round it where the reheat's leftover fuel burns in the air, a train of shock diamonds fading downstream, the soot glowing at its own temperature (Planck's law, so the colour and the brightness are one curve) and the violet of the flame's radicals, eddies carried down the jet; a two-dimensional nozzle's jet flat at its lip, relaxing to round | afterburner lit, staged in behind its light-off; the jet is worked out from the nozzle, the gas and the air: the expansion and the shock spacing from the nozzle's pressure ratio (the inlet's ram pressure with it, so a faster jet has a longer train), the core's length from the jet's density and the airspeed's coflow, quenched in thin air | `afterburner`, `engineN2`, `altitudeMslM`, `airspeedTrueMs` |
| Nozzle glow, from astern | the last turbine stage and the pipe glowing at the gas's total temperature there, as hot metal does (Planck's law, emissivity 0.85, at the exhaust's exposure): nothing to see at idle (some 650 K - below what glows), a dull orange deep in the pipe at military power by day (1,340 K in the model), bright by night; with the afterburner lit, the flame filling the pipe. That is a fighter's augmented turbofan; a jet without an afterburner (its kind from the model's manifest: hangar writes it) has a convergent nozzle, choked at most, and a cooler core - 1,000 K at full dry power out of a turbojet or a low-bypass engine, 850 K out of a high-bypass one (a CFM56's core exhaust at take-off) - so a tanker or an airliner at cruise shows its turbine's metal, not a glow (drawn as a fighter's, a KC-135R's nozzles glowed orange at 1,200 K) | `engineN2`, `afterburner`, `altitudeMslM` |
| A missile's motor: its plume - a supersonic jet out of a highly expanded nozzle (Mach 2.8 - 2.9, its exit 3 - 3.6 times sea level's pressure at full thrust), its throat white-hot seen from astern, the H2 and CO in its exhaust burning in the air round it - following the thrust: wider and longer at the boost, narrower at a sustain, gone at burn-out; under-expanded more the higher it flies; bright from its nozzle, its core holding its exit's temperature through its unresolved shock train; by its propellant: an aluminised motor's hotter (2,327 K at the exit, its alumina freezing) and its alumina dense - a white flame some 1.9 missile lengths long, widening to 3.5 diameters and closing like a tongue; a reduced-smoke motor's (1,648 K) its core's glow, its carbon burnt in its shear layer - a white core into its trail's haze, longer the higher it flies; a minimum-smoke motor's cooler and fainter. Its gas and its exhaust's composition by its propellant class through NASA's equilibrium code (CEA: [world/Propellants.h](../../src/world/Propellants.h), `tools/propellants`). Held to launch photographs ([below](#a-missiles-plume-held-to-launch-photographs)) | while it makes thrust: its model's manifest names it a `rocket` (hangar's `[motor]`), its nozzle's exit and its boost's thrust | `thrustN`, `altitudeMslM`, `airspeedTrueMs` |
| A missile's smoke trail: white smoke laid behind its motor while it burns, spreading with the air's turbulence and the wind's shear, drifting with the wind, lit as the contrails are, gone once it can no longer be seen (optical depth 0.02). Its primary smoke from its propellant's particles - an aluminised motor's alumina (34 % of its exhaust, NASA CEA; 250 m2/kg), dense: about 1.6 across the trail behind an AIM-7M's boost at 6 km, seen for some four minutes; a reduced-smoke motor's catalysts and carbon, a haze (0.15); a minimum-smoke motor's next to nothing. Its secondary smoke - an ammonium perchlorate motor's HCl taking up the air's water into droplets - only where the air is humid enough, more readily cold than warm: none on a typical day at 6 km, a thick trail in humid air (about 16 across behind a reduced-smoke motor at 90 %; its HCl 27 % of its exhaust, an aluminised motor's 21 %, NASA CEA). The motor's mass flow from its class's specific impulse (CEA's ideal, 95 % of it delivered); where the numbers are estimates (that efficiency, the residue's share, the HCl's onset and how much of it condenses) the code says so; 0.05 ms of GPU time with the camera in the trail | while its motor makes thrust (no wake of its own: a missile's is too small to see) | `thrustN`, `airspeedTrueMs`, `altitudeMslM`, the world's humidity and wind |
| A missile's ice contrail: where the air is cold enough, its exhaust's water freezes into a contrail in the same trail, as a jet's does - whether by the Schmidt-Appleman criterion for a rocket's exhaust (its water once its hydrogen has burnt in the air - 0.33 kg per kg of a reduced-smoke propellant, 0.29 of an aluminised one, NASA CEA - against the heat it brings the plume: its sensible heat, its afterburning and the jet's kinetic energy relative to the air - some 5.9 and 10.8 MJ/kg. The mixing line's slope at 10.5 km: a reduced-smoke motor's 2.22 Pa/K against a jet's 1.61, so it forms in air some 3 K warmer than a jet's contrail does; an aluminised motor's 1.06, so it needs colder air - on a typical day the tropopause's, not 10.5 km), how much by CoCiP's initial ice over the trail's section on the exhaust's particles, and then CoCiP's plume: on a typical day it sublimates as the trail takes in the air (an AIM-120C's sustain's for some 25 s at 10.5 km, an AIM-7M's boost's for some 70 s at 11.5 km), where the air is ice-supersaturated it persists and spreads (some 540 m wide after 15 minutes at 110 %). A reduced-smoke motor's haze becomes a dense white line (an optical depth of several across it). The crystals' number (1e15 per kg, CoCiP's for a jet's soot) is an estimate, said so in the code; no GPU time of its own (the smoke trail's draw, 0.009 ms with a trail in view at 1920 x 1080) | while its motor makes thrust, where the criterion holds | `thrustN`, `airspeedTrueMs`, `altitudeMslM`, the world's temperature and humidity |
| An engagement's fire and smoke solved as a fluid (MS-7h): NVIDIA Flow 2's combustion solver on its own Vulkan device - the one way a detonation, a fuel fireball and a burning leak are drawn (the fireballs pieced from spheres are gone, 2026-10-11: no `--fire` option, and a viewer cannot be built without Flow). Its sources: a warhead's charge - its fireball the photographed one, 5.89 m for 1 kg of TNT and 12.05 m for 10 kg fired at the ground, in free air a sphere holding as much (4.67 W^0.31 m), larger in thin air by (p0 / p)^(1/3): an AIM-120's 8 kg some 11.5 m across at 6 km - a holed tank's spill (CCPS's fireball) and the leak burning on from its component along the hole's path (API RP 521's jet flame), each held to its observation by `flightsim-flow-calibrate check`: 12 cases, sizes within 15 %, durations within 20 % (a detonation's 0.89 - 1.03 of the photographed diameter; until 2026-10-11 it was fitted to a size a second 2^(-1/3) smaller, and drawn at 0.73 of it). The solver has a thread of its own and is given each of its steps as it falls due - 60 a second, each with its sources where they are then, as the calibration steps it (given a step a frame, a leak's flame at 160 frames a second was 0.85 of its length and half its smoke) - its smoke and temperature read back each step, packed as bricks in a 3D texture, drawn page by page (32 m) as radiative transfer through soot with the atmosphere's light - the flame at CCPS's 1,600 K, soot near black. The solver's near field: a leak's smoke thins in some 8 s; once nothing burns and nothing is left of it the solver rests (world/FlowFires.h, world/FlowVolumes.h; assets/clouds/shaders/flow.frag). Its cost: [the frame rate](#the-frame-rate) | the engagement's Detonation and Fire events (a fire's `rateKgS` as it changes), the burning component's place on the drawn vehicle | `EngagementEvent`; the vehicles as drawn; the world's air and wind |
| A hit aircraft (MS-7g's damage chain, MS-7h): what the world's damage made of it - its holed tanks' fires streaming from where they are on it (the flame its soot hot, glowing as much as it absorbs, its temperature fluctuating 20 % rms with the eddies as a turbulent fire's does - bright patches over a darker mean, the mean the black body's of a jet fire's 250 kW/m2 - then its black smoke, soot per metre the leak's over the airspeed), a spill's fuel fireball (CCPS: 5.8 M^(1/3) m across, 0.45 M^(1/3) s long, 350 kW/m2, released over 0.15 s from the moving aircraft as five lobes along its path - the QF-4B's fireball, stretched aft), the sections that failed gone from it, each a piece of its own tumbling down (its aircraft's model, its section alone) | the engagement's events (`Fire` with its rate and spill, `Separated`) and the pieces' states in their own slots (type `debris:<aircraft>:<section>`), as the world (World::detonate, its damage stepped) or a showcase's world run writes them | `EngagementEvent`; the pieces' `VehicleState`s; the model's breakup sections (fsim:section:<name>) |
| Heat haze: the scene behind an engine's exhaust bent and shimmering | whenever an engine runs, idle included: the hot gas's index of refraction (Gladstone and Dale, n = 1 + 2.26e-4 m3/kg x its density) - its mean profile across the jet's mixing layer a lens, its eddies' density fluctuations (rms 15 % of the deficit) the shimmer, each ray turned by the gradient integrated along it (the eddies' turns adding as a random walk over the eddies a ray crosses, as long along it as they are drawn: streaked down the jet); the lens a pixel or two near the nozzle at a normal field of view, as the physics gives, the shimmer drawn 8 times what the eddies give (as given, 0.05 px a frame at a 40 m chase: an engine flying showed nothing) - an Su-57 seen from 25 m astern shimmers 0.3 px over its nozzles at idle, 0.7 px at military power, the turbine and flameholders up its nozzles moving behind it; seen against the airframe, the ground and the horizon, not against a clear sky | `engineN2`, `afterburner`, `altitudeMslM`, `airspeedTrueMs` |
| Wing tip vortices: thin tubes trailing from the tips, laid in the air the aircraft flew through (straight in level flight, curving through a turn, a corkscrew in a roll), rolling inboard, sinking, swelling and thinning until they no longer fog | where the vortex's core expands the air past its dew point: its circulation is the wing's lift (DATCOM's lift slope over the model's own planform, Polhamus's vortex lift) over the airspeed, a Scully vortex round a core a few per cent of the span across - a hard pull at speed in humid air; a roll loads the downgoing wing, a sideslip the windward one | `alphaRad`, `betaRad`, `airspeedTrueMs`, `angularRateBodyRadS`, `flapsRad`, `altitudeMslM` |
| Leading-edge vortices: tubes over a strake or a swept wing at a high angle of attack, bursting into a fatter, fainter wake further up the wing the higher the angle | the vortex lift of a sharp or highly swept edge (Polhamus), vortex breakdown after Wentz and Kohlman's deltas | `alphaRad`, `airspeedTrueMs` |
| Vapour over the wing: a sheet on its upper surface in a pull, ended by a hard edge where a transonic shock stands on it, marching aft as the speed climbs; near Mach 1 under the wing too | the section's thin-airfoil suction (its loading along the span from a vortex lattice over the model's planform; the thickness draws suction on both surfaces, the lift on the upper and presses the lower), Karman-Tsien on the flow normal to the sweep, a supersonic rooftop ended by the shock, the outer 30 % of the span relieved by the flow round the tip; it reaches higher off the wing the nearer Mach 1 | `alphaRad`, `airspeedTrueMs`, `altitudeMslM` |
| The vapour cone: a bell round the fuselage, soft in front and ended at the shock behind it, its vapour evaporating just behind | the slender body's transonic pocket past its critical Mach number, its shock running aft to the tail by Mach 1 (and behind it, swept aft away from the body); gone by Mach 1.15 | `airspeedTrueMs`, `altitudeMslM` |
| Contrails: CoCiP's plume (the contrail cirrus prediction model, as pycontrails has it) - dense and narrow behind the jets, carried down and widened by the wake's vortex pair, then, in air supersaturated with respect to ice, persisting and spreading with the shear and turbulence, its ice settling and mixing out; in drier air gone within a kilometre or two | the Schmidt-Appleman criterion, from the air at the aircraft (the world's standard atmosphere and humidity), its fuel flow as the simulation has it (each sample says what each engine burns) and the efficiency it flies at - its thrust's power over that fuel's heat, held to 0.1 - 0.5; a recording from before the simulation said it: the thrust's power at an efficiency of 0.3 | `altitudeMslM`, `airspeedTrueMs`, `thrustN`, `fuelKg`, `velocityNedMs`; the aircraft's span and empty mass from its JSBSim file |

### What the plumes and trails are drawn from

The jet's and the trail's physics are the viewer's; the numbers they start from are the simulation's and the design's,
not the viewer's own guesses:

| What | From | Before |
| --- | --- | --- |
| A contrail's fuel, and the engine efficiency the Schmidt-Appleman criterion takes | what each engine burns now, kg/s - each sample's `derived` values from `kDerivedFuelFlowKgS` (`ipc/WorldLayout.h`, layout 9; JSBSim's own fuel flow), and the thrust's power over that fuel's heat | the thrust's power over the fuel's heat at 0.3 for every engine at every power |
| A missile's smoke and its ice contrail (exhaust per metre of trail) | the propellant its motor burns now, the same values (a rocket's propellant flow) | its thrust over its propellant class's specific impulse - 11 % more than the flight model burns |
| A reheat's flame: its temperature, so its colour, its brightness and its length | the engine's own ratings, its design's through its model's manifest (`engine <index> <bypass> <afterburner> <dry kN> <reheat kN> <fuel g/(kN s) dry> <reheat>`): lit, the burner heats the same flow at the same pressure, so T_reheat = T_dry (F_reheat / F_dry)^2 less the fuel's mass (`Exhaust::reheatTotalK`), T_dry its turbine's exit gas and its bypass air mixed by its bypass ratio; held to 1,850 - 2,300 K. An F100-PW-229's 2,187 K, an AL-31F's 2,053 K, an RD-33's 2,124 K, an M53's and an F119's at the least | one preset, 2,236 K, for every augmented engine |
| A dry jet, a nozzle's glow, a rocket's plume | as before: the engine's kind (its bypass ratio, its afterburner) and a motor's propellant class, exit and boost thrust from the manifest; the state's core speed, afterburner and thrust | |

An engine that turns a propeller has no nozzle on its model, so until now the viewer had nowhere to start its
exhaust from: a C-130J or an EC-130H laid no contrail however high and cold it flew. Its manifest now says where its
exhaust leaves it (`exhaust <engine index> <x> <y> <z> <radius>`, body axes from the origin: the design's
`[engine.exhaust]` position and diameter, else an estimate behind and below the engine's place, its pipe's width from
its power), and the trail is laid from there with what the simulation says the engine burns and the thrust its
propeller makes - a C-130J at 10,500 m trails four contrails. (Its hot gas has no flame to draw; its heat haze is not
drawn: [backlog](../backlog.md), B-59.)

What every model gives the viewer - each design's moving surfaces, its propellers, its nozzles and placed exhausts,
its rocket's line, its lifting surfaces - is listed in [hangar-effects.md](../hangar-effects.md), written by `python -m
hangar effects --record docs/hangar-effects.md` from the designs, the models' nodes and the manifests, with what a
design has that its model gives nothing for; a test holds that list empty for all 87 designs
(`tools/hangar/tests/test_effects.py`). Every fixed-wing design's control surfaces and drag devices have their joints;
a missile's fins have theirs and stand still, because its flight model has no controls to move them
([backlog](../backlog.md), B-36); a tank, a pod, a bomb and a cargo store are rigid bodies with nothing to move.

hangar writes the manifest's engine and exhaust lines with the model, and again whenever the model stage runs - an
edit to an engine's ratings or its exhaust's place re-meshes nothing (`model3d.refresh_engine_lines`). The clock the trails and the fires are drawn by is
the world's: a vehicle that has struck the ground is stopped, its own clock with it, and by that clock every trail
froze and a fire never grew.

### A hit in the air

What a burst does to an aircraft was already worked out from where it was - the fragments that reach it, the blast on
each face, the push and the turn they give the whole aircraft (`ms::damage::assess`, MS-7g). Two things it left out of
what is then drawn, so that a hit from the left looked like a hit from the right:

- **A section the burst breaks off** left its root at the break's 5 m/s, straight outboard, whichever side the burst
  was on. It now leaves with its own share of the hit: the fragments' momentum on its own panels and the blast's
  impulse on its faces, away from the burst, over its mass - less what the whole aircraft took, which is already in
  its motion - and the aircraft takes the reaction (momentum is kept). No faster than 60 m/s from the aircraft (an
  estimate: a lighter part tears, it is not thrown whole).
- **A holed tank's fireball** was laid along the aircraft's path alone. The fuel thrown out at once now goes the way
  the fragments went through the tank - from the burst, out of its far wall (hydrodynamic ram) - at 3 % of their speed
  (an estimate: some 50 m/s behind fragments at 1.7 km/s), on top of the aircraft's own velocity: the fireball is still
  drawn out down the flight path, and off it to the side away from the burst.

Both are in the engagement's events (the `Fire` event's velocity, the piece's own state), so a replay, a live viewer
and the SDK see the same. `tests/test_damage.cpp` mirrors one burst left and right of a MiG-29A and holds each to its
side. What an aircraft does not do in the air is explode as a whole (a tank's vapour, its weapons): it burns and breaks
where it is hit ([backlog](../backlog.md), B-58).

### A ground strike

An aircraft that flies into the ground is no longer simply stopped there. The world works out what the strike does
(`ms/damage/Impact.h`, `session::World::impacted`) from how it came in, and the viewer draws what the world says:

- **Its angle.** Its velocity is split across and along the ground (the ground's own slope: a level flight path into a
  hillside is a strike). Across it, it is stopped within what it has to crush - its mean deceleration v_n^2 / 2d; along
  it the ground can only drag it (Coulomb's impulse). What leaves the strike keeps the rest: a 60 deg dive's wreckage has
  no way on and lies within some 25 m; a 7 deg strike's goes on down its track at nine tenths of its speed - a piece
  1.2 km on.
- **Its forces.** That deceleration is handed to the damage model as an accelerometer's reading: each section whose
  root cannot hold it breaks off, as under a flight load, and flies on as the piece it is (its own flight model), from
  where it lay, with what the strike left it. An F-16C's dive at 233 m/s: 1,470 g, wings, nose and tail off; its
  shallow strike at 200 m/s: 23 g, two sections.
- **Its fuel.** Tanks hold to MIL-STD-1290's design impact (12.8 m/s into the ground) and all burst by 45 m/s; a share
  of what spills, up to 0.4, burns at once as mist - the fireball, a `Fire` event moving with the wreckage - and the
  rest burns on where the wreck lies until it is gone (the dive's 1,270 kg at once; the shallow strike's 210 kg, its
  fireball drawn out down its track).

A released store that strikes the ground is a strike too. It ended there without a trace; now its `Impact` is told
before it ends, and a jettisoned tank's fuel left - the point mass the world set as it left its station - bursts and
burns as an aircraft's does (an F-16C's three tanks let go at 400 m: three fireballs, 339 kg burnt at once; its two
Mk 82s' bodies have no fuel to burn). Nothing of it burns on: the store is gone.

The fire is the engagement's fire: NVIDIA Flow's spill and leak, its cloud of fuel stood on the surface (the solver has
no ground). The strike itself is an `Impact` event (its angle, its energy, its load in g, the ground's normal, what
broke) for a recording and the SDK. A live viewer is given the engagement's events as they happen - a ring of the last
256 in the world's segment (`ipc::EngagementRing`, layout 9) - so a live world shows its detonations, fires and
breakups as a replay does; before, only a replay did. The estimates (the crush depth from the mass alone, the ground's
friction 0.6 and restitution 0.1, the mist's share) are said in `Impact.h`; none is fitted to a crash.

The viewer draws no fire without Flow, and is not built without it: its headers and its two libraries are in
`third_party/flow` (2.2.0, built by `tools/flow/build-flow.sh`), so a clone has them; `-DFSIM_FLOW_DIR` names another
built checkout. Should its libraries be missing beside the viewer at run time, the log says so as an error ("no fire
solver ... NOT drawn") - nothing stands in for it.

### A missile's plume held to launch photographs

These comparisons check rendered plume shape and brightness against photographs. The missile states are scripted
visual fixtures; the comparison does not calibrate missile trajectories, motor performance, guidance or seekers.

Four launch photographs on Wikimedia Commons (U.S. government works; their files, the points measured from and the
numbers in [tools/plume_check/photos.json](../../tools/plume_check/photos.json)) measured by
[tools/plume_check/measure.py](../../tools/plume_check/measure.py), along the missile's axis in missile lengths L - and
the viewer's plumes measured the same way, side-on from 30 m a second into the boost at 6 km
(`python tools/plume_check/check.py`; a check run by hand, it needs the viewer and a display):

| | saturated from .. to | its width at 0.1 / 0.5 / 1 L | dark behind the nozzle |
| --- | --- | --- | --- |
| AIM-7, AIM-7M (aluminised), three photographs | 0 .. 1.6 - 2.3 L | 1.5 - 2.0 / 2.3 - 3.2 / 3.2 - 4.1 diameters | no |
| the viewer's AIM-7M | 0.0 .. 1.9 L | 1.4 / 2.4 / 3.5 | no |
| AIM-120 (reduced smoke), daylight | not saturated: 1.5 times the sky at 0.1 L, a haze by 0.5 L | | no |
| the viewer's AIM-120C, AIM-9M | a white core, at 1 L 1.39 and 1.35 times the sky at 6 km (1.33 and 1.09 at 3 km) | 0.1 - 0.4 / 1.2 - 1.4 / 1.0 - 1.5 | no |
| the viewer's AIM-7M and AIM-120C before | 0.74 .. 1.4 L, 0.76 .. 1.1 L | - / - / 1.4, 0.8 | yes: dark for half a missile length |

Before, the jet was drawn as an ideal gas expanding: at 6 km its core cooled to 1,400 K, a thirtieth of the light at
1,700 K, and the flame began half a missile length aft, where its shear layer had grown hot enough; the aluminised and
the reduced-smoke motors looked alike. Now (world/Exhaust.cpp, rocketOf): the core holds its exit's temperature through
the shock train, which no photograph resolves (the recompression behind each Mach disk keeps the cells' light near the
exit's); an aluminised motor's gas is hotter (NASA CEA: its chamber 3,456 K against a reduced-smoke motor's 2,962 K, its
exit 2,327 K - its alumina freezing - against 1,648 K, its two phases' gamma 1.135) and its alumina, which does not
burn, glows in its shear layer; a
reduced-smoke motor's carbon burns there, so its light is its core's, near the nozzle; a rocket's shear layer spreads
2.2 times a jet's (an AIM-7's flame widens 0.08 of its length each side in the photographs, the compressible layer's
0.036) and its flame closes towards its end, its edge first. The smoke trail's primary smoke shows within 0.01 s
(about a missile length at launch speed) instead of 0.05 s. Its gas, its nozzle's exit and its exhaust are its propellant class's
through NASA's equilibrium code (CEA, through RocketCEA - `tools/propellants/cea_classes.py`, its numbers in
`tools/propellants/classes.json`, which a test holds world/Propellants.h to): one composition a class for every nation's
missiles (aluminised AP 68 % / Al 18 % / HTPB 14 %, reduced smoke AP 86 % / HTPB 14 %) at a 10 MPa chamber through the
hangar designs' area ratio (4.94; their thrust over their throats gives them 9 - 10 MPa). The reduced class's AP share
is the assumption that counts: 84 - 88 % moves its exit 1,477 - 1,847 K. Their exits are under-expanded 3.1 - 3.6 times
sea level's pressure at full thrust (estimated 1.5 before), so the jets widen more: the Sparrow's flame is as wide as
its photographs' now at every station. A reduced-smoke motor's core runs longer with it: at 6 km its glow at 1 L is
1.39 times the sky against the AIM-120 photograph's 1.23 (its altitude unknown; at 3 km 1.33) - the open difference
(`check.py` reports it); held lower by the cells' light (`trainMean`) it went dark behind the nozzle again, so it is
left as the physics gives it. The reduced-smoke photograph's sky is darker than the
viewer's (its plume 1.5 times its sky, which the viewer's brighter sky leaves saturated: compare where it saturates and
how its light falls off, not contrast one for one). No measurable GPU time: the gas pass 0.27 ms with the Sparrow's
flame filling the screen against 0.28 before.

**At the launch.** Four photographs taken as the motor lights or just after (Wikimedia Commons, U.S. government
works: an AIM-9M off an F/A-18F's wingtip rail seen from its cockpit; an AIM-7M ejected from an Australian F/A-18A; an
AIM-9M leaving an F-22's trapeze; an AIM-120 off an F-14A) against the viewer's F/A-18C launches, frame by frame on the
replay's clock (`--screenshot-clock replay`, [Screenshots](#screenshots)):

- No flash. None of them shows a light at ignition beyond the plume itself, so the viewer draws none: the plume and
  the trail start with the motor's thrust, as they do.
- No smoke cloud apart from the trail. The smoke round the Hornet is its Sparrow's trail some 0.05 s old - the air
  carries it past the carrier at its speed - the missile 0.6 s into its boost, 4 m ahead of the nose; it is 5 - 6 m
  across there, against 1.7 - 2 m at the same age behind the F-15s' Sparrows, clear of them (and the viewer's 2 m).
  What spreads it is the carrier's own flow - its intakes', its forebody's vortices' - which the viewer does not model:
  the open difference. The viewer's Sparrow, ejected and lit a quarter second later 3 m below the carrier's CG, lays
  its trail 2 - 4 m under the fuselage, as the photograph's projection allows.
- The flame on the launcher. The F-22's photograph shows the missile's flame splashed over its trapeze as the missile
  leaves it; the viewer's flame passes through what it meets - not modelled, the open difference.
- A reduced-smoke flame from the cockpit: the Sidewinder's is pink-orange to 0.7 L (r/g 1.4 - 1.5, measured along its
  axis where the sky is behind it) and gone into the sky by 0.9 L - with the AIM-120's photograph, the second against
  the viewer's reduced-smoke glow at 1 L.
- The AIM-120 off the F-14A shows a string of separate bright beads - its shock cells - some 1.8 L behind its nozzle,
  and the F-35A's AIM-120 its glow dipping to 0.4 of its peak within 0.5 L: a reduced-smoke motor's cells, with no
  alumina glowing between them, may be seen where an aluminised motor's are not. The viewer holds every rocket's core
  at its exit's temperature through the train (none of the Sparrows' photographs shows cells), so it draws none - and
  the cells' cold gas between them may be what its reduced-smoke glow at 1 L lacks: the lead for that open difference.

The exhaust is a port of [aeronautic](https://github.com/Romainlg29/aeronautic)'s
afterburner (MIT): its jet model, its hull, its ray march through the plume's
field with its step budget and distance tiers, set up as its own fighter example
sets it (the `afterburner` preset; exposure 120 by night, coming down in stops to
the sky's by day; three.js's AgX tone mapping). Not as theirs: the heat haze,
worked out from the gas's density above where theirs is a distortion of the
screen; the nozzle's glow by the turbine's temperature, where theirs is a share
of the screen's brightness. Left out: its pages' bloom; and the tone mapping is
the plume's light alone, where theirs maps the whole frame.
Close behind an F-16 in reheat it costs 0.11 ms a frame at 1600 x 900 on an RTX
5070 Ti.

The vapour is a port of aeronautic's wing vapour (MIT). None of it is drawn to a shape. The pressure
round the aircraft - the four phenomena above, small perturbations of one flow
that add - is worked out from the model's planform and the flight, the day's
moist air is expanded through it on the moist adiabat (the latent heat the
condensing vapour gives back included), and cloud is drawn wherever the air
passes its dew point - thickening over a range of expansion past it, as its
droplets nucleate and grow in the air's few tens of milliseconds in the pocket
(full at twice the deficit that saturates it), so its edges thin out rather than
ending opaque where the air saturates - and patchy where the air is a few per
cent moister or drier -
patches in four octaves of eddies from 0.6 m, weighted as turbulence weights a
passive scalar's (Obukhov and Corrsin's -5/3 spectrum: most of the variation in
the larger eddies), each smeared along the flow as far as the air moves in a
sixtieth of a second (aeronautic's two octaves at the smallest size stood on the
wing as even stripes).
The air is the standard atmosphere from the world's sea-level temperature, as
humid as the world's `humidity` says - or, when it gives none, aeronautic's
default day's 85 %. It runs at aeronautic's `high` quality, its steps at their
full detail (the patches go under 6 % of the screen's height; the cost is held
by the gas pass's budget of rays, below) and its look (sun and sky by day, its night scene's by night, three.js's
AgX tone mapping, as for the exhaust). Dry air stays
clear however hard the pull, and humid air fogs a tip in a firm turn.

`e` switches them off and on, `--no-effects` starts with them off, and
`"effects": false` in `viewer.json` makes that the default. Off, the flames
are the models' own meshes, as before.

The GPU does the work, from a few rows of numbers per vehicle and the clock (the simulation's, so a paused or
replayed world shows the same). The gas round the aircraft - the vapour and the exhaust - is marched in one compute
pass: a ray per texel through every nearby aircraft's vapour field (the nearest 8, within 4 km), skipping the clear
air by the distances the field gives, and every engine's jet showing (the nearest 32), each in its own frame; what
each leaves laid front to back by where it starts, so a plume in a vapour cloud and the cloud round it blend in the
order they lie. The rays end where the vehicles' depth (drawn before the frame) says an airframe is, so a fin or the
fuselage hides what is behind it exactly. The cost is held to a budget of rays (150,000) whatever the window's
size: past it - when the gas fills the screen, its features many pixels across - it is marched at a texel for as
many pixels each way as it takes (any share, up to 4) and laid on the window from the texels round each pixel,
weighted by a Gaussian of their distance (sigma 0.75 texel), which filters the march's dither out. An airframe's
edges (where a texel's pixels lie on different surfaces) are marched pixel by pixel, listed first and then marched
a pixel a thread, so a few of them do not hold up the rays round them, and filtered over their neighbours on the
same surface; they are the one part whose cost grows with the window. The dither is takram's blue noise (the
interleaved gradient noise aeronautic's materials use is a lattice, and stood on the screen as a pattern of dots),
and the vapour is marched at its full detail: aeronautic coarsened the steps up to three times when it filled the
screen, which speckled the thin layer over a wing; the budget of rays holds the cost here. The contrails: a plume
state a sample along the trail, each segment a box whose far faces' pixels march the eye's ray through the segment's
parts, between the planes that end it (so each stretch of the ray is taken once). Vehicles far from the camera draw
nothing. The gas is laid over the frame in a second pass, after the scene is drawn (its depth resolved to one sample
for it): the scene's colour is copied first - only while an exhaust is showing - so a pixel behind hot gas can show
the scene where its ray, turned, goes.

Measured on an RTX 5070 Ti, uncapped, the GPU's time per pass from `--stats` (it prints them each second): an F-16
in a 6.5 g pull in reheat, the camera 10 m behind looking up its nozzle and its tip trails, the gas 0.94 ms at
1600 x 900 and 1.30 ms at 2560 x 1440 (the whole frame's GPU time 1.65 and 2.33 ms, the clouds' march 0.39 and
0.52 ms of it); from 30 m, 0.42 ms; the 14 fighters of `examples/python/fighters.py` in view from 60 m, 0.27 ms. Over
the wing in that pull the vapour's grain (|image - its 5 x 5 blur|, 0..255) is 1.3, against 5.3 before the dither
and the steps changed. The heat haze adds nothing measurable to the march (0.342 ms with it and without it, from 30 m,
alternated three times); the pass over the scene - the frame's pass ending (its resolve), the copy, the gas laid on -
takes 0.09 ms at 1600 x 900, 0.03 ms with no exhaust showing.

The contrails are a port of [pycontrails](https://github.com/contrailcirrus/pycontrails)' CoCiP (Apache-2.0):
its Schmidt-Appleman criterion, wake vortex downwash, initial ice and ice survival, and its time step - the plume's
covariance spread by diffusion and shear, its ice budget against the air's, its particles lost to aggregation and
turbulence, sedimentation, the optical depth from the particles' size and number, and its persistence limits -
function for function with its default parameters (`tests/test_cocip.cpp` checks it against pycontrails' own
functions to 1e-6; `tools/contrails`). CoCiP takes the wake's vortex phase as instantaneous; the viewer draws it as
the wake-vortex literature describes it: the jets roll up into the vortex pair (`b0 = pi b / 4` apart) within some
ten spans; the pair carries the primary wake - two tubes of ice - down from its initial speed `Gamma0 / (2 pi b0)`,
slowed by the buoyancy of the air it carries down, to rest at CoCiP's descent (a quarter of the buoyant oscillation,
as in stably stratified air: at its initial speed to a stop, the trail's sinking part met its level part at a
corner), leaving the secondary wake, a fainter curtain, above it; the Crow instability bends the tubes
(wavelength 8.6 b0, in planes 48 degrees from the horizontal, growing at 0.83 w0 / b0; Crow 1970 - grown from the
air's turbulence, so it is the band of wavelengths the instability amplifies (observed some 6 - 12 b0): three, at
0.7, 1 and 1.4 times 8.6 b0, each its phase wandering and its amplitude coming and going along the trail on its own.
Drawn at one wavelength, even wandering 25 % either way, the tubes bent into the same arc again and again, evenly
spaced, and pinched apart between them: a chain of repeating loops) until they touch
at the lifespan Crow and Bate's law gives from CoCiP's normalised dissipation rate, and pinch into rings, which
dissolve into CoCiP's plume over another lifespan. Each part's ice is a Gaussian section eroded the way takram's
clouds are, by their Perlin-Worley and Worley noises, into a sharp, turbulent edge (its extinction kept on average;
the point the noises are read at drifts across them down the trail - read along one axis, their tiles came round every
55 m, the same eddies again and again),
marched along the eye's ray; each step lit by the sun through the part (its Gaussian's optical depth towards the sun,
in closed form) with the clouds' atmosphere, phase function and multiple scattering, and mapped as they are, so a
contrail and a cloud in the same sky look alike.

`tests/test_contrail_observed.cpp` holds what is drawn against observed contrails (Schumann et al. 2017, "Properties
of individual contrails", ACP 17, 403-438, and the observations it compiles): a B747-400's at 50 s (Sussmann and
Gierens 1999: 90 m wide, 160 m deep, the secondary wake's optical depth 0.24 against the primary's 1.18, the primary
gone by 170 s while the secondary persisted) - drawn 91 m wide and 117 m deep, its total extinction between the
compilation's model and the lidar's; an ATTAS's seen from 25-35 m behind its engines (Busen and Schumann 1995); an
F-16C's young optical depth (0.29) within CONCERT's range (Voigt et al. 2011). Measured off the screen (the trail drawn
pure white and pure black at its own opacity, the difference integrated across it), the drawn extinction per metre
is the model's to 3-5 %, its strongest period along the trail from 8 s of age 62.5-65 m (the Crow wavelength 8.6 b0:
63.6 m), and the trail's height the model's to 0.5 m at 4 s and 5 m (of a 52 m descent) at 14 s (measured with
the descent at its initial speed throughout, and one Crow wavelength). What is the
viewer's, and why:

| Change | Why |
| --- | --- |
| Through the vortex phase (its depth at the pair's initial descent speed) the ice is CoCiP's initial ice for the jets' size as drawn, going to its plume's | CoCiP's, spread over the vortex's whole plume at once, left none in air below ice saturation |
| The young trail (its first 1.5 s: the ice condensing, the wake rolling up) drawn from samples at fixed ages - 0.03 s apart to 0.3 s, further apart to 1.55 s - between those laid every 0.25 s, along the path between them | drawn between the samples laid alone, whose ages moved on every frame until the next was laid, the trail behind the jets faded and filled again four times a second: behind an F-16 from 40 m, the trail's mean brightness swung 7 grey levels at 4 Hz, and a fifth of its pixels changed by more than 4 levels a frame (after: 2.2, no period; a tenth, most of it the trail's texture going by) |
| On a window of 2 million pixels or more, the trail is shaded a fragment to 2 x 2 pixels (VK_KHR_fragment_shading_rate, the draw's own rate, where the device has it; its edges, depth and coverage still each pixel's, its noises filtered for the coarser fragment) | historical measurement using the retired `tactics.bvr` prototype: fullscreen at 3840 x 2160, inside a fighter's trail, the march took 7.3 ms of an 11.5 ms frame (83 fps). After, 2.0 ms of 6.0 ms (155 fps); the ice is soft - the step from one 2 x 2 block to the next averages 0.09 grey levels. This prototype is not a current scenario entry point or native-engagement benchmark |
| Each stretch of trail is drawn as the box round the parts that hold ice at its ends | the box was sized by CoCiP's plume too, which has its size from the start but no ice until the wake has dissolved into it: the young trail's boxes were hundreds of metres across, a camera near an aircraft stood inside dozens of them and every pixel marched through each - a historical view of the retired `tactics.bvr` prototype measured 18 screens' worth of fragments behind its fighters for a puff of trail, the contrails' cost ten times larger looking along the trails than across them (2.5 ms against 0.24). Measured on the same views: 2.47 -> 0.25 ms (40 m ahead looking back), 1.63 -> 0.11, 1.13 -> 0.17, 0.51 -> 0.07 (chase, 60 m); a six-minute-old trail 0.71 -> 0.56, 0.56 -> 0.19; the frames as before (within the difference two captures of one moment make) |
| The wake's tubes and curtain lie in the frame the aircraft was banked in as it laid them (its up, each sample's); the plume's section stays CoCiP's, horizontal and vertical | they were laid level whatever the bank: beside a banked aircraft's nozzles, not behind them |
| The secondary wake holds 0.24 / (0.24 + 1.18) of the young contrail's ice, and keeps it at the flight level while the air there is ice-supersaturated | the B747-400's optical depths over equal sections at 50 s; its primary wake sublimated in the descent, its secondary persisted (Sussmann and Gierens 1999) |
| The jets' ice is seen from half of 0.3 s of age, all of it from 0.3 s | an ATTAS's contrail was seen 25-35 m behind its engines at 200 m/s; within one to two spans behind an airliner |
| Fuel flow is the thrust's power over the fuel's energy at the engine efficiency (0.3, pycontrails' default); the mass is the JSBSim file's empty weight and point masses plus the fuel on board | the state carries neither |
| The shear is 0.004 /s, its part across the track as the world's wind lies to it (2/pi of it without wind) | the world gives no wind profile |
| The world's `humidity` is relative to water (the WMO's definition); none given, a typical day's: 70 % over water at the ground drying to 50 % by 5 km, 60 % over ice through the upper troposphere, 30 % above the tropopause - short contrails where the Schmidt-Appleman criterion holds, none persisting | CoCiP reads specific humidity from weather data; persistent contrails need ice-supersaturated air, which the upper troposphere has a minority of the time |
| A contrail is dropped once its optical depth is under 0.02 (and when CoCiP ends it) | the threshold the contrail-cirrus literature takes for one to be seen (Kaercher et al. 2009); CoCiP's own limits (1e-6, 20 h) kept invisible trails |
| Samples every 0.25 s, the oldest half thinned when 256 are kept | a drawing budget |

At cruise (an F-16 at Mach 0.85, 10.5 km, ice-supersaturated) the contrails cost 0.06 ms a frame from 30 m looking
down the trail, 0.09 ms from 250 m behind (the trail across most of the screen) and 0.08 ms from 400 m (1600 x 900,
RTX 5070 Ti).

The exhaust is part of the vehicle's model, so the vision cameras' colour
images show it too; the rest is the viewer's alone. Segmentation images
never include any of it: an id covers the vehicle itself.

A model made by hangar also tells the viewer where its wings and strakes are
(`wing`, `strake` and `canard` lines in its `.glb.manifest`: per section of
the right half, its leading edge in body axes from the model's origin and
its chord): the wing and its strake are the vapour's wing (the strake its
leading edge inboard, its vortex the leading-edge vortex), a canard sheds tip
vortices of its own. A model without them gets a plain swept wing out to its
widest point.

## Clouds

The world's weather (`cloud_cover`, `cloud_base_m`, `precipitation`; `World::setEnvironment`, Python
`world.set_environment`) is drawn with takram's volumetric clouds for three.js,
[@takram/three-clouds](https://github.com/takram-design-engineering/three-geospatial) (MIT), ported: their shaders,
their textures (the local weather, the shape and detail noise, the turbulence, the spatiotemporal blue noise), their
default layers and their "high" quality preset, lit by the precomputed atmosphere they are lit by (Eric Bruneton's,
as @takram/three-atmosphere ships its tables). The files are in `assets/clouds` (installed in
`share/flightsim/clouds`); each change to theirs is marked `fsim` where it is made.

- Shape: four layers - two low ones of cumulus and stratocumulus (650 m and 1200 m deep, the lower on
  `cloud_base_m`), a thin cirrus sheet at 7.5 km and an empty fourth - over a weather texture repeated round the
  globe on a cube-sphere, shaped by 3D noise, eroded by finer noise and bent by a turbulence; a coverage parameter
  fills them (the coverage modulation Skybolt uses). The world's wind carries the weather and the shapes.
- Light: the sun's and the sky's light from the atmosphere at each sample; the optical depth towards the sun marched
  a little way and read from Beer shadow maps beyond (three cascades over 100 km); multiple scattering as octaves; a
  two-lobed Henyey-Greenstein phase; light bounced off the ground; the powder effect; and the atmosphere's aerial
  perspective between the eye and the cloud. The clouds shade the ground under them.
- Cost: the march runs at a quarter of the window's size each way, one of each 4 x 4 pixels a frame in a Bayer order,
  and a temporal resolve fills the rest from the last frames; the shadow maps are marched at 512 x 512 and resolved
  over frames too. With the 14 fighters of `examples/python/fighters.py` in a chase view at 1600 x 900 on an RTX 5070
  Ti: 2.1 - 2.2 ms a frame with clouds (scattered, broken or overcast alike) against 1.35 ms without, about 0.8 ms.

What is the viewer's and not takram's, and why:

| Change | Why |
| --- | --- |
| The cover is the world's `cloud_cover`, turned into takram's coverage by a curve measured on their own textures (`tools/cloud_map/coverage_curve.py`: coverage 0.3, their default, covers 19 % of the sky; 0.6 all of it) | their coverage is a shape parameter, not a cover |
| Beyond 150 - 300 km of the point under the eye, and everywhere once the eye is above 30 - 100 km, the cover is NASA's Blue Marble cloud map's (`clouds/earth-clouds-8192.png` in the map set, `tools/cloud_map/make_cloud_map.py`), read as Skybolt reads its global coverage map | takram draws no views from space, and the world's weather is the weather round the eye |
| Nearer, the world's cover is laid out as the map's clouds lie there: the map's cover at a point over its mean within 150 km of the eye, f, gives 1 - (1 - cover)^f - the world's cover where f is 1, none where the map is clear, more where it is cloudier (where the map round the eye is nearly clear, the world's cover as it is) | takram's one coverage made an even field of like puffs to the horizon, its weather texture's 100 km tiles alike; real cloud fields cluster and clear over tens of kilometres, and the map has the day's. 0.01 ms (the map read once a ray and again 20 km on) |
| The march's first step is no longer than their longest step | from far off theirs grew past the layers' depth and stepped over them |
| Far off, Skybolt's far clouds (`VolumeClouds.frag`, its method): as a pixel spans more of the weather texture the march gives way to the global map lit as a layer, and past that is not made - from the texture's level 0.75 to 2 (Skybolt's 2 to 6, for a coarser texture): while takram's cumulus (1 - 2 km) span three pixels or more they are marched, and as they shrink to one the layer - the cover a pixel's area holds, what a camera records of clouds smaller than its pixels - takes over | marched from far off, a pixel's few samples in the thin layers fell in or out of a cloud by chance: the globe sparkled with separate, flickering puffs (3000 km up, 11 % of pixels changed frame to frame; 4.7 % then); handed over at Skybolt's levels, a view from 1000 km up still drew every cloud mass as a speckle of separate puffs a pixel or two across. That view costs 0.1 ms less now |
| The noise textures have mip levels and every texture is read at the level a pixel's footprint asks for | takram's 3D noise has none; from far off it aliased |
| Drawn over the viewer's frame with three.js's AgX at their exposure (10), and depth-tested against it where the cloud starts; the rays do not end at the vehicles | their clouds are composited into a frame lit by their atmosphere and mapped as a whole; here the scene's depth does not exist before the march, and ending the rays at the vehicles' depth cut an aircraft-shaped hole in the cloud that the quarter-size march moved off the aircraft (a dark double of it) |
| The upscaling as a temporal upscaler does it (AMD's FSR 2, Epic's TAAU): every pixel takes the frame's samples round it, weighted by how far from it each was marched, their share growing with how far the pixel has moved (0.02 still, 0.1 at 4 px a frame, 0.5 at 32 px - zooming or turning fast - where the frame's samples are read smoothly, as they are where there is no history); the history read through a Catmull-Rom filter (takram's own alternative) and held to the variance of those samples where they were marched | takram gives each sample to the one pixel it was marched for and shows it as it is: their march's jitter stood on the screen as grain and flicker, and in motion a pixel's history was resampled fifteen frames running before it saw a sample - its edge went soft, then sharp, every sixteen frames, and the box the history is held to pulsed as the Bayer offset moved the samples under it. Behind an F-16 over broken cloud (each edge tile matched into the next frame, what is left after the shift, 0..255): the edges' change of shape 2.12 -> 1.69 and their grain 2.21 -> 1.45, with the row below; under the cloud base 2.71 -> 2.34 and 2.97 -> 2.52; still views' flicker 0.42 -> 0.41 (30 km off) and 0.39 -> 0.22 (400 km up); 0.02 ms. Zooming, the history drawn from elsewhere every frame trailed behind the clouds radially, and the edge of the screen - new each frame - showed the samples as a mosaic of 4 x 4 blocks |
| A pixel whose nearest low-res rays found no cloud this frame, while the resolve has cloud there, is drawn at the cloud start of the rays round those, or else where its ray enters the layers | it was dropped: one ray a low-res texel, jittered, can miss thin cloud, and four missing together cut a 4 x 4 hole through it and stepped its edges on the low-res grid, differently every frame - the edges crawled |
| No march step across the layers is longer than 60 m vertically; the gaps between them are skipped to their edge | from high up, kilometre steps through layers 650 m deep found a cloud or missed it by chance (400 km up: flicker 3.45 -> 0.64) |
| Values that are not numbers are not kept (the shadow map's front depth when its transmittance underflows, and anything else) | one spread through the temporal resolves as growing black blocks |
| The resolve carries each pixel from where its light came from - each march step weighted by the cover it adds - not from takram's front depth (each step weighted by the light left after it) | through a thin layer over a dense one (the cirrus over the cumulus, from above), the front depth sat in the thin layer, and the dense one - most of what the pixel shows - was moved with the thin one's parallax: it trailed as the camera zoomed or turned. Zooming from 16 km above the cirrus, the frames' difference from frames drawn without history (blurred 6 px; 1.9 still, 0.65 between two such draws) went 7.9 -> 5.2; behind an F-16 over broken cloud the edges' crawl stayed as it was (2.9 / 2.7, within its spread) |
| The clouds' shadow on the ground is worked out for each pixel in the composite, from the shadow map, the two cascades blended where they overlap | it was marched with the clouds, a ray a 4 x 4 block at the pixel the frame's Bayer offset picked, and read back as if it stood still (and the cascade picked by noise): every shadow's edge on the ground moved up to 2 px a frame on a sixteen-frame cycle. Looking at the ground under scattered cloud, the ground's change frame to frame went 0.1 - 0.2 (cycling) -> 0.01 grey levels |
| The resolve's nearest texel is looked for from no depth at all (takram's search started at 10,000 km; the shadow maps' resolve too) | from orbit every texel's cloud was further: none was picked, the velocity was taken as none, and the history stood still on the screen while the globe turned or zoomed under it - the clouds smeared out from the middle of the view as it zoomed, and round it in a spiral as it was dragged |
| Seen from 1,000 km and more, the cloud is depth-tested a step nearer than it starts (1 % from 5,000 km) | there everything lies near the far plane, where the depth (reversed) is the difference of two nearly equal small numbers, the terrain's as well as the cloud's: the test fell either way from frame to frame and the clouds seen from orbit sparkled with holes onto the ground below (13,000 km out, still: 5 % of their pixels flickering by 3 grey levels or more; after, none) |
| No haze and no light shafts | both belong to their atmosphere's sky and ground, which the viewer does not draw (it has its own haze) |
| The viewer's night light (its ambient) added as the day fades | their atmosphere has no moon or night sky; the clouds were black over a moonlit ground |

`--clouds clear|few|scattered|broken|overcast|rain` draws a sky of the viewer's own whatever the world says
(`world`, the default, follows the world; `off`, none); `k` (or "clouds" in the monitor) turns them off and on. The testbench
sets its session's sky from the "Sky" list beside the time of day (scattered to begin with).

## The sea

The sea is drawn over the imagery wherever GEBCO says there is water, in one pass over the frame after the clouds'
shade on the ground (`world::Clouds`, `assets/clouds/shaders/ocean`):

- **Waves**: [GodotOceanWaves](https://github.com/2Retr0/GodotOceanWaves)' FFT ocean (MIT), ported: a TMA spectrum
  for the wind with Hasselmann spreading, three cascades of 256 x 256 (tiles of 1987 m, 409 m and 71 m, each a band of
  the spectrum, so no wave length is counted twice), animated by an inverse FFT each frame, foam where crests break.
  The waves' clock loops every 1000 s with no seam, so it stays exact however long the viewer runs. Their heights
  are held to the observations: the spectrum is fully developed past a dimensionless fetch of 12,000, which gives
  Pierson & Moskowitz's significant height within 3 % from 5 to 15 m/s (0.6 m at 5 m/s, 1.2 m at 7, 2.4 m at 10,
  5.4 m at 15), and the amplitudes give the field the spectrum's variance (GodotOceanWaves' made the waves 2.83 times
  too high and as much too steep - 8 m at 7 m/s, the far sea busy and glaring with it).
- **Geometry** (Bruneton et al.'s screen-space grid, their demo BSD-3): a vertex every 8 pixels, each where its ray
  meets the ellipsoid, moved by the waves' displacement read at the level its spacing resolves - near the eye the waves
  stand up, hide the sea behind them and cut the horizon low over the water; past 5 - 8 km they are flat and each
  pixel meets the ellipsoid itself. The sea writes its depth, half a metre below its surface near the eye, so a
  vehicle's hull under the water stays under it.
- **Light**: Bruneton, Neyret & Holzschuch's ocean lighting (Proland, BSD-3). Each pixel reads its cascades' slopes
  and squared slopes at the level its footprint asks for (LEAN mapping): a mean normal and the roughness round it,
  with what the cascades do not resolve taken from Cox & Munk's measured slopes for the wind. So the sea is the same
  sea near and far: the sun's glitter becomes a broad glint from the air and from orbit, with no aliasing and no
  tiling. Patches of rougher and smoother water kilometres across (gusts, slicks) scale the slopes, so the tiles'
  one pattern does not repeat to the horizon.
  - Its terms are the sky reflected with the Fresnel averaged over the slopes, the sun reflected by the slope
    distribution (anisotropic Beckmann with Smith masking), and the clouds' shadow on the sun's part only. The sea is
    not darkened all over as the land is: a shadow dims its glint and its colour, not the sky it mirrors.
  - The clouds in it: where the reflected ray lands on the screen, the frame's own clouds (their resolved image),
    smeared down the screen by the spread the slopes give the reflection - the long streaks under each cloud a rough
    sea makes; elsewhere an overcast's grey for the cover round the eye. The clouds off the screen are not reflected
    (looking down from high up the sea reflects the zenith, and only 2 % of it). 0.1 ms.
  - Light through the crests near the eye (GodotOceanWaves' water shader, after Atlas's wave-peak scattering):
    sunlight entering the back of a wave between the eye and the sun leaves its face blue-green - most in the highest
    crests, the face turned from the sun, the eye looking into it.
  - Then come light scattered up from deep water (clear ocean water's reflectance) and foam: whitecaps as Dupuy &
    Bruneton have them (2012) - the crests compressed past a threshold break, the share of the pixel where the waves'
    horizontal displacement's divergence is below it (its mean and variance, mipmapped like the slopes), the threshold
    set so the whole sea is the cover observed for the wind (Monahan & O'Muircheartaigh 1980: 0.3 % at 7 m/s, 1 % at
    10, 4 % at 15): foam on the crests near the eye, that share of white far off. `--sea-wind <m/s>` sets the wind the
    sea is made for, whatever the weather says.
  - It is lit as the land beside it is: its light as a share of a white surface's in the same light (its apparent
    reflectance), times the light `world::setDaylight` gives the terrain, highlights rolled off rather than clipped.
    It is hazed exactly as the terrain is, so the shore does not show a seam. (Mapped at the clouds' exposure, the
    glint from 10 km up was a white sheet half the view across, and the clouds in front of it looked lit from below.)
- **Depth**: GEBCO 2026's water depth (`ocean/gebco-ocean-2arcmin.png` in the map set, made by
  `tools/ocean_map/make_ocean_map.py` from GEBCO's grid and its type identifiers). The imagery, which shows shallow
  water's own colours (reefs, banks, sand), is seen through the sea as far as the depth lets light down and back:
  turquoise banks stay turquoise and deep ocean is the sea's own navy.
- **Shallows and surf**: each cascade's waves shrink where the water is shallower than about half their length
  (`sqrt(tanh(k d))`), from GEBCO's depth offshore and, within 6 km of the eye, from the distance to the shore: a map
  of it round the eye (512 x 512 texels of 25 m, made on a worker thread from the map set's elevation tiles as the eye
  moves below 8 km, smoothed) and a beach rising at 1.5 %. Where the water is shallower than 1.3 times the waves'
  significant height they break: lines of white water along the shore, running in, broken along their length, and the
  swash at the edge - on shores open to the sea only (400 m out from the shore the water must still deepen: a harbour
  or a lagoon is calm).
- **Coast**: within the shore map round the eye (below 8 km, 6 km each way) the map draws it: a signed distance to
  the coast, smoothed, its zero line the shore - the terrain's mesh, stepping between its land and its sea floor by its
  elevation texels, cut the flats of the bay into jagged polygons - and the sea is depth-tested 1.5 m above the water,
  so only real relief (a cliff, a levee) hides it. Beyond it the terrain draws it. The sea's floor is drawn down to 25 m below sea level (`world::setSeaFloorDepth`:
  Terrarium heights at or below 0 m; 1 m down at the shore, 4 m deeper a texel out - a step from the shore straight to
  25 m left the mesh's slope under the sea most of the way up a beach, and the sea covered the sand), below a storm's
  troughs, and the sea is depth-tested between - half a metre under its
  waves near the eye, 12.5 m down far off - so any land above sea level hides it, however low, and the floor never
  does. Which texels at or below sea level are land is burnt into the map set's elevation tiles
  from OpenStreetMap's land polygons (`tools/ocean_map/sea_mask_tiles.py`; ODbL): without it the stored tiles hold
  polders, salt flats and beaches at 0 m, as the sea is, and the sea flooded them. GEBCO's water share only bounds
  where the sea may be (generously, 3.7 km a texel; it cannot tell a polder from a lake).

What it costs (4K fullscreen, the screen all sea, 25 m over the water): 1.0 ms of GPU time, 0.07 ms for the FFT and
the rest for the grid and its shading; over a coast with surf, 1.2 ms.

The sea's wind is the world's, following it over minutes as a real sea does. It is rebuilt in steps too small to see,
and with no world it is 7 m/s (the open ocean's mean). Without the map the imagery's own sea is drawn as before.
GEBCO's grid is a 4 GB download from gebco.net, so `fetch-maps` does not fetch it; to make the map:

    python tools/ocean_map/make_ocean_map.py GEBCO_2026.nc GEBCO_2026_TID.nc assets/maps/ocean/gebco-ocean-2arcmin.png

(numpy, and h5py for the netCDF-4 elevations). The coastline is burnt into the elevation tiles once, after they are
fetched (OpenStreetMap's land polygons, a 0.9 GB download from osmdata.openstreetmap.de; tiles fetched later need the
same run, and the changed tiles' originals are kept in the backup directory):

    python tools/ocean_map/sea_mask_tiles.py land_polygons.shp assets/maps/s3.amazonaws.com/elevation-tiles-prod/terrarium terrarium-backup

## The frame rate

The target is the display's refresh at its full size: 160 frames a second at 3840 x 2160, fullscreen, with the fire
solver running - and never by drawing less. `tools/viewer_perf/viewer_perf.py` holds four scenes to it, each a world
the simulation flies (or a recording of one) drawn by `flightsim-viewer --stats`:

| Scene | What it is |
| --- | --- |
| fleet | all 35 aircraft with their loads, line abreast at 3,000 m, turning together; the camera 900 m off |
| strike | an F-16C flown into the sea at 60 deg and 233 m/s: 1,270 kg of fuel a fireball, then the fire that burns on; the camera 160 m off |
| hit | a visual recording made by `flightsim-showcase` from `tools/viewer_perf/hit.json`: scripted missile motion and scheduled detonation against a native JSBSim MiG-29A, with native damage and a Flow-solved fuel fireball, a 5 kg/s leak burning down its path, and its sections parting; the camera 70 m off. This fixture measures rendering, not native missile guidance or fuze behavior |
| reheat | an F-16C in full reheat through the transonic from 35 m: its flame, its shock train, its vapour cone |

A scene passes when, after its first five seconds, every second's mean rate is within 1 % of the target and no frame
took more than two refresh intervals (12.5 ms). `--stats` prints, each second, the frames' median and 99th percentile
and how many took longer than `--stats-budget`. The picture is held with the rate: `viewer_perf.py --shots <label>`
replays each scene's recording on its own clock and saves frames at fixed times of it, `--compare` sets two such sets
side by side - what moves from run to run in one viewer (the clouds' temporal noise, the fire's turbulence: up to
0.9 % and 5 % of a frame's pixels more than 8 of 255 apart) is the floor a change is read against.

Measured 2026-10-11 on an RTX 5070 Ti, the sun at midday (`before`: the viewer at 12fb394):

The `now` rows refer to the revision measured in that dated comparison. They are not a fresh benchmark of the
current checkout or proof of viewer compatibility with the later native engagement delivery.

| Scene | | fps (mean) | lowest second | worst frame, ms | frames over 12.5 ms | the viewer's passes on the GPU, ms | the frame's thread outside the draw, ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| fleet | before | 142.0 | 105.2 | 27.7 | 15 of 4988 | 6.07 | 0.35 |
|  | now | 160.0 | 159.5 | 18.0 | 1 of 5619 | 3.81 | 0.34 |
| strike | before | 143.9 | 114.5 | 123.8 | 6 of 3031 | 5.40 | 2.91 |
|  | now | 159.0 | 146.3 | 12.9 | 3 of 3349 | 3.72 | 0.10 |
| hit | before | 154.2 | 119.0 | 18.9 | 6 of 4490 | 3.72 | 3.72 |
|  | now | 160.0 | 158.0 | 19.0 | 2 of 4653 | 2.46 | 0.15 |
| reheat | before | 160.0 | 159.9 | 7.8 | 0 of 3373 | 4.03 | 0.11 |
|  | now | 160.0 | 160.0 | 7.2 | 0 of 3377 | 3.97 | 0.11 |

What changed, none of it what is drawn:

- **The gas round the aircraft is marched only where it can be.** The vapour's and the exhausts' march ran every
  pixel of the window through every aircraft's box - 8.3 million rays at 3840 x 2160, each looking for 40 boxes, to
  find nothing over most of the screen: 2.7 ms with the fleet in view. Each box's outline on the window is now worked
  out (a ray meets a box only inside it), the march runs over the rectangle round them and looks for each box only
  inside its own outline, and the composite lays it on only there: 0.35 ms, the same picture (the texels the
  composite's filter reads round the outlines are still marched).
- **The fire solver has a thread of its own** (world::FlowVolumes). Its recording, its readback and the packing of
  its fields into the texture were in the frame - 3 to 7 ms of the frame's thread with a fireball, spikes to 28 ms -
  and a brick texture of 12 MB was packed again every frame whether or not the solver had stepped. The frame's thread
  now gives the solver its steps and takes the newest packed fields: 0.01 ms.
- **A piece of a broken aircraft shares its aircraft's model.** Each piece read and compiled the whole model file
  again as it broke off: a frame of 100 to 124 ms at a ground strike. The file is read once.
- **The leaves' hash is an eighth full** (it was half), and a ray keeps the leaf its last sample was in: fewer looks
  a sample in the fire's march.

What is left, and why:

- **A large fireball's peak.** For the two seconds a 1.27 t fuel fireball is at its largest and fills a quarter of
  the screen, the rate is 146 to 150: its march costs 3 ms of the GPU then, and the solver - 2.4 to 4.4 ms of the
  same GPU a step for a fire that size, sixty steps a second, measured alone - has to share it. Both are what they
  were: the march's steps and the solver's half-metre cells are the picture's detail and are not coarsened for the
  rate. A warhead's burst with a burning leak (the hit) holds 160.
- **The solver's steps land in some frames.** With several thousand leaves alive a step takes 5 to 7 ms of the GPU
  as the two take turns, and the frame it lands in is late: a few frames a second of 9 to 13 ms through the strike's
  fire. The driver gives neither device a priority over the other (VK_KHR_global_priority: a higher one for the
  frame's queue is refused without elevation, a lower one for the solver's changed nothing - tried and taken out).
- **A first use.** The first time a kind of thing is drawn - a missile leaving its rail, the vapour at a first turn -
  one frame takes 13 to 19 ms; the second time, in the same run, none does.

## Screenshots

`flightsim-viewer.exe --screenshot shot.png [--screenshot-after 5]` saves the
window as PNG after the given number of seconds and exits (any mode: mirror,
`--demo`, `--replay`); the copy is taken from the swapchain, so it works on a
locked desktop and in scripts. `--screenshot-frames 8 --screenshot-every 0.034`
saves a sequence instead - `shot_000.png`, `shot_001.png`, ... that far apart -
to see what moves: a rotor turning at the rpm user code set, a control surface
following its command. `--screenshot-zoom 0.5` zooms the camera while they are
saved (its distance times 0.5 a second; 2 zooms out) and `--screenshot-orbit 30`
turns it round what it looks at (30 degrees a second), as the wheel and a drag do -
to see what the clouds' temporal resolve leaves behind in motion. Saving a frame
takes some 0.1 s, so a sequence that close together is drawn at some 9 frames a
second; to see the viewer in motion at its own rate, save two or three frames
0.5 s apart. In a replay, `--screenshot-clock replay` counts `--screenshot-after`
and `--screenshot-every` in the recording's own time instead: each frame is drawn
at its time exactly, however long saving the last took (a launch every 40 ms:
`--screenshot-clock replay --screenshot-after 5.96 --screenshot-frames 12
--screenshot-every 0.04`). On the window's clock a slow frame left the drawn time
running ahead and snapping back, a missile drawn ahead of its rail, then behind
it. `--view lat,lon,alt,dist[,az,el]` starts with
the free camera looking at a point from `dist` metres - at its height, a point in
the air held there until the view is dragged (it was put on the ground under it)
- (whole-Earth views:
`--view 20,-30,0,30000000,180,80`); above the atmosphere the sky fades to
black. `--camera chase --chase-distance 15` starts right behind the selected
vehicle, also when attached to a trainer's world (which otherwise starts with
a regional view); `--chase-azimuth` and `--chase-elevation` move the camera
around it; `--follow <slot>` starts on that vehicle instead of the first, or goes to it when it appears (a showcase's missile, 1, from its launch).

## If it crashes

`flightsim-viewer.exe` and `flightsim.exe` install a crash handler: an
unhandled exception writes a minidump to `logs/crash/` beside the program
(`<exe>-<date>-<time>.dmp`, openable in Visual Studio or WinDbg) and one line
to stderr / `viewer.log`. Libraries never install one - a trainer owns its own
process.

## Limits (current)

- One machine: the segment is local shared memory (a network relay is a planned `ext/` module).
- One sample aircraft model ships (Cesium Air, fixed surfaces) besides the hangar designs; drop `models/<type>.glb` files next to it for your own types.
- The trainer's physics ground is flat unless `WorldOptions::terrain` is on (then it is the same relief the viewer draws); with it off a low-flying vehicle can appear below hills.
