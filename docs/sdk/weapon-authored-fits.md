# Authored loadout integration

The complete M03 campaign must join source inventory to executed installed
loadouts. A selected compatible carrier for a store does not establish every
authored fit. `tools/validation/weapon_quality/fits.py` derives its denominator
directly from current airframe TOML, `payloads.json`, station payload cards and
store cards. It independently compares that population with `audit.py` coverage.
The current source has 35 airframes, 21 with loadouts, 46 loadouts, 214
station/store bindings and 215 physical stores. A-10C's right outboard dual rail
contains two AIM-9M stores. There are 89 AAM bindings and 119 armed bindings
(120 physical stores), including authored bomb and cruise-missile fits.

Run with the installed SDK interpreter after the same-run production and normal
installation stages. Use a fresh output and retain the complete records folder:

```powershell
python -B tools/validation/weapon_quality/fits.py `
  --prefix D:/ENV/<current-installed-prefix> `
  --coverage <campaign-output>/coverage.json `
  --producer <campaign-output>/production/installed-production-manifest.json `
  --output <campaign-output>/authored-fits.json
```

Every loadout starts with a fresh installed `World` and empty installed carrier.
Only normal `Vehicle.load_loadout(name)` populates its stations. The test checks
the exact payload/station/card/count inventory, then performs actual emergency
jettison of every physical store. Every binding with available guidance, fuze
or warhead additionally receives a fresh armed scene loaded with its complete
original loadout. `Vehicle.strike(stores=[exact native indices], consent=True,
target=None)` selects only that binding, including every position on a multiple
rail. Remaining inventory must remain present and unconsumed.

These armed scenes are **referee integration**. They do not submit Agent
requests or provide consent to an actor request. The uncued native release
executes armed provider initialization, navigation/guidance and finite native
controller decisions without asserting acquisition, illumination, a turn or an
intercept. Guided releases require actual available controller decisions and
finite physical-fin observations joined to entity, slot and generation.
`fleet.py` remains the separate mandatory actor-permitted, modeled-C2-consent,
sensor-acquisition and flight execution gate. This fit campaign cannot replace
it. Bomb fits execute armed native initialization and finite ballistic flight;
their lack of a guidance controller is required, not counted as guided control.

The scene policy is frozen in `fits.py`: 1/120 s native step, frame skip 4,
one worker, seed 41, disabled terrain, **fuel_fraction=0.5 for every carrier in
every scene**, at most 1,350 outer steps for station
operation and 1,350 for release, followed by exactly 60 outer integration steps.
Jettison uses the accepted M02 short untrimmed basic consumer condition:
90 m/s at 1,500 m, zero axis commands, full throttle and retracted gear. Armed
fits use the existing M03 fleet nominal 250 m/s at 6,000 m velocity hold.
These bounded scenes establish installation and execution; they are not an
envelope, performance or measured-equipment calibration pass. Failure retains
the original scene and numerical predicate; there is no per-model retry or
parameter adjustment.

Before execution, a separate source-only mass preflight independently sums
generated XML empty weight and fixed point masses, one half of actual generated
internal FUEL tank **capacities**, and every complete authored store's dry mass
plus fuel, including count2 racks. It binds the exact TOML, XML and card hashes
and compares against source `mass.max_takeoff` / `mass.mtow` where declared.
All 44 currently declared-limit loadouts fit this single global loading policy;
A-10C/external tanks has a 2,264.6328 kg source-arithmetic margin. Both J-10A
loadouts have no declared MTOW: their limit availability is `unavailable`, MTOW
and margin are `null`, and `passed` is `null`, with the exact missing declaration
reason. Those rows do not become MTOW passes. This preflight predicts loading
feasibility; it does not replace actual native load acceptance or quantitative
native mass-conservation tests.

`fsim.sim-m03.installed-authored-fits.v1` receipts contain the independently
derived population, exact source hashes, same-run producer and generated
product maps, actual installed facade/native-extension/DLL paths and hashes,
actual imported `fsim.world` and `fsim._state` module identities, an explicit
source-copy-bound runtime map of consumed store/station/fit JSON and Python
files, and an epoch digest joining them. Runtime inputs are checked before
execution, after execution and during standalone admission. Paths come only
from the independently derived population and known module objects; no broad
installed-tree discovery is used. Each scene retains a separate hash-bound raw
JSON record, exact inventory, native request/activity and weapon correlations,
generation/slot identity, every post-release outer-step body state and control
sample, and original native outcomes. Output is saved incrementally. A failed
scene retains its partial raw record and leaves the campaign receipt failed.

`fits.validate_receipt(receipt, independently_derived_population, producer)`
is the fail-closed report integration interface. It returns admitted counts
only after reconciling every unique loadout and armed binding, installed product
bytes, raw artifact bytes, exact counts, consumed inventory, armed/inert meaning,
native generations/correlations, controller execution and finite native bodies.
Use the same-run producer and a fresh `fits.derive_population()`; do not trust
the population copied into the receipt as its own reference. The parent complete
campaign must invoke this command and its final report must invoke this
validator before declaring the authored-fit gate closed.

Native time ordering is checked explicitly. `World.step` first integrates the
old bodies, advances time, then performs `flyReleases`. A newly released body
therefore has a finite body snapshot at its launch boundary, but no preceding
controller decision. Every subsequent completed guided integration interval in
this fixed live-control scene must have an available controller decision from
the final native substep (`sample_time - dt`), no earlier than that specific
release, with advancing timestamps. Controller dropout fails this bounded
scene; one stale available sample cannot admit it. Native termination during
the fixed live-body interval remains a recorded failure, not a synthesized
continuation. Warmup/ripple admission requires exactly one finite state for
each known body at every outer boundary at which it exists, including the first
dual-rail member before the second is released. Missing, duplicate, unknown
and prelaunch body rows fail admission.

Pure tests do not import the SDK or execute a World:

```powershell
python -B -m unittest discover -s tools/validation/weapon_quality -p test_fits.py -v
```

They exercise actual source denominator discovery and corrupt otherwise valid
synthetic evidence to check missing/duplicate rows, lost dual-rail counts,
changed sources/products/raw bytes, inert releases relabeled armed, absent
controller decisions, wrong generations/activities, unconsumed stores, invalid
fins, missing steps and nonfinite/diverged bodies. Added review regressions
cover uniform bound fuel, capacity-based complete-loadout mass arithmetic and
unavailable MTOWs, operational JSON/world/state byte mutations, negative/stale/
frozen decision times, decision dropout and complete dual-rail ripple states.
They verify admission logic;
they are not native execution evidence.

No current authored loadout belongs to a rotorcraft. The 28 accepted M02 rotor
deferrals retain policy hash
`d52e0e76afdb93ddbe05513f8a04253812271cf902f865d92f03527bb11e3645`.
The 14 configurations without authored loadouts remain explicit source
applicability rows; they receive no invented loadout or new rotor dynamic
acceptance. Pod rows establish authored carriage/release and finite native
flight with positive finite mass telemetry; this tool does not prove quantitative
native pod/carrier mass conservation. Authored component inapplicability reasons remain exact; this tool
does not add pod sensors, guns or arbitrary weapons.
