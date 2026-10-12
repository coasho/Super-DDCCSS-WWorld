# Architecture A6: store-to-flight-model state ownership

## Intent and scope

Starting from `39be3c1`, give carried-store flight bindings a private owner without changing public API/ABI,
principal boundaries or simulation behavior. This is the first World state-owner slice. Conversation is Chinese;
source and documentation are English. Initial implementation review preceded the user's request to deliver A6.

## Selected ownership

`World::Entry` holds a `session::StoreFlightBinding` by value. The new private header/source own station property
handles, cached mass and drag revisions, the between-step weight-room stamp/value, drag points/qbar and the last
mass revision reported to the control host. The binding borrows property nodes; it owns no World, model, Host,
worker or registry and retains no pointer/reference to those objects.

World retains entities/model lifetime, aircraft-level ordered pointmass-name caches, permissions, all traversal,
`loadTick_`, model acquisition, host publication and mass-change transactions. The existing session World methods
remain available. Native SDK adapters, public headers, Python, flight models, mission rules and resources freeze.

Grouping fields alone would leave their invariants scattered. Moving whole store orchestration would also move
entity creation, authority and release/damage timing. The selected middle scope moves only binding-local operations:
station discovery, mass/drag synchronization, weight-room arithmetic, transfer arithmetic and force accumulation.

## Private interface

- `bind(const ms::Host&, sim::FlightModel&, const std::vector<std::string>* names)` clears station handles and
  initializes the mass revision, then resolves the original ordered station names; null names means none available.
- `applyMasses(const ms::Host&, sim::FlightModel&)` refreshes carriage drag first, then writes changed station mass.
- `weightRoomKg(double maximumKg, double weighedKg, double carriedKg, unsigned tick)` owns the original stamp/math;
  World retains the non-finite early return before obtaining carried mass or updating the stamp.
- `hasStations()`, `loadChanged()` and `loadPublished()` expose only the predicates/report acknowledgement needed
  by the existing World traversal and host publication.
- `transferFuel(ms::Host&, sim::FlightModel&, double dt)` keeps energy/room/draw/refuel order. World keeps the
  original no-stations/no-external-fuel guard before obtaining the model.
- `addDrag(const sim::VehicleState&, sim::FlightModel&, double push[3], double turn[3]) const` adds to the caller's
  existing forces in the original order and returns true only when the original positive-q path contributed.

## Preservation contract

- Initial host load precedes Mission Host bind, which precedes station property binding.
- Reset neither rebinds nor clears binding state/revisions; removal destroys it with Entry. Slot reuse creates a
  fresh Entry/binding on the existing reset model. This slice preserves the current loaded-reset semantics.
- Store revision drives drag; store plus fuel revision drives mass. Empty station bindings do not advance mass
  revision. Qbar is looked up only for nonempty drag and an invalid cached handle.
- Rejected and successful load calls both retain the original global mass-application side effect.
- Transfer uses `min(room, 10.0 * dt)` and `room > 0.01`; it does not multiply dt by frameSkip. Draw precedes refuel,
  whose returned amount remains ignored. Preserve all constants and numeric expression/accumulation ordering.
- Step remains transfer, masses, pool step, then old-tick modulo-eight or changed-mass host load, host afterStep,
  Mission step, sensing/engagements/releases/damage. Force accumulation remains effects, damage, store drag, guidance.
- Release/damage begin/apply/reaction/end mass-change transactions stay in World and WorldDamage unchanged.

## Acceptance

Characterize current loaded reset and loaded-history slot reuse before production extraction. Retain a normalized
field-bit replay before/after for between-step edits/refusal, tick boundaries, fuel draw, asymmetric release, reset
and reuse, including worker/frameSkip/drag variants. Do not compare structure padding or replace current semantics
with assumed clean-reset behavior. Compare exact moved operations/remainders and protected hashes.

Build `ucrt64-headless` with warnings as errors using existing tools under `D:/ENV` and temporary files under
`D:/ENV/tmp`. Run stores/mass/drag/load, authority/ABI/binding and architecture gates, then the full applicable
functional suite and installed consumers. Check 1,338 SDK export names, Python runtime identities, distribution
provenance, documentation and diff scope. Unchanged Hangar generation, renderer/GPU and remote CI are outside this
acceptance; no dependency installation or resource regeneration is required.
